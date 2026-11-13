# SPDX-License-Identifier: GPL-2.0-or-later
"""Scan a mounted slide or a film strip on the PrimeFilm 1800 AFL into TIFF files (see pf1800.py).

  python3 pf1800_scan.py out/slide                  # 1800 lines at 16 bits per sample
  python3 pf1800_scan.py out/slide --autofocus      # quick autofocus (about 20 s; 7 s on a strip) before the scan
  python3 pf1800_scan.py out/slide --flatfield      # open and dark references (about 10 s), then a corrected scan
  python3 pf1800_scan.py out/slide --dry-run        # prints every transfer and moves nothing
  python3 pf1800_scan.py out/roll --strip           # a strip the feeder has caught: every frame, then eject
  python3 pf1800_scan.py out/roll --strip --frames 2 --no-eject
  python3 pf1800_scan.py out/roll --strip --rewind  # feed a strip left in back to its start, then scan it again
  python3 pf1800_scan.py out/slide --rgb            # read R, G and B and save their mean (the slower path)
  python3 pf1800_scan.py out/slide --colour         # read R, G and B and save them as an RGB TIFF (experimental)

Output is greyscale by default (black and white film): the scanner reads the red channel alone at 16 bits (the front
end in mono mode), so an 1800-line scan streams in about 18 s instead of 37 s. The gain for that is picked from a reading
of open light first (one to four short scans of the lit calibration window, about 4 s each); --gains sets it instead.
--rgb reads R, G and B at 16 bits and saves their mean, as before; --depth 8 implies --rgb, and its levels clip green and
blue on negatives. --colour reads as --rgb does but keeps the three channels (RGB TIFF and preview), at the default
front-end levels unless --gains: no gain is picked for colour film yet, and nothing is done about a colour negative's
orange mask.

Writes <prefix>_raw.bin (the lines exactly as read: red samples, or all three colours with --rgb or --colour),
<prefix>.tif (greyscale, or RGB with --colour; full resolution, mirrored, 16 or 8 bits per sample, 1800 dpi) and
<prefix>_preview.png (contrast-stretched, 8 bits).
The scanner is parked (carriage home, lamp off) on the way out, also after an error or Ctrl-C.

Strip mode aligns the first whole frame to the start of the gate (from the clear gap between frames, so it is meant for
negatives), scans it, advances one frame pitch (--pitch-mm, default 38) closed loop and re-aligns, until --frames are done
or the strip ends (page B bits 7 and 6 both clear, or no frame edge is left), then ejects it forward through the exit slot.
Frames go to <prefix>_frameNN*; they are negatives. The alignment scan before each frame
is kept as <prefix>_alignNN*. A frame found already in the gate with its leading gap gone (fed a little too far) is
scanned where it is if at most about 2 mm of it is lost. When no frame edge is found, the alignment scan is kept as
<prefix>_alignNN_failed* and the gate is scanned as it stands (<prefix>_frameNN_unaligned*).
"""

import argparse
import sys
import time
from concurrent.futures import ThreadPoolExecutor
from dataclasses import dataclass
from pathlib import Path

from pf1800 import (ALIGN_SCAN, BLACK_TARGET, CAL_DARK_AT, CAL_LINES, DEFAULT_LEVELS, FRAME_SCAN, LINE_PERIOD, LONG_LINE,
                    PF1800, RESCAN_LOSS, EndOfFilm, Frame, ScannerError, align_plane, falloff_gain, frame_columns, is_leader,
                    is_open_light, line_exposure, png_bytes, stretch, tiff_bytes)

# Autofocus sweep (start, step, count, in lens units from home) for a strip: the feeder holds a strip near focus at +0 to
# +3 (a slide's emulsion is at +40, so it keeps autofocus()'s full 0 to 75), so 0 to 15 is usually enough and follows curl.
# About 7 s instead of 21.
STRIP_FOCUS = (0, 3, 6)
STRIP_FOCUS_MORE = 10  # further sweep positions, while the last is the sharpest (roll39 was still rising at +15; to +45)


def triple(text):
    values = tuple(int(v) for v in text.split(","))
    if len(values) != 3:
        raise argparse.ArgumentTypeError("expected three comma-separated values")
    return values


def save(frame: Frame, prefix: Path, colour=False):
    """Write <prefix>_raw.bin, <prefix>.tif and <prefix>_preview.png; greyscale, or RGB with `colour` (a colour frame)."""
    prefix.parent.mkdir(parents=True, exist_ok=True)
    (prefix.parent / f"{prefix.name}_raw.bin").write_bytes(b"".join(frame.lines))
    channels = 3 if colour and frame.channels == 3 else 1
    rows = frame.aligned() if channels == 3 else frame.grey(frame.aligned())
    width = len(rows[0]) // channels
    (prefix.parent / f"{prefix.name}.tif").write_bytes(tiff_bytes(rows, width, frame.depth, frame.dpi, channels=channels))
    preview = stretch(frame.to_8bit(rows), channels=channels)
    (prefix.parent / f"{prefix.name}_preview.png").write_bytes(png_bytes(preview, width, channels=channels))
    print(f"saved {prefix}.tif ({width} x {len(rows)}, {frame.depth}-bit {'RGB' if channels == 3 else 'greyscale'}, "
          f"{frame.dpi} dpi), {prefix}_raw.bin, {prefix}_preview.png")


# Open light is read (a seek home, a 64-line scan and the way back out, about 8 s) before the first SAG_EVERY_FROM frames, while
# the lamp settles, then every `sag_every` frames; the frames between take the trend of the last two readings, held within
# SAG_EXTRAPOLATE of the last. Simulated on roll38 and roll41 to roll44 (the readings along each roll, every third taken):
# the estimate is off by 0.25 to 0.7 % on average and 1 to 4 % at most, where the reading itself is steady to 0.1 %.
SAG_EVERY, SAG_EVERY_FROM, SAG_EXTRAPOLATE = 3, 2, 0.03

@dataclass
class ScanOptions:
    """What to scan and how; shared by the command line and the GUI. `lines` and `approach` apply to a slide only (a strip
    frame always uses FRAME_SCAN)."""

    strip: bool = False
    frames: int = 0  # strip: frames to scan (0 = to the end of the strip)
    pitch_mm: float = 38.0
    eject: bool = True
    dpi: int = 1800  # 1800, or 300 for a fast proof scan
    lines: int | None = None
    approach: int = 1550
    depth: int = 16
    rgb: bool = False  # read R, G and B (the mean is the grey picture) instead of red alone
    colour: bool = False  # read R, G and B and keep them: the frames handed to on_frame are colour (channels 3)
    gains: tuple | None = None
    offsets: tuple | None = None
    warmup: float = 10.0
    autofocus: bool = False
    focus: int | None = None
    flatfield: bool = False
    proof: bool = False  # strip: scan each frame at 300 dpi first and set its 1800 dpi gain from it
    sag: bool = True  # strip: read open light before each frame and scale the frame to frame 1's (pf1800.OPEN_PERCENTILE)
    line: int = LONG_LINE  # strip: the frames' line period byte, with the strip gain picked for it (pf1800.line_exposure())
    sag_every: int = SAG_EVERY  # strip, with sag: frames between open-light readings (the rest are extrapolated), 1 = every frame
    predict: bool = True  # strip: advance to the edge the frame pitch predicts and check it with a short scan, not an alignment
    falloff: bool = True  # strip, with sag: flatten the light along the CCD from the first reading (pf1800.falloff_gain())
    settle: bool = True  # grey: warm the lamp until its open light is steady (pf1800.LAMP_SETTLE), `warmup` at least

    @property
    def step(self):
        return 1800 // self.dpi

    @property
    def grey(self):
        return self.depth == 16 and not self.rgb and not self.colour

    def check(self):
        """Raise ValueError for a combination the scanner cannot do."""
        if (self.step > 1 or self.proof) and not self.grey:
            raise ValueError("a 300 dpi or proof scan needs a grey scan (no rgb or colour)")
        if self.rgb and self.colour:
            raise ValueError("rgb (the mean of R, G and B) and colour (all three kept) exclude each other")
        if self.proof and not self.strip:
            raise ValueError("proof is for strips")
        if self.proof and self.gains is not None:
            raise ValueError("proof sets the gain itself; leave out the gains")


MONITOR_REST = (0, 0.5, 1, 2, 4, 8)  # --monitor-rest: minutes after the lamp is relit at which to read


def monitor_reading(scanner):
    """The scanner's state for --monitor (probe55's reading): open light in the lit calibration window in 16-bit mono red,
    green and blue at gain 0 (99.5th percentile of CAL_LINES lines), and the dark carriage body in red at gain 15 and
    offset 160 (median). Returns the four levels and each one's column means ({"R", "G", "B", "dark"}, see
    pf1800.frame_columns()); zeros and no columns in a dry run."""
    import numpy as np

    _, (_, g0, b0) = DEFAULT_LEVELS[16]
    levels, columns = [], {}
    for channel, name in enumerate("RGB"):
        levels.append(scanner.open_level(channel) or 0.0)
        columns[name] = scanner.open_columns
    frame = scanner.scan(CAL_LINES, CAL_DARK_AT, (15, 15, 15), (160, g0, b0), 0.0, depth=16, grey=True, channel=0)
    levels.append(float(np.median(np.frombuffer(b"".join(frame.lines), "<u2"))) if frame.lines else 0.0)
    columns["dark"] = frame_columns(frame) if frame.lines else None
    return tuple(levels), columns


class RollSession:
    """The levels a roll's frames share, and the scan of one frame with them: the lamp settle, the gain (from open light,
    then a strip's from its first alignment), the black offset, the sag scale and the falloff gain. scan_frames() runs a
    whole slide or strip with it; a caller that places the film itself (a manual drive: PF1800.jog(), seek_frames()) makes
    one per strip and calls prepare(), strip_levels() and then scan() for each frame it captures.

    on_frame, progress, before_scan and log are as for scan_frames()."""

    def __init__(self, scanner, opts, on_frame=None, progress=None, before_scan=None, log=print):
        opts.check()
        self.scanner, self.opts, self.on_frame = scanner, opts, on_frame
        self.progress, self.before_scan, self.log = progress, before_scan, log
        self.step = opts.step
        self.lines, self.approach = FRAME_SCAN if opts.strip else (opts.lines or 1800 // self.step, opts.approach)
        self.gains, self.warmup = opts.gains, opts.warmup  # warmup: what is left of it for the next scan
        self.calibration = None
        self.frame_line = LINE_PERIOD  # the frame scans' line period: a strip's, once its gain is picked for it
        self.black = {"offsets": None, "slope": None}  # the red offset kept at BLACK_TARGET (strips, see pf1800.black_offset())
        # open light at frame 1, this frame's scale (pf1800.OPEN_PERCENTILE), and the falloff gain from frame 1's reading
        self.sag = {"on": False, "first": None, "scale": 1.0, "flat": None, "reads": []}  # reads: (frame n, level)

    def prepare(self):
        """Before the first frame: a slide's autofocus (a strip focuses on its first frame, once it is aligned), the lamp,
        the gain from open light and the flat-field references, as the options ask."""
        scanner, opts, log = self.scanner, self.opts, self.log
        if opts.focus is not None:
            scanner.focus = opts.focus
        elif opts.autofocus and not opts.strip:
            log(f"autofocus: lens +{scanner.autofocus(self.approach + self.lines // 2)} units")
        if opts.grey and opts.settle:
            scanner.warm_lamp(self.warmup)
            self.warmup = 0.0
        if opts.grey and self.gains is None:
            code = scanner.grey_gain(self.warmup)
            self.gains, self.warmup = (code,) * 3, 0.0
            log(f"grey gain {code} (from open light)")
        if opts.flatfield:
            self.calibration = scanner.calibrate(opts.depth, self.gains, opts.offsets, self.warmup, grey=opts.grey)
            self.warmup = 0.0

    def strip_levels(self, align):
        """A strip's levels from an alignment scan of it (`align`, pf1800.align_scan()), after prepare(): one gain for the
        whole roll from its brightest part, the film base, the same along the roll, so every frame shares one gain and one
        level scale (only when the gain was picked from open light, without the proof, which sets each frame's own, and
        without flat-field correction, calibrated at that gain); then the black offset at that gain (the fixed offsets clip
        the densest negative to 0 at gain 15, roll36e), and the sag correction on."""
        scanner, opts, step = self.scanner, self.opts, self.step
        plain = opts.grey and not opts.proof and self.calibration is None and step == 1
        if plain and opts.gains is None:
            code = scanner.align_gain(align, line_exposure(opts.line))
            if code is not None:
                self.log(f"strip gain {code} from the first alignment (open light gave {self.gains[0]})")
                self.gains = (code,) * 3
                self.frame_line = opts.line
        if plain and opts.depth == 16 and opts.offsets is None:
            found = scanner.black_offset(self.gains, ALIGN_SCAN[1])
            if found is not None:
                code, slope = found
                _, (_, g0, b0) = DEFAULT_LEVELS[16]
                self.black.update(offsets=(code, g0, b0), slope=slope)
        self.sag["on"] = opts.sag and plain and opts.depth == 16

    def _before(self):
        if self.before_scan:
            self.before_scan()

    def scan(self, n, kind="frame", warmup=0.0, place=True):
        """Scan frame `n` (0 for a slide) of `kind` ("frame" or "unaligned") as it lies, hand it to on_frame and return it
        (None when nothing was read). With `place`, a strip frame that the scan shows lying too far in or fed too far is put
        in place and scanned again, once; a caller that placed the frame by hand passes False."""
        scanner, opts, log, sag, black = self.scanner, self.opts, self.log, self.sag, self.black
        use = self.gains
        if opts.proof:  # the 300 dpi proof of this frame sets its 1800 dpi gain
            self._before()
            shot = scanner.proof_frame(self.approach)
            if shot.lines:
                if self.on_frame:
                    self.on_frame(n, shot, "proof", None)
                use = (scanner.proof_gain(shot),) * 3
                log(f"frame gain {use[0]} (roll gain {self.gains[0]})")
            warmup = 0.0
        if sag["on"]:  # open light now, against frame 1's: the frame (and a rescan of it) is scaled back to that level
            self._before()
            reads = sag["reads"]
            if n <= SAG_EVERY_FROM or len(reads) < 2 or n - reads[-1][0] >= opts.sag_every:
                level = scanner.open_level()
                if level:
                    reads.append((n, level))
            else:  # between readings: carry the trend of the last two on, within SAG_EXTRAPOLATE of the last (a seek home
                (n0, l0), (n1, l1) = reads[-2:]  # and back costs about 8 s a frame; the sag is slow, see SAG_EVERY)
                level = min(max(l1 + (l1 - l0) / (n1 - n0) * (n - n1), l1 * (1 - SAG_EXTRAPOLATE)), l1 * (1 + SAG_EXTRAPOLATE))
            if level:
                if sag["first"] is None:
                    sag["first"] = level
                    if opts.falloff:
                        sag["flat"] = falloff_gain(scanner.open_columns)
                sag["scale"] = sag["first"] / level
                if abs(sag["scale"] - 1) >= 0.005:
                    log(f"open light {level:.0f} ({level / sag['first']:.3f} of frame 1's): frame scaled by "
                        f"{sag['scale']:.3f}")

        def full_scan(warmup):
            self._before()
            scanner.line = self.frame_line
            try:
                frame = scanner.scan(self.lines, self.approach, use, black["offsets"] or opts.offsets, warmup,
                                     progress=self.progress, depth=8 if self.step > 1 else opts.depth, grey=opts.grey,
                                     step=self.step)
            finally:
                scanner.line = LINE_PERIOD  # the other scans keep theirs: open light would saturate
            if black["offsets"] and frame.lines:  # take the black level off, and move the offset toward BLACK_TARGET
                level = scanner.frame_black(frame)
                if level is not None:
                    frame.black = level
                    code, rest = black["offsets"][0], black["offsets"][1:]
                    new = min(max(int(round(code - (BLACK_TARGET - level) / black["slope"])), 0), 255)
                    if new != code:
                        log(f"black level {level:.0f}: offset {code} -> {new}")
                        black["offsets"] = (new, *rest)
            if sag["on"] and frame.lines:
                frame.scale, frame.flat = sag["scale"], sag["flat"]
            return frame

        frame = full_scan(warmup)
        # the frame scan sees every picture row, the alignment only some: when it shows the frame lying too far in (pale
        # picture beside a gap fooled the alignment) or fed too far (its left end lost), put the frame's edge in place and
        # scan it again, once
        off = None
        if opts.strip and kind == "frame" and place:
            off = scanner.late_frame(frame)
            off = scanner.early_frame(frame) if off is None else off
        if off is not None:
            where = "too far in" if off > 0 else "fed too far"
            log(f"frame {n} lies {where} (its left edge at {off}); putting it in place and scanning it again")
            at = scanner.place_edge(off)
            if at is not None and abs(at - off) < RESCAN_LOSS:
                log(f"frame {n}: the film did not move (edge at {at}); keeping the first scan")
            else:
                log(f"frame {n} edge now at {'?' if at is None else at}")
                frame = full_scan(0.0)
        if frame.lines:
            if self.calibration is not None:
                frame = self.calibration.apply(frame)
            if self.on_frame:
                self.on_frame(n, frame, kind, self.calibration)
            return frame
        return None


def scan_frames(scanner, opts, on_frame, on_align=None, progress=None, before_scan=None, cancelled=None, log=print,
                between=None):
    """Scan one slide, or every frame of a strip the feeder has caught, on an open `scanner`.

    on_frame(n, frame, kind, calibration) gets each finished frame: n counts from 1 (0 for a slide), kind is "frame",
    "unaligned" (no frame edge was found: whatever was in the gate) or "proof" (the 300 dpi look that set the gain), and
    calibration is the flat-field reference applied to it, or None. on_align(n, frame, failed) gets each alignment scan.
    before_scan() runs right before every stream starts, so the caller can finish work that would compete with the USB
    transfer for the CPU. progress(fraction) is passed on to the scan, cancelled() is polled between frames (a frame
    under way is finished). between(n), if given, runs on a strip before frame 1 (n 0) and after each frame scan, with
    the film where it is (--monitor). Ejects the strip at the end if opts.eject. Returns the number of frames scanned."""
    roll = RollSession(scanner, opts, on_frame, progress, before_scan, log)
    roll.prepare()

    def aligned(n, failed=False):  # every alignment scan is kept, to check later how each frame was placed
        frame = scanner.failed_align if failed else getattr(scanner, "last_align", None)
        if on_align and frame is not None and frame.lines:
            on_align(n, frame, failed)

    if not opts.strip:
        roll.scan(0, "frame", roll.warmup)
        return 1
    try:
        scanner.align_frame(warmup=roll.warmup)
    except EndOfFilm:
        aligned(1, failed=True)  # no frame edge before the first frame: keep that scan too, to see what the gate held
        if is_leader(align_plane(scanner.failed_align)):
            raise EndOfFilm("the gate holds only a roll's fogged leader: cut the leader off and load the film again") from None
        raise
    aligned(1)
    if opts.autofocus and opts.focus is None:  # on frame 1, not on whatever the gate held at load
        sweep = dict(zip(("start", "step", "count"), STRIP_FOCUS))
        log(f"autofocus: lens +{scanner.autofocus(roll.approach + roll.lines // 2, **sweep, more=STRIP_FOCUS_MORE)} units")
    roll.strip_levels(getattr(scanner, "last_align", None))
    n, kind = 0, "frame"
    if between:
        between(0)
    while not (cancelled and cancelled()):
        n += 1
        log(f"frame {n}" if kind == "frame" else f"frame {n} (unaligned: whatever is in the gate)")
        frame = roll.scan(n, kind)
        if between:
            between(n)
        if (opts.frames and n >= opts.frames) or (scanner.dry_run and n >= 2):  # a dry run shows one advance
            break
        try:
            scanner.advance_frame(opts.pitch_mm, upto=scanner.next_frame_edge(frame) if frame else None,
                                  frame=frame if opts.predict else None)
            aligned(n + 1)
            kind = "frame"
        except EndOfFilm as e:
            failed = getattr(scanner, "failed_align", None)
            if failed is not None:
                aligned(n + 1, failed=True)  # no frame edge: keep the alignment scan
            if failed is None or is_open_light(align_plane(failed)) or kind == "unaligned":
                log(f"end of the strip after {n} frame(s): {e}")
                if failed is not None:  # scan whatever is in the gate as it stands
                    log(f"frame {n + 1} (unaligned: whatever is in the gate)")
                    roll.scan(n + 1, "unaligned")
                    n += 1
                break
            # no frame edge, but film fills the gate (a blank or pale frame): scan it as it lies and go on, so the rest of
            # the roll is not lost; a second one in a row ends the strip
            log(f"no frame edge with film in the gate after frame {n}; scanning it as it lies and going on")
            kind = "unaligned"
    if opts.eject:
        log("ejecting the strip")
        scanner.eject()
    return n


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("prefix", type=Path, help="output path prefix, e.g. out/slide")
    ap.add_argument("--lines", type=int, default=None, help="lines to scan (default 1800, or 300 with --dpi 300)")
    ap.add_argument("--dpi", type=int, choices=(300, 1800), default=1800,
                    help="300: a fast proof scan (grey only; every 6th pixel and motor step); experimental; 8-bit")
    ap.add_argument("--proof", action="store_true",
                    help="strip: scan each frame at 300 dpi first and set its 1800 dpi gain from it (grey only)")
    ap.add_argument("--approach", type=int, default=1550, help="steps past the reference edge where the scan starts")
    ap.add_argument("--gains", type=triple, default=None,
                    help="AFE gains R,G,B (each 0..15; default: picked from open light, or per --depth with --rgb)")
    ap.add_argument("--offsets", type=triple, default=None, help="AFE offsets R,G,B (default per --depth)")
    ap.add_argument("--line", type=lambda v: int(v, 0), default=LONG_LINE, metavar="BYTE",
                    help=f"strip: the frames' line period byte, 0x73 to {LINE_PERIOD:#x}: lower gathers more light and is "
                         f"slower, the gain picked to match (default {LONG_LINE:#x}, about twice the light and the time)")
    ap.add_argument("--no-falloff", action="store_true",
                    help="strip: do not flatten the light along the CCD (about 5 to 12 %% at the frame's ends)")
    ap.add_argument("--warmup", type=float, default=10.0,
                    help="lamp warm-up in seconds (grey: at least this, then until open light is steady; see --no-settle)")
    ap.add_argument("--no-settle", action="store_true",
                    help="grey: warm the lamp for --warmup seconds only, not until its open light is steady (up to 6 "
                         "minutes from cold)")
    ap.add_argument("--depth", type=int, choices=(8, 16), default=16, help="bits per sample (default 16; 8 implies --rgb)")
    ap.add_argument("--rgb", action="store_true", help="read R, G and B and save their mean (about 2.5 times slower)")
    ap.add_argument("--colour", action="store_true",
                    help="read R, G and B and save an RGB TIFF (as slow as --rgb; experimental: default levels)")
    ap.add_argument("--autofocus", action="store_true", help="run a quick autofocus in the middle of the frame first")
    ap.add_argument("--focus", type=int, default=None, help="lens units from home (skips --autofocus)")
    ap.add_argument("--flatfield", action="store_true", help="calibrate (open and dark references) and correct the scan")
    ap.add_argument("--strip", action="store_true", help="scan the frames of a film strip the feeder has caught")
    ap.add_argument("--frames", type=int, default=0, help="strip: frames to scan (default 0 = to the end of the strip)")
    ap.add_argument("--pitch-mm", type=float, default=38.0, help="strip: frame pitch for the advance (default 38)")
    ap.add_argument("--no-eject", action="store_true", help="strip: leave the strip in after the last frame")
    ap.add_argument("--rewind", action="store_true",
                    help="strip: first feed a strip left in (--no-eject) back to its start, to scan it again")
    ap.add_argument("--monitor", action="store_true",
                    help="strip: before frame 1 and after every frame, read open light (R, G, B) and the dark level into "
                         "<prefix>_monitor.txt, with their column means in <prefix>_monitor_columns.txt (about 16 s a "
                         "frame), to follow the scanner's drift along a roll")
    ap.add_argument("--monitor-rest", type=float, default=0.0, metavar="MIN",
                    help="with --monitor: after the strip, turn the lamp off for MIN minutes, then read again at "
                         + ", ".join(str(m) for m in MONITOR_REST) + " minutes after relighting")
    ap.add_argument("--sag-every", type=int, default=SAG_EVERY, metavar="N",
                    help="strip: read open light every N frames and extrapolate in between (default %(default)s; 1 = every "
                         "frame)")
    ap.add_argument("--no-predict", action="store_true",
                    help="strip: align every frame with a full alignment scan. By default a frame whose own left edge shows "
                         "is followed by a feed to the edge its pitch predicts, checked by one short scan (about 10 s a "
                         "frame faster; the full alignment follows where the check disagrees)")
    ap.add_argument("--no-sag", action="store_true",
                    help="strip: do not read open light before each frame to scale it to frame 1's level (the lamp "
                         "sags about 14 %% along a roll; the reading takes a few seconds a frame)")
    ap.add_argument("--dry-run", action="store_true", help="print the transfers, send nothing")
    ap.add_argument("--yes", action="store_true", help="do not wait for Enter before moving")
    args = ap.parse_args(argv)

    opts = ScanOptions(strip=args.strip, frames=args.frames, pitch_mm=args.pitch_mm, eject=not args.no_eject,
                       dpi=args.dpi, lines=args.lines, approach=args.approach, depth=args.depth, rgb=args.rgb,
                       colour=args.colour, gains=args.gains, offsets=args.offsets, warmup=args.warmup,
                       autofocus=args.autofocus, focus=args.focus, flatfield=args.flatfield, proof=args.proof,
                       sag=not args.no_sag, sag_every=max(1, args.sag_every), predict=not args.no_predict, line=args.line,
                       falloff=not args.no_falloff, settle=not args.no_settle)
    try:
        opts.check()
    except ValueError as e:
        ap.error(str(e))
    if args.rewind and not args.strip:
        ap.error("--rewind is for --strip")
    progress = None if args.dry_run else lambda f: print(f"\r{f:4.0%}", end="", flush=True)
    try:
        with PF1800(dry_run=args.dry_run) as scanner:
            kind = scanner.film_type()
            print(f"film holder sensor: {kind}")
            if args.strip and not args.dry_run and kind != "strip" and not args.rewind:
                raise ScannerError("--strip needs a strip caught by the feeder (page B bit 7 or 6)")
            version = scanner.firmware_version()
            print(f"firmware version: {version}")
            if not args.dry_run and not args.yes:
                input(f"{'Strip in the feeder' if args.strip else 'Slide in the slot'}? Press Enter to scan (Ctrl-C to stop) ")
            if args.rewind:
                print(f"rewind: {scanner.rewind()} coarse units back")
                if not args.dry_run and scanner.film_type() != "strip":
                    raise ScannerError("after the rewind the feeder no longer holds a strip (page B bit 7 or 6)")

            saver = ThreadPoolExecutor(1)  # frames are written while the strip feeds and aligns, never while one streams
            pending = []

            def drain():  # the previous save must be done: it would compete with the USB stream for the CPU
                for job in pending:
                    job.result()
                pending.clear()

            def prefix_for(n, kind):
                if not args.strip:
                    return args.prefix
                name = f"{args.prefix.name}_frame{n:02d}"
                return args.prefix.parent / (name + {"frame": "", "unaligned": "_unaligned", "proof": ""}[kind])

            def on_frame(n, frame, kind, calibration):
                print()
                prefix = prefix_for(n, kind)
                if kind == "proof":
                    prefix = prefix.parent / f"{prefix.name}_proof"
                elif calibration is not None:
                    prefix.parent.mkdir(parents=True, exist_ok=True)
                    calibration.save(prefix.parent / f"{prefix.name}_cal.npz")
                    print(f"flat-field corrected; references saved to {prefix}_cal.npz")
                pending.append(saver.submit(save, frame, prefix, opts.colour and kind != "proof"))

            def on_align(n, frame, failed):
                suffix = "_failed" if failed else ""
                save(frame, args.prefix.parent / f"{args.prefix.name}_align{n:02d}{suffix}")

            between = None
            if args.monitor:
                args.prefix.parent.mkdir(parents=True, exist_ok=True)
                monitor = open(args.prefix.parent / f"{args.prefix.name}_monitor.txt", "w")
                profiles = open(args.prefix.parent / f"{args.prefix.name}_monitor_columns.txt", "w")
                t0 = time.time()

                def between(n, phase="roll"):
                    drain()
                    (r, g, b, dark), columns = monitor_reading(scanner)
                    line = (f"{phase} {(time.time() - t0) / 60:7.2f} min  after frame {n:2d}  open R {r:6.0f} G {g:6.0f} "
                            f"B {b:6.0f}  dark R {dark:6.0f}")
                    print(line)
                    monitor.write(line + "\n")
                    monitor.flush()
                    for name, values in columns.items():  # one line per reading: phase, frame, what, the column means
                        if values is not None:
                            profiles.write(f"{phase} {n} {name} " + " ".join(f"{v:.0f}" for v in values) + "\n")
                    profiles.flush()

            frames = scan_frames(scanner, opts, on_frame, on_align, progress, before_scan=drain, between=between)
            drain()
            if args.monitor and args.monitor_rest > 0:
                scanner.lamp(False)
                print(f"lamp off for {args.monitor_rest} min")
                if not args.dry_run:
                    time.sleep(args.monitor_rest * 60)
                t1 = time.time()
                for at in MONITOR_REST:
                    if not args.dry_run:
                        time.sleep(max(0.0, t1 + at * 60 - time.time()))
                    between(frames, "rest")
            saver.shutdown()
    except (ScannerError, KeyboardInterrupt) as e:
        print("stopped:", e or "interrupted", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
