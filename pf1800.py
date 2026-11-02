# SPDX-License-Identifier: GPL-2.0-or-later
"""Driver for the Pacific Image PrimeFilm 1800 AFL film scanner (USB 2770:0119), written from a reverse-engineered protocol.

Everything here was verified on one unit (docs/protocol.md describes the protocol):
the vendor handshake, the chip registers, the carriage moves, the MCU command interface (lens, firmware version, film
feed), the film sensors, 8- and 16-bit motor-driven scans of mounted slides and film strips, fast 16-bit greyscale scans
(the front end in mono mode, about twice the 16-bit colour rate) with the gain set from open light, autofocus,
flat-field correction, and the strip transport (closed-loop feed measured from the sprocket holes, frame alignment, eject).

Needs pyusb (pip install pyusb) and libusb. With dry_run=True nothing is sent: the transfers are printed and every wait is
skipped, which is how a new sequence should be reviewed before it is run on the hardware.

Geometry (all in motor steps, 1/1800 inch if the nominal resolution holds; mode 0 moves away from home, mode 1 toward it):
the home sensor zone is set at the park position; `reference()` releases it going away (about 85 steps) and every position is
counted from that exit edge. The lit calibration window is at about 100 to 250 steps, the carriage body is dark from about 275
to 1550, the open film gate runs from about 1555 to 3308. A mounted slide's opening spans about 1595 to 3252 steps and CCD
pixels 178 to 2720, so the default scan reads the whole CCD line for 1800 lines from 1550.
"""

from __future__ import annotations

import struct
import time
import zlib
from array import array

import usb.core
import usb.util

VID, PID = 0x2770, 0x0119
EP_BULK_OUT, EP_BULK_IN = 0x02, 0x81
CTRL_TIMEOUT_MS, BULK_TIMEOUT_MS = 1500, 3000
CHUNK = 8192  # a multiple of 64 and larger than a line: libusb reports an overflow when the device sends more than the buffer

SPEED = 0xDC
RETURN_SPEED = 0xE4  # moves toward home: about 490 steps/s against 390, 10 of 10 out-and-back tests without lost steps (0xE8 14
# of 14, 0xEC failed one of two, 0xF0 stalled); the moves away from home stay at SPEED, where a stall would lose the position
MAX_MOVE = 3500  # steps in one move
MAX_POSITION = 3450  # steps past the reference edge (the far-end mapping ran to 3500 without trouble)
PIXELS = 0xAC0  # pixels per scan line: the whole CCD line (the vendor's 2586-pixel window cut a slide's far edge)
LINE_BYTES = PIXELS * 3
LEFT = 1  # first CCD pixel of the scan window
CROP = 2  # first CCD pixels dropped from images (pixel 0 carries a counter, not image data)
WIDTH = PIXELS - CROP  # image width after the crop
# AFE gains and offsets (R, G, B) per bits per sample. 16-bit: gain code 0 on all channels keeps open light (the lit
# calibration window, about 60 % of full scale) clear of clipping with a warm lamp; red at code 1 clipped there once the lamp
# had warmed up (probe33_gain16.py, probe35_flat.py). Flat-field correction rescales open light to FLAT_TARGET anyway.
DEFAULT_LEVELS = {8: ((0, 15, 15), (200, 208, 208)), 16: ((0, 0, 0), (200, 208, 200))}
CCD_PIXELS = 0xAC0
TIMING = [0x14, 0, 0, 0x40, 0, 0, 0x30, 0, 0, 0, 0, 0, 1, 2]  # the vendor's variant A (the 2688-pixel CCD of this unit)
LAMP_LEVEL, LAMP_SECOND = 0xFA, 0xAA
MCU_BUSY_S = 20  # longest wait for a busy MCU (a long lens move)
MCU_WAIT_S, MCU_FIRST_WAIT_S = 5.0, 30.0  # longest wait for one MCU handshake step; the retry of a session's first one
MCU_REOPEN_S = 5.0  # pause between closing a session whose first MCU handshake went unanswered and reopening it
# (roll22 to roll24, gf1, blk1, roll39: the first handshake after a power cycle, a reconnect or a fresh strip load went
# unanswered even for 30 s, and a rerun a few seconds later answered at once; firmware_version() retries it so)
FLAT_TARGET = 60000 / 65535  # flat-field: open light maps to this share of full scale (the vendor uses 64000; the lower
# target leaves headroom for the lamp drifting brighter between the reference and the scan, which clipped empty-gate scans)
MIN_SPAN = 200  # flat-field: smallest light - dark span (16-bit units) divided by; weaker pixels are not amplified
WEAK_SPAN = 0.5  # flat-field: pixels whose light - dark span is below this share of the median are set to 0
CAL_OPEN_AT, CAL_DARK_AT, CAL_LINES = 30, 700, 64  # flat-field references: inside the lit calibration window (open
# light from 0 to about 130 steps past the reference edge, falling to dark by 180) and in the dark carriage body
BACKLASH_OVERSHOOT = 100  # steps a move toward home overshoots before coming back out (the measured backlash is ~45)
# Fast greyscale (probe48, probe49): the front end in mono mode converts one colour per pixel and register 0x90 bit 7 makes
# the controller send one 16-bit sample per pixel. In this mode the controller does not hold back when the host falls
# behind (in colour it does), so the line rate must stay under what the USB 1.1 bus carries: a line takes 1 + 0xBB base
# periods; 0xBB 3 (148 lines/s) always overruns, 0xBB 4 (118 lines/s) overran once in seven frames (roll18), 0xBB 5
# (98 lines/s, 530 KB/s, about 18 s for 1800 lines) leaves a third of the bus spare. 16-bit colour streams 48 lines/s.
GREY_BB = 5  # 0xBB for 16-bit grey; 0xC8 is GREY_BB + 1, as the vendor pairs them
# The byte in 0x88 and 0xC6 sets the line time, which is proportional to 256 minus the byte, and so the light each line
# gathers (noise1). A strip's frames run at LONG_LINE: at 0x90 with the gain cut to match, signal to noise on dense film is
# about twice that at LINE_PERIOD, the dark noise not rising with the time, for 39 s a frame against 18. Open light tops
# out near 30000 at gain 0, past about 0x90, and 0xE8 is too fast (the stream breaks).
LINE_PERIOD, LONG_LINE = 0xCB, 0x90
SLIP_CLEAN, SLIP_NOISE, SLIP_LOOK = 0.8, 0.5, 8  # byte-slip repair (repair_slips): neighbouring lines of a picture or an
# empty gate correlate above 0.9, lines after a slip around 0; SLIP_LOOK lines decide
# Proof scans at 300 dpi (8-bit grey only): the vendor's table for 300 dpi is
# 6 motor steps per output line (0x85), line period 0xE7 (0x88 and 0xC6), 0xBB 1 (0xC8 = 2), CCD timing t[7..9] = 0, 0, 2 for a
# strip (3, 3, 3 for a slide), a window mask on every 6th pixel and the step count lines * 6 + 2. Measured on the unit: lines
# must be read one per bulk transfer (the vendor does; 8192-byte chunks scramble short lines) and a multiple of 64 bytes (448
# samples, the CCD's 2688 pixels); lines are then 448 samples plus a varying 0 to 30 more, each ended by a marker of 11 zero
# samples, which sync_proof_lines() splits on. 16-bit mono at 300 dpi gave noise.
PROOF_STEP, PROOF_SPEED, PROOF_BB = 6, 0xE7, 1
PROOF_TARGET = 56000  # a frame's brightest part (film base, the 99.5th percentile) is brought to this at the final scan
PROOF_GAIN = 2  # front-end gain of the proof: at 5 the brightest part of a frame came within 7 % of clipping (proof5)
PROOF_SCALE = 0.42  # 1800 dpi level / 300 dpi proof level of film at the same gain (both linear in the gain code, as
# GREY_GAIN_STEPS has it; 8-bit and 16-bit 1800 dpi agree): 0.42 on film base (pc1), 0.40 on the strip's tail (ps1),
# 0.36 to 0.41 over the picture of five frames. Open light with no film reads higher (0.64 to 0.69), so an empty gate may
# clip; it is never part of a frame
ALIGN_SCALE = 1.05  # 1800 dpi grey frame level (16-bit) / its alignment scan's level (ALIGN_CHANNEL at ALIGN_GAIN, 8-bit
# times 257) per unit of gain factor, both the 99.5th percentile. With green frames it was 1.07 to 1.16 (gf1, proof7, rw2:
# 7 frames at gains 4 and 15); green reads 1.11 times red at the same gain (probe51), so red frames, now read like the
# alignment scan, come to 0.96 to 1.05. That is derived, not yet measured on red frames; the worst case, so none clips
OPEN_LIGHT = 130  # an alignment scan whose median column reads this or more shows open light in most of the gate: the strip's
# end has passed (the end of all ten saved rolls and strips: 151 to 194; film base reads 77 to 97). One that finds no frame
# edge with film still in the gate (a blank or pale frame) does not end the roll.
# Black level of 16-bit grey strip frames (probe54, roll36e). At the fixed red offset of DEFAULT_LEVELS (200) and gain 15
# the black level sits around 0 and drifts down along a roll (roll36e: 3088 at frame 1, 0 from frame 8; roll36b to roll36d
# 0 throughout), so the densest negative, the picture's highlights, read 0. A lower offset code raises it, by about 165
# counts per code at gain 15, linearly (probe54: 2576 at 180 to 25648 at 40). black_offset() sets the red offset at the
# start of a strip so the black level sits at BLACK_TARGET, each frame's own black level (BLACK_COLUMNS, past the gate end,
# where no light reaches the CCD) then moves the offset for the next scan, and it is subtracted from the frame
# (Frame.black), so the margin costs nothing but a little range
BLACK_COLUMNS = slice(2700, 2745)  # image columns past the gate end (GATE_END 2686): the holder, no light
BLACK_TARGET = 6000  # roll36e drifted 2500 counts in its first 4 minutes, a frame takes about 1
BLACK_PROBE = (160, 120)  # offset codes of black_offset()'s two short scans
BLACK_LINES = 64
# The response sags along a roll (roll37: open light in the calibration window, no film, fell 14 % over 39 frames, under
# 1 % at idle; each frame's clear film tracked it within about 1 % at gain 0 and at gain 15, so the loss comes before the
# front-end gain). A strip reads open light (open_level()) before frame 1 and before each frame, and each frame is scaled
# by the first reading over its own (Frame.scale), so the whole roll keeps the first frame's level scale.
OPEN_PERCENTILE = 99.5  # open_level(): of CAL_LINES lines at CAL_OPEN_AT, 16-bit mono at gain 0
# The light falls off along the CCD, to about 0.95 of the middle at the frame's left end and 0.88 at its right (roll40,
# roll41; the same in open-gate alignment scans within 1 %, and steady along a roll). falloff_gain() flattens that from a
# strip's first open-light reading (Frame.flat), smoothed over FALLOFF_SMOOTH columns so it follows the light and not
# single pixels, and at most FALLOFF_MOST.
FALLOFF_SMOOTH, FALLOFF_MOST = 41, 1.3
# The lamp (a cold-cathode tube) rises about 60 % over its first 4 to 5 minutes (probe55), so a fixed 10 s warm-up left
# roll38's frame 1 on a lamp still rising fast: its open-light reading, the roll's sag reference, was 7 % off by the time
# the frame was scanned. warm_lamp() reads open light every LAMP_EVERY seconds until it rises less than LAMP_SETTLE in one
# interval, or LAMP_MOST seconds have passed.
LAMP_EVERY, LAMP_SETTLE, LAMP_MOST = 20.0, 0.003, 360.0
GREY_CHANNEL = 0  # the colour the mono front end samples (0 red, 1 green, 2 blue). Red, as the alignment scan and the
# autofocus metric read: red and green focus several lens units apart, so a green frame was scanned off focus (probe51).
# Green would read 1.11 times as much at the same gain; the vendor's default mono channel is green
GREY_GAIN_STEPS = ((0, 1.0), (2, 2.14), (4, 2.9), (8, 4.5), (15, 6.3))  # front-end gain code -> relative level (probe49
# --gain; code 2 from seven strip frames scanned at 15 seconds after their proof at 2: 2.94 times the level, not the 3.23
# a straight line from 0 to 4 gave, which had every frame land 6 to 10 % under its prediction)
GREY_TARGET = 46000  # grey scans: open light in the lit calibration window is kept under this; the open gate reads about
# 1.25 times brighter (roll17), so an empty gate or clear film stays under full scale with a little margin for the lamp

# Film strips (probe40 to probe45). The film runs along the CCD line; feeding forward (into the scanner, toward the exit
# slot) moves the picture toward CCD pixel 0. MCU commands: 2 / 6 a coarse unit forward / back (about 312 to 340 px, it
# varies), always followed by 7 (without it the fine units stall deep in); 1 / 5 a fine unit forward / back (about 15 px).
# Reversing the feed loses a varying slack (90 to 190 px), so positioning only ever feeds forward, closed loop.
STRIP_PARK = 1420  # carriage steps over the middle of the sprocket-hole row: feeds and hole reads happen here
HOLE_SCAN = (400, 1360)  # lines and approach of a motor scan of the hole window and the top of the picture (probes)
WHOLE_HOLE = (125, 160)  # widths of an uncut sprocket hole in a stationary line (142 px)
HOLE_ROWS, PROFILE_ROWS = (10, 100), (250, 390)  # rows of that scan: the hole window, the picture
HOLE_WINDOW = (960, 1670)  # CCD pixels of the hole window
PERF_PITCH = 341  # pixels between sprocket holes (4.75 mm)
PX_PER_MM = PERF_PITCH / 4.75
FINE_PX, COARSE_LEFT, FEED_TOLERANCE = 15.0, 345, 8  # fine units close the last COARSE_LEFT pixels to within the tolerance
REWIND_MAX = 64  # coarse units rewind() may feed back: a 6-frame strip is about 50 (8 per frame); its end took 23 for 3
REWIND_FIND = 16  # units back within which the strip's tail must reach page B bit 6 when it starts past it (live: 0)
REWIND_PAST = 3  # more units back once bit 6 clears: it clears about 760 px in from where the auto-load holds the strip
# (rw1, proof6: the first frame's left edge then lay 760 px past the gate start and the alignment took a false gap)
ALIGN_SCAN = (600, 2000)  # lines and approach of the scan that finds the inter-frame gap (all picture rows)
ALIGN_SHORT = 150  # a check scan after a feed (_check_edge()) reads only the first ALIGN_SHORT of those lines. The alignment
# after an advance used to as well, but pale, even picture beside a gap fooled its top rows: over 110 roll36b-d frame scans,
# 20 short-row edges against 5 on 600 rows were more than 30 px from the edge over the whole frame height. Before that
# (offline on roll20 and roll21: edges within 7 px of the full scan's, about 4 s saved per frame); the first alignment,
# with several gaps in view, needs all rows (roll21: 101 px on 600 lines, 12 on 200, 1568 on 300). A short scan that finds
# no edge is redone in full. (roll22: twice it found an edge far into the gate, 1222 and 708, and the full rescan gave
# 1216 and 715, so a far edge is not a reason to rescan.)
# The alignment scan in 8-bit grey (148 lines/s, half the time of 8-bit colour) on the red channel at a gain that brings the
# mono level to about the colour scan's red at gain 0 (mono reads about a third of colour at the same gain), so the gap
# finder's absolute thresholds (GAP_FLAT, the mask test) still apply: at gain 5 probe50 read the same film base (82
# against 81), the same gaps and the same frame edge as the colour scan; gain 3 reads too low, gain 7 loses a gap. The
# frame it skipped in roll18 was skipped by the colour scan too (roll19): the frame was already in the gate, which
# frame_edge now handles (CUT_OK).
ALIGN_GREY, ALIGN_CHANNEL, ALIGN_GAIN = True, 0, 5
FRAME_SCAN = (1800, 1500)  # lines and approach of a strip frame scan (the frame lies from about 1550 to 3260 steps)
EDGE_AT = 45  # CCD pixel where a frame's left edge is placed (the gate shows about 0 to 2686, a frame is about 2600).
# Feeds overshoot and the edge reads early: with 20, frames came out 25 px cut to 13 px clear on the left (roll21 to roll23)
MIN_GAP, WIDE_GAP, GAP_FLAT, GAP_BASE, GAP_WIDTH = 60, 250, 8.0, 0.75, 134  # gap finder (see frame_gaps); the base
# share allows for the lamp falling off toward the ends of the CCD line (a gap at the gate edge reads about 85 % of the base)
# A scratched film: long scratches run along the film through its gaps, so a gap's columns are not flat over the rows and
# frame_gaps() misses them, but they barely move a median over part of a column. clear_gaps() splits the rows into
# CLEAR_BANDS bands and takes the runs of columns whose darkest band's median is at least CLEAR_SHARE of the film base,
# closing breaks up to CLEAR_BRIDGE wide: a gap is clear over every row, where the whole column's median passed bright skin
# on thin negatives (roll40 frame 37, roll48 align25: darkest band 0.77 and under). Real gaps on roll50 and roll51 read 0.86
# to 0.90 there, the pale wall beside one 0.79 (roll51 align02, where counting pixels off the median passed the wall and
# broke the gap). Only where the flatness test finds nothing (used everywhere it brought back roll22's false edge at 1698),
# and an edge only from a gap cut by the gate start or a whole gap at least SCRATCH_GAP wide with film on both sides: a gap
# cut by the gate's end, and thinner runs, were false there (roll48 align25, roll50 align02 at 370 to 435)
CLEAR_SHARE, CLEAR_BANDS, CLEAR_BRIDGE, SCRATCH_GAP = 0.83, 8, 20, 90
GATE_TOUCH = 25  # a gap ending this close to GATE_END runs into the gate's mask even where mask() cannot see it; with no
# better inner edge it places the frame before it (roll22 frame 1: a gap ending at 2681 of 2686 was taken for an inner one,
# a false edge at 1695 was found in bright picture and the whole frame before the gap lost)
EDGE_GAP, FRAME_PX = 8, 2605  # a gap cut by the gate's ends needs only EDGE_GAP columns; a frame is about 2600 to 2610 px
REF_GAP, FALSE_SPREAD, FALSE_DIP = (16, 400), 1.2, 0.04  # plausible_gaps(): drops a bright patch of picture taken for a
# gap; the reference gaps are REF_GAP columns wide (real ones are up to about 250; wider is open light or a strip's tail)
CUT_FALSE = (1.6, 0.1)  # a gap cut by the gate's ends reads rougher (the lamp falls off), so it must be this much worse
CUT_OK = 150  # a frame found already in the gate is scanned where it is if no more than this (about 2 mm) lies before the
# gate; the strip only feeds forward, so the alternative would be to lose the frame (roll18 and roll19 did)
GATE_END, COARSE_PX = 2686, 320  # last CCD pixel the gate shows; a typical coarse unit (312 to 340 px)
FINE_MAX, FINE_ROUNDS = 7, 6  # fine units per round and rounds: a hole read is exact only modulo PERF_PITCH and is folded
# toward the expected travel, so a round must expect under half a pitch (170 px; a unit moves 15 to 22 px), or a stall reads
# as a whole pitch (roll36b: rounds of 12 to 20 units that did not move read 341 to 384 px and left 23 of 37 frames short)
ALIGN_FEEDS = 3  # feeds that bring a frame edge to EDGE_AT, each checked by a short alignment scan (fine units stall at
# random along a long roll, roll36b)
OVER_OK, BACK_TO, BACK_MAX, BACK_TIMES = 20, 400, 4, 2  # a frame edge more than OVER_OK before the gate start (fed too far) is
# backed off, up to BACK_TIMES times:
# coarse units back until the edge is BACK_TO into the gate (at most BACK_MAX), one coarse unit forward to take up the
# reversal slack and re-engage the fine drive, then fine feeds (probe53: from -150 back 264 + 311, forward 265, fine 120,
# ending at 38 to 42 every time; fine units alone hardly move after a reversal, probe52)
CHECK_AGREE = 60  # a check scan more than this from the hole reads' measure is not believed: the holes measure the film to a
# few pixels, a check scan can be fooled both ways (roll36e frame 22: a pale wall read 45 where the holes said -213, so no
# back-off and the frame lost 650 px; the roll simulation: 30 read as -24, a needless back-off)
PREDICT_AT = 35  # where a predicted advance aims the edge (a little before EDGE_AT: the pitch scatters by about 20 px, and an
# edge landing early costs nothing until it passes the gate start, a late one costs a second feed and check scan)
PREDICT_LEFT, PREDICT_NEXT = 150, 100  # advance without an alignment scan (predict_edge()): needs the frame's own left edge
# within PREDICT_LEFT of the gate start, and, where the next frame's gap shows in the scan too, that to agree within PREDICT_NEXT
BACK_PX, AFTER_BACK_PX = 260, 240  # expected travel of a coarse unit back, and of the first one forward after it (the
# hole reads fold toward these: probe52/53 measured 187 to 311 back, 265 forward; probe42 183 forward)
FINE_STALL = 0.3  # a fine round that moves less than this share of its nominal travel has stalled (the strip's tail has
# left the entrance roller, which is all the fine units drive)
RESCAN_LOSS, FRAME_ROWS = 40, (150, 1700)  # also: a frame scan whose gap after the frame starts more than RESCAN_LOSS short of
# FRAME_PX shows the frame fed too far, its left end lost (early_edge(); on every saved frame scan it flags exactly the 20
# known overshoots, roll36c-e among them)
_RESCAN_NOTE = None  # a strip frame scan whose first real gap (over the picture rows FRAME_ROWS,
# all of which a short alignment scan does not see) puts the frame's left edge more than RESCAN_LOSS past where the whole
# frame still fits the gate is fed on and scanned again (late_edge(); roll36b frames 2 and 25, roll36c frame 26: pale
# picture beside a gap fooled the alignment)
LEADER_DARK = 8  # an uncut roll starts with its leader, fogged black when the camera was loaded: its alignment scan read
# column means of at most 2.7 (99.5th percentile 4) against 77 to 97 for film base (roll36). A gate whose columns are all
# under LEADER_DARK holds only leader, which the user is to cut off before loading


class ScannerError(Exception):
    pass


class Transport:
    """The USB transport: vendor control transfers plus bulk endpoints."""

    def __init__(self, dev, send, trace):
        self.dev, self.send, self.trace = dev, send, trace
        self.stream_started = False

    def ctrl_out(self, wvalue, windex, data):
        data = bytes(data)
        request = 0x04 if len(data) > 1 else 0x0C
        if self.trace:
            print(f"  OUT bRequest 0x{request:02X} wValue 0x{wvalue:04X} wIndex 0x{windex:04X} data {data.hex(' ')}")
        if self.send:
            self.dev.ctrl_transfer(0x40, request, wvalue, windex, data, CTRL_TIMEOUT_MS)
            time.sleep(0.005)

    def write_regs(self, *pairs):
        self.ctrl_out(0x00B0, 0, bytes(b for pair in pairs for b in pair))

    def write_reg(self, reg, val):
        self.write_regs((reg, val & 0xFF))

    def read_reg(self, reg, twice=False):
        self.ctrl_out(0x00B0, 0, bytes([reg]))
        if not self.send:
            return None
        value = None
        for _ in range(2 if twice else 1):
            value = self.dev.ctrl_transfer(0xC0, 0x0C, 0x0007, 0x0000, 1, CTRL_TIMEOUT_MS)[0]
        return value

    def clean_buffer(self):
        self.ctrl_out(0x00C0, 0x8406, b"\x00")
        self.ctrl_out(0x00C0, 0x0406, b"\x00")

    def write_line(self, data):
        self.ctrl_out(0x00C0, 0x0406, b"\x00")
        self.ctrl_out(0x00B0, 0, b"\x8f")
        self.ctrl_out(0x00C0, 0x8406, b"\x8f")
        self.ctrl_out(0x00C0, 0x0406, b"\x8f")
        self.ctrl_out(0x0002, len(data), b"\x02")
        if self.send:
            self.dev.write(EP_BULK_OUT, data, timeout=BULK_TIMEOUT_MS)

    def read_small(self, n):
        self.ctrl_out(0x0003, n, b"\x00")
        if not self.send:
            return None
        return bytes(self.dev.read(EP_BULK_IN, max(n, 64), timeout=BULK_TIMEOUT_MS))[:n]

    def stream_chunk(self, size=CHUNK):
        if not self.stream_started:
            self.clean_buffer()
            self.ctrl_out(0x0001, 0x0000, b"\x01")
            self.stream_started = True
        if not self.send:
            return None
        return bytes(self.dev.read(EP_BULK_IN, size, timeout=BULK_TIMEOUT_MS))


class Frame:
    """The raw lines of one scan (PIXELS x RGB samples each, or PIXELS grey samples when `channels` is 1, as read; 8 bits,
    or 16 bits little-endian when depth is 16) with the vendor's post-processing."""

    def __init__(self, lines, depth=8, levels=None, channels=3, step=1):
        self.lines, self.depth, self.levels = lines, depth, levels  # levels: the (gains, offsets) it was scanned with
        self.channels = channels
        self.step = step  # CCD pixels per sample: 1 at 1800 dpi, PROOF_STEP at 300 dpi
        self.dpi = 1800 // step
        self.black = 0  # 16-bit grey: subtracted from every sample by aligned() (the measured black level, see BLACK_TARGET)
        self.scale = 1.0  # 16-bit grey: aligned() then multiplies by this (the lamp sag correction, see OPEN_PERCENTILE)
        self.flat = None  # 16-bit grey: and by this, one value per column (the falloff correction, see falloff_gain())

    def _samples(self, line):
        return array("H", line) if self.depth == 16 else line

    def aligned(self):
        """Colour rows 4 and 8 lines apart realigned (G from line n+4, B from n+8), each line mirrored left to right and the
        first CROP CCD pixels dropped (WIDTH pixels remain). Rows are bytes (8-bit) or arrays of 16-bit samples. Grey lines
        come from one CCD row, so they are only mirrored and cropped."""
        if self.channels == 1:
            if self.step > 1:  # a proof scan starts at LEFT: no counter pixel to drop
                return [self._samples(line)[::-1] for line in self.lines]
            rows = [self._samples(line)[::-1][:WIDTH] for line in self.lines]
            if self.depth == 16 and (self.black or self.scale != 1.0 or self.flat is not None):
                import numpy as np

                black = np.uint16(min(int(self.black), 65535))
                out = []
                for r in rows:
                    v = np.maximum(np.frombuffer(r.tobytes(), "<u2"), black) - black
                    if self.scale != 1.0 or self.flat is not None:
                        gain = self.scale if self.flat is None else self.scale * self.flat
                        v = np.minimum(np.rint(v * gain), 65535).astype(np.uint16)
                    out.append(array("H", v.tobytes()))
                rows = out
            return rows
        out = []
        for n in range(len(self.lines) - 8):
            r, g, b = (self._samples(self.lines[n + k]) for k in (0, 4, 8))
            row = array("H", bytes(2 * len(r))) if self.depth == 16 else bytearray(len(r))
            row[0::3], row[1::3], row[2::3] = r[0::3][::-1], g[1::3][::-1], b[2::3][::-1]
            row = row[: 3 * WIDTH]  # the mirrored line ends with the first CCD pixels
            out.append(row if self.depth == 16 else bytes(row))
        return out

    def grey(self, rows):
        """Single-channel rows (bytes or array('H'), WIDTH samples) from aligned rows: the mean of R, G and B, for black
        and white film, whose channels carry the same picture (balanced at the 16-bit default levels). Grey rows are
        returned as they are."""
        import numpy as np

        if self.channels == 1:
            return list(rows)

        kind = np.uint16 if self.depth == 16 else np.uint8
        out = []
        for row in rows:
            rgb = np.frombuffer(row.tobytes() if self.depth == 16 else row, kind).reshape(-1, 3)
            mean = rgb.astype(np.uint32).sum(1) // 3
            out.append(array("H", mean.astype(np.uint16).tobytes()) if self.depth == 16 else mean.astype(np.uint8).tobytes())
        return out

    def means(self):
        """Per line (R, G, B) means, or (grey,), for quick looks (in the scan's own units)."""
        c = self.channels
        return [tuple(sum(s[CROP * c + k :: c]) / (PIXELS - CROP) for k in range(c)) for s in map(self._samples, self.lines)]

    def to_8bit(self, rows):
        """16-bit rows to 8-bit bytes (top byte), for previews."""
        import numpy as np

        if self.depth != 16:
            return rows
        return [(np.frombuffer(row.tobytes(), "<u2") >> 8).astype(np.uint8).tobytes() for row in rows]


class Calibration:
    """Per-pixel, per-channel dark and open (lamp light through the empty calibration window) levels in CCD order, for
    host-side flat-field correction. Taken with the same depth, gains, offsets and channels as the scan it corrects."""

    def __init__(self, dark, light, depth, gains, offsets):
        self.dark, self.light, self.depth, self.gains, self.offsets = dark, light, depth, tuple(gains), tuple(offsets)

    @classmethod
    def from_frames(cls, dark, light):
        import numpy as np

        def mean(frame):
            dtype = "<u2" if frame.depth == 16 else "u1"
            lines = np.stack([np.frombuffer(line, dtype=dtype) for line in frame.lines])
            return lines.reshape(len(frame.lines), -1, frame.channels).mean(0)

        return cls(mean(dark), mean(light), dark.depth, dark.levels[0], dark.levels[1])

    def save(self, path):
        import numpy as np

        np.savez(path, dark=self.dark, light=self.light, depth=self.depth, gains=self.gains, offsets=self.offsets)

    @classmethod
    def load(cls, path):
        import numpy as np

        z = np.load(path)
        return cls(z["dark"], z["light"], int(z["depth"]), tuple(map(int, z["gains"])), tuple(map(int, z["offsets"])))

    def apply(self, frame):
        """(raw - dark) / (light - dark), scaled so open light lands at FLAT_TARGET of full scale; a new Frame."""
        import numpy as np

        channels = self.light.shape[1]
        if (frame.depth != self.depth or tuple(frame.levels[0]) != self.gains or tuple(frame.levels[1]) != self.offsets
                or frame.channels != channels):
            raise ScannerError("the calibration was taken with a different depth, gains, offsets or channels than the scan")
        dtype, full = ("<u2", 65535) if frame.depth == 16 else ("u1", 255)
        span = self.light - self.dark
        weak = span < WEAK_SPAN * np.median(span, axis=0)  # pixels the open light hardly reaches (window edge, CCD ends)
        gain = np.where(weak, 0.0, FLAT_TARGET * full / np.maximum(span, MIN_SPAN * full / 65535))
        out = []
        for line in frame.lines:
            raw = np.frombuffer(line, dtype=dtype).reshape(-1, channels).astype(np.float32)
            fixed = np.clip(np.rint((raw - self.dark) * gain), 0, full).astype(dtype)
            out.append(fixed.tobytes())
        return Frame(out, frame.depth, frame.levels, frame.channels)


def _line_corr(a, b):
    """Correlation of each row of `a` with the same row of `b` (float arrays, rows x samples)."""
    import numpy as np

    a0, b0 = a - a.mean(1, keepdims=True), b - b.mean(1, keepdims=True)
    den = np.sqrt((a0 * a0).sum(1) * (b0 * b0).sum(1))
    return np.where(den > 0, (a0 * b0).sum(1) / np.where(den > 0, den, 1), 0.0)


def repair_slips(stream, lines, line_bytes=PIXELS * 2):
    """Split a 16-bit grey stream into `lines` lines, repairing byte slips: now and then the controller inserts or drops
    a few bytes (an odd number splits every later sample across two, so the rest of the frame reads as noise; roll18,
    roll19, probe49). A slip shows as a clean line followed by lines that no longer correlate with their neighbours; the
    rest of the stream is then re-split at the offset (-3..+3 bytes) under which the following lines correlate again,
    choosing among those the one that best continues the last clean line. The lines between the last clean one and the
    first clean one after the re-split (the one holding the slip, sometimes the next) are interpolated between those two;
    a stream that comes up short is padded with its last line. Returns (lines, slips) with slips as (line, offset)
    pairs."""
    import numpy as np

    data = bytes(stream)
    samples = line_bytes // 2

    def rows(at, count):
        count = min(count, (len(data) - at) // line_bytes)
        if at < 0 or count <= 0:
            return np.empty((0, samples - CROP))
        return np.frombuffer(data[at : at + count * line_bytes], "<u2").reshape(count, samples)[:, CROP:].astype(float)

    out, slips, gaps = [], [], []
    base, look, split = 0, 0, None  # byte offset of the next line to take, where to look for a slip from (in lines past
    # base), and the lines split from base (kept while base stays put)
    while len(out) < lines:
        if split is None:
            arr = rows(base, lines - len(out))
            if not len(arr):
                break
            c = _line_corr(arr[:-1], arr[1:])  # c[j]: line j against line j + 1
            split = base
        cut = None
        for j in range(max(1, look), len(c)):
            if c[j - 1] >= SLIP_CLEAN and np.median(c[j : j + SLIP_LOOK]) < SLIP_NOISE:
                cut = j + 1  # line j is the last clean one (a slip late in a line still half correlates)
                break
        if cut is None:
            out.extend(bytes(data[base + i * line_bytes : base + (i + 1) * line_bytes]) for i in range(len(arr)))
            base += len(arr) * line_bytes
            break
        at = base + cut * line_bytes
        best = None
        for d in (-3, -2, -1, 1, 2, 3):
            cand = rows(at + d, SLIP_LOOK + 1)
            if len(cand) < 4:
                continue
            cc = _line_corr(cand[:-1], cand[1:])
            if np.median(cc) < SLIP_CLEAN:
                continue
            k = int(np.argmax(cc >= SLIP_CLEAN))  # cand[k] is the first clean line under this offset
            k = max(k, 1)  # cand[0] holds the slip
            back = float(_line_corr(arr[cut - 1 : cut], cand[k : k + 1])[0])
            if best is None or back > best[0]:
                best = (back, d, k)
        if best is None:  # no offset makes the lines clean again (the dark mask, say): keep looking further on
            look = cut
            continue
        _, d, k = best
        out.extend(bytes(data[base + i * line_bytes : base + (i + 1) * line_bytes]) for i in range(cut))
        gaps.append((len(out), k))
        slips.append((len(out), d))
        out.extend([None] * k)  # the damaged lines, interpolated below
        base, look, split = at + d + k * line_bytes, 0, None
    while len(out) < lines:
        out.append(out[-1] if out else bytes(line_bytes))
    out = out[:lines]
    for first, k in gaps:
        if first >= len(out):
            continue
        last = min(first + k, len(out)) - 1
        lo = np.frombuffer(out[first - 1], "<u2").astype(float) if first > 0 else None
        hi = np.frombuffer(out[last + 1], "<u2").astype(float) if last + 1 < len(out) and out[last + 1] else None
        lo, hi = (hi if lo is None else lo), (lo if hi is None else hi)
        for i in range(first, last + 1):
            t = (i - first + 1) / (k + 1)
            line = np.zeros(samples) if lo is None else (1 - t) * lo + t * hi
            out[i] = np.rint(line).astype("<u2").tobytes()
    return out, slips


def scan_pixels(step=1):
    """Samples per scan line: the window mask marks every `step`-th pixel from LEFT. A proof scan keeps to the CCD's 2688
    pixels, so its line is a multiple of the controller's 32-byte buffer unit (448 samples)."""
    return PIXELS if step == 1 else 2688 // step


def sync_proof_lines(stream, lines, width=432, mark=(9, 13), shortest=436):
    """Split the 8-bit stream of a proof scan into `lines` lines of `width` samples. The controller ends every 300 dpi
    line with a marker of 11 zero samples, and lines are 448 samples plus a varying 0 to 30 more, the extra at the line's
    start: the picture keeps its place against the marker that ends the line (a straight film edge sits at the same
    distance before every marker, ps1), so each line is the `width` samples before a marker. Zero runs of `mark`
    samples are the markers; one closer than `shortest` to the last is dark picture, not a marker. A line whose marker
    was not found repeats the last line, and so does the end of a stream that ends short."""
    import numpy as np

    a = np.frombuffer(bytes(stream), "u1")
    edge = np.flatnonzero(np.diff(np.concatenate(([0], (a <= 2).astype(np.int8), [0]))))
    out, last = [], None
    for s, e in zip(edge[0::2], edge[1::2]):
        if not mark[0] <= e - s <= mark[1] or s < width or (last is not None and s - last < shortest):
            continue
        if last is not None:  # a missed marker: hold the last line for each line it hid
            out.extend([out[-1]] * (int(round((s - last) / (shortest + 16))) - 1))
        out.append(bytes(a[s - width : s]))
        last = int(s)
    while len(out) < lines:
        out.append(out[-1] if out else bytes(width))
    return out[:lines]


def frame_columns(frame):
    """The mean of each column of a 16-bit grey scan as read (before Frame.black or Frame.scale), in the columns of
    aligned(): mirrored, first CROP CCD pixels dropped, WIDTH values."""
    import numpy as np

    return np.mean([np.frombuffer(line, "<u2")[::-1][:WIDTH] for line in frame.lines], axis=0)


def line_exposure(period):
    """The light a line gathers at line period byte `period`, against LINE_PERIOD."""
    return (256 - period) / (256 - LINE_PERIOD)


def falloff_gain(columns):
    """The gain per column that flattens the light along the CCD, from frame_columns() of an open-light scan: the lit
    columns' median over their smoothed level, at most FALLOFF_MOST; 1 outside the lit columns (the gate mask)."""
    import numpy as np

    lit = np.flatnonzero(columns >= 0.5 * np.median(columns))
    lo, hi = lit[0], lit[-1] + 1
    pad = FALLOFF_SMOOTH // 2
    level = np.convolve(np.pad(columns[lo:hi], pad, mode="edge"), np.ones(FALLOFF_SMOOTH) / FALLOFF_SMOOTH, "valid")
    gain = np.ones(len(columns))
    gain[lo:hi] = np.minimum(np.median(columns[lo:hi]) / level, FALLOFF_MOST)
    return gain


def grey_gain_factor(code):
    """The relative level of front-end gain `code` against code 0, interpolated between the measured GREY_GAIN_STEPS."""
    for (c0, f0), (c1, f1) in zip(GREY_GAIN_STEPS, GREY_GAIN_STEPS[1:]):
        if c0 <= code <= c1:
            return f0 + (f1 - f0) * (code - c0) / (c1 - c0)
    raise ValueError(f"gain code {code} outside 0..15")


def tiff_bytes(rows, width, depth, dpi=1800, channels=3):
    """An uncompressed TIFF (little-endian), RGB from rows of width * 3 samples or greyscale (channels=1, black is zero)
    from rows of width samples; rows are bytes for 8 bits, array('H') for 16."""
    data = b"".join(row.tobytes() if depth == 16 else bytes(row) for row in rows)
    height = len(rows)
    entries = 13
    ifd = 8
    extra = ifd + 2 + entries * 12 + 4
    bits_at, xres_at, yres_at = extra, extra + 6, extra + 14
    data_at = extra + 22
    tags = [
        (256, 4, 1, width), (257, 4, 1, height), (258, 3, 3, bits_at) if channels == 3 else (258, 3, 1, depth),
        (259, 3, 1, 1), (262, 3, 1, 2 if channels == 3 else 1),
        (273, 4, 1, data_at), (277, 3, 1, channels), (278, 4, 1, height), (279, 4, 1, len(data)), (282, 5, 1, xres_at),
        (283, 5, 1, yres_at), (284, 3, 1, 1), (296, 3, 1, 2),
    ]
    out = bytearray(b"II*\x00" + struct.pack("<I", ifd) + struct.pack("<H", entries))
    for tag, kind, count, value in tags:
        packed = struct.pack("<H", value) + b"\x00\x00" if kind == 3 and count == 1 else struct.pack("<I", value)
        out += struct.pack("<HHI", tag, kind, count) + packed
    out += struct.pack("<I", 0) + struct.pack("<HHH", depth, depth, depth)
    out += struct.pack("<II", dpi, 1) + struct.pack("<II", dpi, 1)
    return bytes(out) + data


def png_bytes(rows, width, channels=3):
    """A minimal 8-bit PNG, RGB from rows of width * 3 bytes or greyscale (channels=1) from rows of width bytes."""

    def chunk(kind, data):
        body = kind + data
        return struct.pack(">I", len(data)) + body + struct.pack(">I", zlib.crc32(body) & 0xFFFFFFFF)

    raw = b"".join(b"\x00" + row for row in rows)
    header = struct.pack(">IIBBBBB", width, len(rows), 8, 2 if channels == 3 else 0, 0, 0, 0)
    return b"\x89PNG\r\n\x1a\n" + chunk(b"IHDR", header) + chunk(b"IDAT", zlib.compress(raw, 6)) + chunk(b"IEND", b"")


def stretch(rows, channels=3):
    """Per-channel stretch of an image to 0..255 between its 0.5th-from-top and 1st percentiles (sampled); rows hold
    `channels` interleaved samples per pixel (bytes, or arrays of 16-bit samples)."""
    import numpy as np

    kind = np.uint16 if rows and isinstance(rows[0], array) else np.uint8
    img = np.array([np.frombuffer(row.tobytes() if kind is np.uint16 else row, kind) for row in rows], np.int64)
    out = np.empty(img.shape, np.uint8)
    for k in range(channels):
        plane = img[:, k::channels]
        values = np.sort(plane[:: max(1, len(rows) // 100)], axis=None)
        lo, hi = values[len(values) // 100], values[int(len(values) * 0.995)]
        scale = 255.0 / max(1, hi - lo)
        out[:, k::channels] = np.clip(((plane - lo) * scale).astype(np.int64), 0, 255)  # int() truncates toward 0 too
    return [row.tobytes() for row in out]


# ---- film strip measurements (numpy arrays of the red plane, rows x WIDTH) ---------------------------------------------


def red_plane(frame):
    """The red samples of a frame, aligned like its images, as a float array (rows x WIDTH)."""
    import numpy as np

    return np.array([np.frombuffer(r, np.uint8 if frame.depth == 8 else "<u2").reshape(WIDTH, 3)[:, 0]
                     for r in frame.aligned()], float)


def align_plane(frame):
    """The plane the gap finder reads: a grey frame's samples, or a colour frame's red, as floats (rows x WIDTH)."""
    import numpy as np

    if frame.channels == 1:
        kind = np.uint8 if frame.depth == 8 else "<u2"
        return np.array([np.frombuffer(r if frame.depth == 8 else r.tobytes(), kind) for r in frame.aligned()], float)
    return red_plane(frame)


def hole_centres(r):
    """Centres of the sprocket holes (bright runs at least 60 pixels wide) in the hole rows of a hole scan."""
    import numpy as np

    band = r[HOLE_ROWS[0] : HOLE_ROWS[1]].mean(0)
    thr = 0.5 * (band.max() + np.median(band[band > 20])) if (band > 20).any() else band.max()
    centres, start = [], None
    for x, on in enumerate([*(band > thr), False]):
        if on and start is None:
            start = x
        if not on and start is not None:
            if x - start >= 60:
                centres.append((start + x - 1) / 2)
            start = None
    return centres


def picture_profile(r):
    """Column edges of the picture rows of a hole scan, clear of the gate mask edges, which do not move."""
    import numpy as np

    return np.diff(r[PROFILE_ROWS[0] : PROFILE_ROWS[1]].mean(0)[100:2550])


def picture_shift(before, after, limit=1500, overlap=800):
    """The shift s (pixels) maximising the correlation of after[x] with before[x - s], and that correlation."""
    best = (None, -2.0)
    n = len(before)
    for s in range(-limit, limit + 1):
        lo, hi = max(0, s), min(n, n + s)
        if hi - lo < overlap:
            continue
        a, b = after[lo:hi], before[lo - s : hi - s]
        a, b = a - a.mean(), b - b.mean()
        den = ((a * a).sum() * (b * b).sum()) ** 0.5
        c = (a * b).sum() / den if den else -2.0
        if c > best[1]:
            best = (s, float(c))
    return best


def film_travel(before, after, expect=None):
    """Film travel in pixels (negative = forward) between two hole scans given as (hole centres, profile), and a note on
    how it was measured. With both holes clear of the window edges the hole shift is exact modulo the perforation pitch:
    it is folded next to `expect` when given (a fine round, whose travel stays well under half a pitch, so the picture is
    not needed), else next to the picture shift. Without clean holes it is the picture shift, which can lock onto the
    CCD's fixed column pattern (shift 0) when the picture rows are flat; `hole_based` is False then."""

    def clean(h):
        return (len(h) == 2 and abs(h[1] - h[0] - PERF_PITCH) <= 6
                and h[0] - 71 > HOLE_WINDOW[0] and h[1] + 71 < HOLE_WINDOW[1])

    def fold(value, near):
        return near + (value - near + PERF_PITCH / 2) % PERF_PITCH - PERF_PITCH / 2

    (h0, p0), (h1, p1) = before, after
    shift, corr = picture_shift(p0, p1)
    holes = clean(h0) and clean(h1)
    hs = min((a - b for a in h1 for b in h0), key=abs) if holes else None
    note = f"picture {shift} (corr {corr:.2f}), " + (f"holes {hs:.1f}" if holes else "holes not clean")
    if holes and expect is not None:
        return fold(hs, expect), note, True
    if shift is None:
        return None, note, False
    if holes:
        return fold(hs, shift), note, True
    return float(shift), note, False


def line_red(line):
    """The red samples of a raw 8-bit stationary line in image coordinates (mirrored, the first CROP CCD pixels dropped)."""
    import numpy as np

    return np.frombuffer(line, np.uint8)[0::3][::-1].astype(float)[:WIDTH]


def whole_holes(red):
    """Centres of the uncut sprocket holes in the hole window of one stationary line (red): bright runs of WHOLE_HOLE
    width that do not touch the ends of the searched span. The window is wider than two pitches, so there is always one."""
    import numpy as np

    lo, hi = HOLE_WINDOW[0] - 20, HOLE_WINDOW[1] + 20
    seg = red[lo:hi]
    thr = 0.5 * (seg.max() + np.median(seg))
    centres, start = [], None
    for x, on in enumerate([*(seg > thr), False]):
        if on and start is None:
            start = x
        if not on and start is not None:
            if WHOLE_HOLE[0] <= x - start <= WHOLE_HOLE[1] and start > 0 and x < len(seg):
                centres.append(lo + (start + x - 1) / 2)
            start = None
    return centres


def hole_travel(before, after, expect):
    """Film travel in pixels (negative = forward) between two sets of uncut hole centres: the shift of a hole, exact
    modulo the perforation pitch, folded next to `expect` (which must be right to within half a pitch, 170 px). None when
    either set is empty (no film over the hole row)."""
    if not before or not after:
        return None
    d = after[0] - before[0]
    return expect + (d - expect + PERF_PITCH / 2) % PERF_PITCH - PERF_PITCH / 2


def frame_gaps(r, flat=GAP_FLAT, min_len=MIN_GAP):
    """Inter-frame gaps of a negative as column runs (start, end): at least `min_len` columns whose spread over the rows is
    below `flat` and whose level is at least GAP_BASE of the film base (the 99.5th percentile); the dark mask is excluded."""
    import numpy as np

    level, spread, base = r.mean(0), r.std(0), np.percentile(r, 99.5)
    runs, start = [], None
    for x, ok in enumerate([*((spread < flat) & (level >= GAP_BASE * base)), False]):
        if ok and start is None:
            start = x
        if not ok and start is not None:
            if x - start >= min_len:
                runs.append((start, x - 1))
            start = None
    return runs


def plausible_gaps(r, gaps):
    """Drop the gaps that are a flat, bright patch of picture rather than clear film between frames. A real gap is clear
    film base over every row; a bright patch dips somewhere. So a gap is dropped when it is both rougher (its mean spread
    over the rows more than FALSE_SPREAD times that of the flattest gap REF_GAP columns wide) and dips lower (its darkest
    row more than FALSE_DIP below the best such gap's, as a fraction of the film base); CUT_FALSE for a gap cut by the
    gate's ends. With no real gap in view nothing stands out and all are kept. rw1, proof6, pc1: the patches had spread
    4.4 to 7.0 and darkest rows at 0.57 to 0.79, the real gaps 3.4 to 3.7 and 0.85 to 0.87."""
    import numpy as np

    base = np.percentile(r, 99.5)
    spread, rows = r.std(0), r / base
    stats = [(float(spread[s : e + 1].mean()), float(rows[:, s : e + 1].mean(1).min())) for s, e in gaps]
    ref = [st for (s, e), st in zip(gaps, stats) if REF_GAP[0] <= e - s + 1 <= REF_GAP[1]]
    if not ref:
        return gaps
    flat, dip = min(st[0] for st in ref), max(st[1] for st in ref)

    def false(gap, sp, lo):
        cut = gap[0] == 0 or gap[1] >= GATE_END - GATE_TOUCH
        times, below = (CUT_FALSE if cut else (FALSE_SPREAD, FALSE_DIP))
        return sp > times * flat and lo < dip - below

    return [g for g, (sp, lo) in zip(gaps, stats) if not false(g, sp, lo)]


def clear_gaps(r):
    """Runs (start, end) of clear film in a negative by each column's darkest row band, for a scratched film (see CLEAR_SHARE)."""
    import numpy as np

    bands = np.array_split(r[:, :GATE_END], CLEAR_BANDS)
    level = np.min([np.median(band, axis=0) for band in bands], axis=0)
    level = np.convolve(np.pad(level, 4, mode="edge"), np.ones(9) / 9, "valid")
    runs, start = [], None
    for x, ok in enumerate([*(level >= CLEAR_SHARE * np.percentile(r, 99.5)), False]):
        if ok and start is None:
            start = x
        if not ok and start is not None:
            if runs and start - runs[-1][1] <= CLEAR_BRIDGE:
                runs[-1] = (runs[-1][0], x - 1)
            else:
                runs.append((start, x - 1))
            start = None
    return runs


def scratched_edge(r, at_least=0):
    """A scratched film's frame edge by its clear gaps (see CLEAR_SHARE), or None: the end of a gap cut by the gate start, the
    left edge of a frame already in the gate before a whole gap, or the first whole gap's end at or right of `at_least`."""
    for s, e in clear_gaps(r):
        if s == 0 and EDGE_GAP <= e + 1 < WIDE_GAP:  # cut by the gate start: its end is the frame's edge (roll52, 0 to 209)
            return e + 1
        if SCRATCH_GAP <= e - s + 1 < WIDE_GAP and s > 0 and e < GATE_END - GATE_TOUCH:
            if -CUT_OK <= s - FRAME_PX <= EDGE_AT:  # the gap after a frame in the gate, as in frame_edge()
                return s - FRAME_PX
            if e + 1 >= at_least:
                return e + 1
    return None


def frame_edge(r, at_least=0, gap=GAP_WIDTH):
    """The left edge of the frame to align: a frame whose left edge is near the start of the gate (a gap cut by the start
    of the gate ends at its left edge), or a frame already in the gate whose leading gap has gone past the start (the
    strip was fed a little too far): the gap after it, running into the mask at the end of the gate or followed by the
    next frame, starts FRAME_PX after the frame's left edge, which may then lie up to CUT_OK pixels before the gate
    (negative: that much of the frame is lost, better than feeding it out of the gate). Otherwise the first edge at or
    right of `at_least` after an inner gap of at least MIN_GAP columns (its right end + 1, or its left end + `gap` when it
    is wider than WIDE_GAP, a thin, uniform part of the next frame then continuing it). A gap running into the gate's end
    gives way to such an inner edge when its darkest row lies more than FALSE_DIP lower than the inner gap's (roll36b and
    roll36c frame 7: pale picture at the gate end, darkest row 0.74 against 0.93, put the frame at -69 instead of feeding the
    real gap's edge, 207, in). With clear film in the gate an inner gap must overlap it, and with no edge from the flat runs
    scratched_edge() has the last word, so None if neither finds one."""
    import numpy as np

    level, spread, base = r.mean(0), r.std(0), np.percentile(r, 99.5)

    def dip(s, e):  # the darkest row of a gap as a share of the film base: a real gap is clear film over every row
        return float((r[:, s : e + 1].mean(1) / base).min())

    def mask(x):  # the mask beyond the gate: the dark floor, uniform (dense film still varies from row to row), and only
        # at the gate end: very dense film reads as dark and flat in this scan (roll37 frame 11: the frame after a real gap
        # at 234-345 passed for the mask, the gap was taken for the one before the gate end, and the frame was missed)
        return x >= len(level) or (x >= GATE_END - GATE_TOUCH and level[x] < 0.15 * base and spread[x] < 2)

    clear = clear_gaps(r)
    placed, inner, trailing, touching = [], [], [], []
    for s, e in plausible_gaps(r, frame_gaps(r, min_len=EDGE_GAP)):
        if s == 0:  # wider than a gap: gap and pale picture run together, and where the gap ends is unknown; under-feeding
            placed.append(e + 1 if e < WIDE_GAP else EDGE_AT)  # can be put right after the frame scan, overshooting cannot
        elif mask(e + 4):
            placed.append(s - FRAME_PX)
        elif e >= GATE_END - GATE_TOUCH:  # runs into the mask unseen; its estimate of the frame edge is the less exact
            touching.append((s - FRAME_PX, dip(s, e)))
        elif e - s + 1 >= MIN_GAP and (not clear or any(cs <= e and s <= ce for cs, ce in clear)):
            # with clear film elsewhere in the gate, a flat run that is not clear film is an even wall (roll52b align02: 2022
            # to 2123 beside the real gap at 2512, and the frame in the gate between them was fed past)
            inner.append((e + 1 if e - s < WIDE_GAP else s + gap, dip(s, e)))
            if -CUT_OK <= s - FRAME_PX <= EDGE_AT:
                trailing.append((s, s - FRAME_PX))
    placed = [x for x in placed if x >= -CUT_OK]
    if placed:
        return min(placed)
    inner = sorted((x, d) for x, d in inner if x >= at_least)
    if not (inner and inner[0][0] <= GATE_END - FRAME_PX):  # no inner edge that leaves the whole frame in the gate
        clear = inner[0][1] - FALSE_DIP if inner else 0.0
        touching = [x for x, d in touching if x >= max(-CUT_OK, at_least - CUT_OK) and d >= clear]
        if touching:
            return min(touching)
    for s, left in trailing:  # the gap after a frame in the gate, with no frame edge before it (roll18, roll19)
        if not any(x < s for x, _ in inner):
            return left
    return inner[0][0] if inner else scratched_edge(r, at_least)


def frame_scan_gaps(r):
    """The real gaps in a strip frame scan (`r`: its plane): over the picture rows FRAME_ROWS, scaled to the alignment scan's
    level so the gap finder's thresholds hold, and no wider than a gap (not open light). By clear_gaps() where the flatness
    test finds none: on a scratched strip early_edge() and late_edge() would otherwise see nothing (roll51 frames 2 and 3)."""
    import numpy as np

    r = r[FRAME_ROWS[0] : FRAME_ROWS[1] : 3, : GATE_END + 64].astype(float)
    r *= 80.0 / max(1.0, float(np.percentile(r[:, :GATE_END], 99.5)))  # film base about 80, as at ALIGN_GAIN
    gaps = plausible_gaps(r, frame_gaps(r, min_len=EDGE_GAP))
    gaps = [(s, e) for s, e in gaps if (s == 0 or MIN_GAP <= e - s + 1) and e - s + 1 < 400]
    return gaps or [(s, e) for s, e in clear_gaps(r) if (EDGE_GAP if s == 0 else MIN_GAP) <= e - s + 1 < 400]


def next_edge(r):
    """The next frame's left edge in a strip frame scan (`r`: its plane) when the gap after this frame shows in the gate's
    second half: its right end + 1, or, where it runs into the gate end or is wider than WIDE_GAP, its left end + GAP_WIDTH.
    None when no such gap shows. roll36d frame 19 lay 700 px too far and showed the next frame from 2024, which a whole
    pitch then fed past the gate start: that frame was skipped."""
    gaps = [(s, e) for s, e in frame_scan_gaps(r) if s > GATE_END // 2]
    if not gaps:
        return None
    s, e = gaps[0]
    return e + 1 if e < GATE_END - GATE_TOUCH and e - s < WIDE_GAP else s + GAP_WIDTH


def left_edge(r):
    """A strip frame's own left edge in its scan (`r`: its plane): the right end + 1 of the gap at the gate start. None when
    none shows (the edge lies before the gate, or the gap is too short for the gap finder)."""
    for s, e in frame_scan_gaps(r):
        if s == 0:
            return e + 1
    return None


def early_edge(r):
    """The frame's left edge (negative) in a strip frame scan (`r`: its plane) when the frame was fed so far that more than
    RESCAN_LOSS of its left end lies before the gate: the gap after the frame, in the gate's second half, starts FRAME_PX
    after that edge. None when the frame is placed well enough, or no such gap shows."""
    gaps = [(s, e) for s, e in frame_scan_gaps(r) if s > GATE_END // 2]
    if not gaps or gaps[0][0] >= FRAME_PX - RESCAN_LOSS:
        return None
    return gaps[0][0] - FRAME_PX


def late_edge(r):
    """The frame's left edge in a strip frame scan (`r`: its plane) when the frame lies so far in that more than RESCAN_LOSS
    of its right end falls past the gate: the end of the first real gap over the picture rows FRAME_ROWS (scaled to the
    alignment scan's level, so the gap finder's thresholds hold). None when the frame is placed well enough, or the scan
    shows no gap in the gate's first half."""
    gaps = frame_scan_gaps(r)
    if not gaps or gaps[0][0] > GATE_END // 2:
        return None
    s, e = gaps[0]
    edge = e + 1 if s == 0 or e - s < WIDE_GAP else s + GAP_WIDTH
    return edge if edge > GATE_END - FRAME_PX + RESCAN_LOSS else None


def is_open_light(r):
    """True when an alignment plane shows open light over most of the gate (OPEN_LIGHT): the strip's end has gone by."""
    import numpy as np

    return bool(np.median(r.mean(0)[:GATE_END]) >= OPEN_LIGHT)


def is_leader(r):
    """True when an alignment plane shows only a roll's fogged leader: near-black from the gate start to within
    GATE_TOUCH of its end."""
    level = r.mean(0)[: GATE_END - GATE_TOUCH]
    return bool((level < LEADER_DARK).all())


class EndOfFilm(ScannerError):
    pass


class MCUTimeout(ScannerError):
    pass


class PF1800:
    """One scanner session: `with PF1800() as scanner:`. All moves are bounded and every wait has a timeout."""

    def __init__(self, dry_run=False, trace=None, log=print):
        self.dry_run = dry_run
        self.trace = dry_run if trace is None else trace
        self.log = log
        self.mcu_wait = MCU_WAIT_S  # seconds one MCU handshake step may take (firmware_version's retry allows longer)
        self.tr = None
        self.position = None  # steps past the reference edge, None while unknown
        self._rest_holes = None  # the last hole read, while the film has not moved since (mcu_command() clears it)
        self.speed = SPEED  # byte of register 0x88 for carriage moves (probe_move_speed.py tries others)
        self.line = LINE_PERIOD  # byte of registers 0x88 and 0xC6 for scans (scan_frames() lengthens a strip's frames)
        self.lamp_is_on = False
        self.lamp_level = LAMP_LEVEL  # first byte of SetLamp (register 0x8D, bank 0x20); the vendor's lamp calibration lowers it
        self.lens_used = False
        self.film = None  # 'strip', 'slide' or 'none' as film_type() last saw it
        self.focus = None  # lens units from home applied before each scan (None: leave the lens where it is)

    # ---- session -----------------------------------------------------------------------------------------------------

    def __enter__(self):
        self.open()
        return self

    def __exit__(self, *exc):
        self.close()

    def open(self):
        dev = None
        if not self.dry_run:
            dev = usb.core.find(idVendor=VID, idProduct=PID)
            if dev is None:
                raise ScannerError(f"{VID:04x}:{PID:04x} not found: plugged in and switched on?")
            try:
                dev.set_configuration()
            except usb.core.USBError:
                pass  # already configured
            usb.util.claim_interface(dev, 0)
        self.tr = Transport(dev, not self.dry_run, self.trace)
        for byte in (0x64, 0x65, 0x44, 0x45):  # OpenScanner
            self.tr.ctrl_out(0x0090, 0, bytes([byte]))
        # The vendor's identification resets the chip before its first MCU command; without it the first
        # MCU handshake after a power cycle or reconnect timed out (status 0xF1) and only a retry, after close()'s reset, ran.
        self.safe_initial_chip()

    def close(self):
        """Best-effort park: chip safe state, lens home when used, carriage to the home zone, lamp off, close handshake."""
        if self.tr is None:
            return
        try:
            self.park()
            for byte in (0x64, 0x65, 0x16, 0x17):  # CloseScanner
                self.tr.ctrl_out(0x0090, 0, bytes([byte]))
        except (usb.core.USBError, ScannerError) as e:
            self.log(f"close: {e} (switch the scanner off with its power switch if it stops answering)")
        finally:
            if self.tr.dev is not None:
                usb.util.dispose_resources(self.tr.dev)
            self.tr = None

    def park(self):
        """Every step is tried even when an earlier one fails, so the lamp always goes off; the first error is raised."""
        steps = [self.stop_motor, self.safe_initial_chip]
        if self.lens_used:
            steps.append(self.lens_home)
        if self.position is not None:
            steps.append(self.seek_home)
        steps.append(lambda: self.lamp(False))
        error = None
        for step in steps:
            try:
                step()
            except (usb.core.USBError, ScannerError) as e:
                error = error or e
        if error:
            raise error

    # ---- chip functions --------------------------------------------------------------------------------------------

    def status(self, page=0):
        self.tr.write_reg(0x97, page)
        return self.tr.read_reg(0x8B)

    def _need(self, bits, where):
        value = self.status(0)
        if value is not None and value & bits != bits:
            raise ScannerError(f"status bits 0x{bits:02X} not set after {where} (status 0x{value:02X})")

    def safe_initial_chip(self):
        for reg in (0x90, 0x80, 0x84, 0xBE, 0xC0, 0x86, 0x89, 0x94):
            self.tr.write_reg(reg, 0)
        self.tr.write_reg(0x01, 0x02)
        self.tr.clean_buffer()

    def lamp(self, on):
        for reg, val in ((0xA8, 0x80), (0x83, 0x20), (0x8D, self.lamp_level), (0x83, 0x00), (0x8D, LAMP_SECOND)):
            self.tr.write_reg(reg, val)
        self.tr.write_reg(0x93, 0x06 if on else 0x05)  # lamp 1 on (the lit lamp), or neither
        self.lamp_is_on = on

    def set_gpio(self, a=0xE1FD, b=0xF1FF):
        for reg, val in ((0x83, 0), (0xA3, ~a & 0xFF), (0xA4, ~(a >> 8) & 0xFF), (0xA1, b & 0xFF), (0xA2, (b >> 8) & 0xFF)):
            self.tr.write_reg(reg, val)

    def driving_capability(self, cf=0x0C):
        """DrivingCapabilitySetting with the vendor's struct: byte 6 = 4 before every call, byte 7 = 8."""
        self.tr.write_reg(0xCE, 0)
        self.tr.write_reg(0xCF, cf)

    def set_ccd_pixel_number(self, n):
        for reg, val in ((0x83, 0x22), (0x87, n & 0xFF), (0x83, 0x02), (0x87, (n >> 8) & 0xFF)):
            self.tr.write_reg(reg, val)

    def set_ccd_timing(self, t):
        w = self.tr.write_reg
        w(0x83, 0)
        w(0xA0, t[0x0D] | t[0x0C] | t[2])
        w(0x9C, (t[3] & 0xC7) | 0x80)
        w(0x9D, t[4] | 0x40)
        w(0x9E, t[5])
        w(0x9F, t[6] & 0xBF)
        w(0x83, 0x20)
        w(0xA5, t[7])
        w(0xA6, t[8])
        w(0xA7, t[9])

    def set_adc(self, gains, offsets, channel=None):
        """SetADC. With a `channel` the front end runs in mono mode (setup register 1 = 7 instead of 3) and converts only
        that colour (register 3 bits 7-6: 0 red, 1 green, 2 blue) for grey scans."""
        w = self.tr.write_reg
        mono = channel is not None
        afe_init = ((0x80, 0), (0x80, 1), (0x04, 0x80), (0x04, 0), (0x00, 0), (0x01, 0x07 if mono else 0x03),
                    (0x03, 0x12 | (channel << 6 if mono else 0)), (0x02, 0x04), (0x05, 0))
        for reg, val in afe_init:
            w(reg, val)
        for reg, val in zip((0x28, 0x29, 0x2A, 0x20, 0x21, 0x22), (*gains, *offsets)):
            w(reg, val)
        for reg in (0x24, 0x25, 0x26):
            w(reg, 0)

    def write_table(self, table):
        """WriteTable: three 16 KB chunks at RAM 0x1F4000, 0x1F8000, 0x1FC000 (per-pixel words; bit 13 = pixel flagged)."""
        w = self.tr.write_reg
        for i, val in enumerate((0x00, 0x10, 0x1F, 0xFF, 0xFF, 0x1F)):
            w(0xB0 + i, val)
        third = len(table) // 3
        for k, addr in enumerate((0x1F4000, 0x1F8000, 0x1FC000)):
            w(0x94, 0x41)
            for i in range(3):
                w(0xB0 + i, (addr >> (8 * i)) & 0xFF)
            for i, val in enumerate((0xFF, 0xFF, 0x1F)):
                w(0xB3 + i, val)
            self.tr.write_line(table[k * third : (k + 1) * third])

    def set_image_buffer_range(self, line_bytes, depth=8, grey=False):
        end = (0x70FFE - line_bytes) >> 5
        w = self.tr.write_reg
        w(0x94, 0x61 if depth == 16 and not grey else 0x41)  # the vendor's 0x61 is for 48-bit colour only
        for i in range(3):
            w(0xB0 + i, 0)
        for i, val in enumerate((0xFF, 0x0F, 0x07)):
            w(0xB3 + i, val)
        for i, val in enumerate((1, 0, end & 0xFF, end >> 8)):
            w(0xB6 + i, val)

    def set_motor_type(self, enabled):
        w = self.tr.write_reg
        if not enabled:
            w(0x84, 0x00)
            w(0x80, 0x01)
            return
        w(0xC0, 0x00)
        w(0x84, 0x00)
        w(0x84, 0xB1)
        w(0x80, 0xE1)

    def set_scan_acc_dec(self, count, depth=8, grey=False, step=1):
        """The line period and the step count (one motor step per line); register values of the vendor's struct."""
        w = self.tr.write_reg
        w(0x83, 0x00)
        w(0x85, 0x00)
        w(0x83, 0x20)
        w(0x85, step)  # motor steps per output line
        w(0xBC, 0)
        w(0xBD, 0)
        speed = PROOF_SPEED if step > 1 else self.line
        w(0x88, speed)
        w(0xC1, 0)
        w(0xC2, 0)
        w(0xC3, 1)
        w(0xC6, speed)
        w(0xC7, 0)
        # 1 + 0xBB (the vendor's struct byte 1)
        w(0xC8, PROOF_BB + 1 if step > 1 else GREY_BB + 1 if grey and depth == 16 else 5 if depth == 16 else 4)
        w(0xC0, 0xC0)
        w(0x89, count & 0xFF)
        w(0x86, (count >> 8) & 0xFF)

    def set_auto_scan(self, depth=8, grey=False, channel=GREY_CHANNEL, step=1):
        """SetAutoScan with the vendor's struct for 24-bit (mode 0x18) or 48-bit (0x30) lines, or grey: 16-bit (mode 0x10
        with the mono flag, bit 7 of 0x90, which SetAutoScan itself only sets for 8-bit grey) or 8-bit (mode 8: 0x90 0x80,
        0xBE | 3), 0xBF = the channel. Bit 5 of 0x90 (the gamma look-up) stays off because no look-up table is uploaded; 16
        bits per sample set bit 6 as SetAutoScan does. 0xBB 4 for 16 bits is the vendor's 48-bit value; 16-bit grey uses
        GREY_BB to stay under the bus limit (8-bit grey runs at 148 lines/s with 0xBB 3, well inside it)."""
        w = self.tr.write_reg
        w(0xBB, PROOF_BB if step > 1 else (GREY_BB if grey else 4) if depth == 16 else 3)
        w(0x9B, 0x10 | 4 | 0x04)
        w(0x8E, 0)
        w(0x8B, 0)
        w(0xBF, channel if grey else 0)
        w(0x90, (0xC0 if grey else 0x40) if depth == 16 else (0x80 if grey else 0))
        w(0x91, 0)
        w(0x83, 0x82)
        self.tr.clean_buffer()
        self.tr.read_small(1)
        w(0xBE, 0x0F if grey and depth == 8 else 0x0D)  # start

    # ---- carriage -----------------------------------------------------------------------------------------------------

    def motor_move(self, mode, steps):
        """MotorMoveNormal: mode 0 = away from home, mode 1 = toward home, mode 3 = stop."""
        w = self.tr.write_reg
        w(0xC0, 0)
        w(0x80, 0xE1 if mode == 0 else 0xC1)
        w(0x84, 0xB1)
        w(0xC8, 0)
        w(0x88, self.speed)
        w(0x89, steps & 0xFF)
        w(0x86, (steps >> 8) & 0xFF)

    def stop_motor(self):
        self.motor_move(3, 0)

    def _prepare_motor(self):
        self.safe_initial_chip()
        self.set_gpio()
        self.driving_capability()

    def _wait(self, test, limit, what):
        if self.dry_run:
            return 0.0
        t0 = time.time()
        while not test(self.status(0)):
            if time.time() - t0 > limit:
                self.stop_motor()
                raise ScannerError(f"{what} did not finish within {limit:.0f} s")
            time.sleep(0.05)
        return time.time() - t0

    def move(self, mode, steps):
        """One vendor-style move: MotorMoveNormal, wait for status & 3 == 3, stop move."""
        if not 0 < steps <= MAX_MOVE:
            raise ScannerError(f"refusing a move of {steps} steps (limit {MAX_MOVE})")
        self._prepare_motor()
        self.motor_move(mode, steps)
        self._wait(lambda s: s & 0x03 == 0x03, steps / 250 + 3, f"the {steps}-step move")
        self.stop_motor()

    def in_home_zone(self):
        value = self.status(0)
        return None if value is None else bool(value & 0x40)

    def reference(self):
        """Release the home zone going away: the position is then 0 at the exit edge. A carriage out of the zone (a run
        killed before it could park) is homed first."""
        if self.in_home_zone() is False:
            self.log("the carriage is not in the home zone: homing it first")
            self.seek_home()
        self._prepare_motor()
        self.motor_move(0, 200)
        took = self._wait(lambda s: not s & 0x40, 2.5, "the release of the home zone")
        self.stop_motor()
        self.position = 0
        self.log(f"reference edge: home zone cleared after {took:.2f} s")

    def seek_home(self):
        """Mode 1 (cap 4000 steps) until the home sensor is set. Only safe from a position past the home zone."""
        self._prepare_motor()
        self.motor_move(1, 4000)
        self._wait(lambda s: bool(s & 0x40), 20.0, "the seek of the home zone")
        self.stop_motor()
        self.position = None

    def go_to(self, target):
        """Move to `target` steps past the reference edge (0 to MAX_POSITION). Every position is reached moving away from
        home: a move toward home overshoots by BACKLASH_OVERSHOOT and comes back out, because reversing the drive leaves
        the carriage about 45 steps further out than commanded (probe36_window.py)."""
        if self.position is None:
            raise ScannerError("no reference position: call reference() first")
        if not 0 <= target <= MAX_POSITION:
            raise ScannerError(f"position {target} outside 0..{MAX_POSITION}")
        if target > self.position:
            self.move(0, target - self.position)
        elif target < self.position:
            self.speed = RETURN_SPEED  # a stall on the way back only leaves the carriage short of home
            try:
                self.move(1, self.position - target + BACKLASH_OVERSHOOT)
            finally:
                self.speed = SPEED
            self.move(0, BACKLASH_OVERSHOOT)
        self.position = target

    # ---- MCU command interface (lens, firmware) -----------------------------------------------------------------------

    def _poll_a(self):
        self.tr.write_regs((0x97, 0x0A), (0xA3, 0x1E))
        return self.tr.read_reg(0x8B, twice=True)

    def _wait_bit0(self, want, code=None):
        """Polls until bit 0 is `want` and returns that reading: the first poll can still be the idle level, whose bits
        4..2 are no reply. With `code`, says so when it was (a late reply, which an MCU kept busy is most likely to give)."""
        if self.dry_run:
            return None
        t0, first, value, told = time.time(), None, None, False
        while time.time() - t0 < self.mcu_wait:
            value = self._poll_a()
            first = value if first is None else first
            if value & 1 == want:
                if code is not None and first & 1 != want:
                    self.log(f"MCU code {code}: late reply 0x{value:02X} (first poll 0x{first:02X})")
                return value
            if not told and time.time() - t0 > MCU_WAIT_S:
                told = True
                self.log(f"the MCU has not answered for {MCU_WAIT_S:.0f} s (status 0x{value:02X}); "
                         f"waiting up to {self.mcu_wait:.0f} s")
        raise MCUTimeout(f"MCU interface: bit 0 did not become {want} within {self.mcu_wait:.0f} s "
                         f"(first status 0x{first:02X}, last 0x{value:02X})")

    def _mcu_code(self, code):
        v = code << 2
        self.tr.write_regs((0x83, 0), (0xA3, 0x1E), (0xA1, v))
        self.tr.write_regs((0xA1, v | 2))
        self.tr.write_regs((0xA1, v))
        reply = self._wait_bit0(0, code)
        self.tr.write_regs((0x83, 0), (0xA3, 0x1E), (0xA1, v | 2))
        self._wait_bit0(1)
        return None if reply is None else (reply & 0x1C) >> 2

    def mcu_command(self, cmd, sub=None):
        """A command code (repeated while the reply is 5, busy), then for cmd 0 the sub-command after reply 2.
        The vendor repeats a busy command without limit; a long lens move (homing from +120 units) keeps it busy for a
        while, so this retries for up to MCU_BUSY_S seconds. Any other reply to cmd 0 is an error: the lens would not move
        and nothing else would show it."""
        if cmd not in (0, 4):  # the lens and the firmware query leave the film where it is
            self._rest_holes = None
        t0 = time.time()
        while True:
            reply = self._mcu_code(cmd)
            if reply != 5:
                break
            if time.time() - t0 > MCU_BUSY_S:
                raise ScannerError(f"the MCU kept answering busy for {MCU_BUSY_S} s")
            time.sleep(0.1)
        if cmd == 0 and sub is not None:
            if reply not in (None, 2):
                raise ScannerError(f"the MCU refused lens command {sub}: reply code {reply}")
            answer = self._mcu_code(sub)
            if answer not in (None, 4):
                self.log(f"MCU: lens command {sub} answered code {answer}")

    def firmware_version(self):
        """The command 4 handshake twice; the version is in status bits 4..2 of the second pass. The first
        MCU command of a session: when it goes unanswered (MCU_WAIT_S), the session is closed and reopened as a rerun
        would (open, film_type), and the command tried once more, waiting up to MCU_FIRST_WAIT_S."""
        try:
            self.mcu_command(4)
        except MCUTimeout as e:
            self.log(f"first MCU handshake unanswered ({e}); reopening the session and trying again")
            self.close()
            time.sleep(MCU_REOPEN_S)
            self.open()
            self.film_type()
            self.mcu_wait = MCU_FIRST_WAIT_S
            try:
                self.mcu_command(4)
            finally:
                self.mcu_wait = MCU_WAIT_S
        v = 0x10
        self.tr.write_regs((0x83, 0), (0xA3, 0x1E), (0xA1, v))
        self.tr.write_regs((0xA1, v | 2))
        self.tr.write_regs((0xA1, v))
        self.tr.write_regs((0x83, 0))
        status = self._wait_bit0(0, 4)
        self.tr.write_regs((0x83, 0), (0xA3, 0x1E), (0xA1, v | 2))
        self._wait_bit0(1)
        return None if status is None else (status & 0x1C) >> 2

    def lens_home(self):
        self.mcu_command(0, 6)

    def lens_move(self, units):
        """`units` lens steps away from home (sub 2 = 10, sub 1 = 5, sub 0 = 1)."""
        self.lens_used = True
        for _ in range(units // 10):
            self.mcu_command(0, 2)
        rest = units % 10
        if rest >= 5:
            self.mcu_command(0, 1)
            rest -= 5
        for _ in range(rest):
            self.mcu_command(0, 0)

    def autofocus(self, position, start=0, step=5, count=16, reads=2, more=0):
        """Quick autofocus in the vendor's manner (stationary lines, one lens step between them, keep the
        sharpest): at `position` steps past the reference edge, lens home, then `count` lens positions start, start + step,
        ...; at each, `reads` stationary 8-bit lines of the whole CCD and their lateral sharpness (red, normalised gradient
        energy); while the last position is the sharpest, up to `more` further positions. Sets and returns `self.focus`
        (lens units from home, refined by a parabola through the best three) and leaves the lens there. Takes about 10 s.
        The position must hold picture detail (the middle of a frame)."""
        if self.position is None:
            self.reference()
        self.go_to(position)
        self.lens_home()
        self.lens_used = True
        if start:
            self.lens_move(start)
        scores = []
        t0 = time.time()
        k = 0
        while k < count or (k < count + more and scores[-1] == max(scores) > 0):
            if k:
                self.lens_move(step)
            k += 1
            self._setup_stationary()
            lines = [self._read_stationary(n > 0) for n in range(reads)]
            if any(line is None for line in lines):
                scores.append(0.0)
                continue
            score = 0.0
            for line in lines:
                red = line[3 * (CROP + 200) : 3 * (PIXELS - 200) : 3]
                mean = sum(red) / len(red)
                score += sum((b - a) ** 2 for a, b in zip(red, red[1:])) / len(red) / max(1.0, mean) ** 2 * 1e4
            scores.append(score / reads)
        self.safe_initial_chip()
        best = max(range(len(scores)), key=scores.__getitem__)
        focus = start + best * step
        if 0 < best < len(scores) - 1:
            a, b, c = scores[best - 1], scores[best], scores[best + 1]
            if a - 2 * b + c < 0:
                focus += round(0.5 * (a - c) / (a - 2 * b + c) * step)
        focus = max(0, focus)
        if not self.dry_run:
            self.log("autofocus: " + " ".join(f"{start + k * step}:{v:.1f}" for k, v in enumerate(scores))
                     + f" -> lens +{focus} units in {time.time() - t0:.1f} s")
        self.lens_home()
        self.lens_move(focus)
        self.focus = focus
        self._lens_at = focus
        return focus

    def _setup_stationary(self):
        """The verified stationary calibration-style read (variant A, 8-bit, whole CCD line, motor disabled, count 3)."""
        gains, offsets = DEFAULT_LEVELS[8]
        w = self.tr.write_reg
        self.tr.stream_started = False
        self.safe_initial_chip()
        self.lamp(True)
        self.set_gpio()
        self.driving_capability()
        self.set_ccd_pixel_number(CCD_PIXELS + 0x40)
        self.set_ccd_timing(TIMING)
        self.set_adc(gains, offsets)
        self.write_table(self._scan_table())
        self.set_image_buffer_range(PIXELS * 3)
        self.set_motor_type(False)
        self.set_scan_acc_dec(3)
        for reg, val in ((0xC3, 0),):  # the stationary struct has byte 9 = 0
            w(reg, val)
        self.set_auto_scan()

    def _read_stationary(self, rearm):
        """One 8-bit line; with `rearm` a QuickInitialAutoScan first (start bit off, levels, empty WriteTable, start)."""
        if rearm:
            gains, offsets = DEFAULT_LEVELS[8]
            w = self.tr.write_reg
            w(0xBE, 0x0C)
            for reg, val in zip((0x28, 0x29, 0x2A, 0x20, 0x21, 0x22), (*gains, *offsets)):
                w(reg, val)
            for i, val in enumerate((0x00, 0x10, 0x1F, 0xFF, 0xFF, 0x1F)):
                w(0xB0 + i, val)
            for reg in (0xB0, 0xB1, 0xB2):
                w(reg, 0)
            self.tr.read_small(1)
            w(0xBE, 0x0D)
            self.tr.stream_started = False
        data = bytearray()
        while len(data) < PIXELS * 3:
            chunk = self.tr.stream_chunk()
            if chunk is None:
                return None
            data += chunk
        return bytes(data[: PIXELS * 3])

    # ---- film holder sensors ------------------------------------------------------------------------------------------

    def film_type(self):
        """The vendor's detection: 'strip' (page 0x0B bits 7/6), 'slide' (page 0x0A bit 5 clear), else 'none'."""
        self.set_gpio()
        self.tr.write_regs((0x97, 0x0A), (0xA3, 0x1E))
        a = self.tr.read_reg(0x8B, twice=True)
        self.tr.write_regs((0x97, 0x0B), (0xA4, 0xFF))
        b = self.tr.read_reg(0x8B, twice=True)
        if a is None or b is None:
            return None
        self.film = "strip" if b & 0xC0 else "slide" if a & 0x20 == 0 else "none"
        return self.film

    # ---- scanning -----------------------------------------------------------------------------------------------------

    def _scan_table(self, step=1):
        table = bytearray(0xC000)
        for i in range(LEFT, LEFT + scan_pixels(step) * step, step):  # the vendor marks every step-th pixel
            table[2 * i + 1] |= 0x20  # bit 13 of the little-endian word, first chunk
        return bytes(table)

    def _setup_scan(self, lines, gains, offsets, depth=8, grey=False, channel=GREY_CHANNEL, step=1):
        self.tr.stream_started = False
        self.safe_initial_chip()
        self.lamp(True)
        if not self.dry_run:
            time.sleep(0.5)
        self._need(0x02, "the lamp setup")
        self.set_gpio()  # the order of the vendor's scan setup, as verified by the full scans
        self.driving_capability()
        self.set_ccd_pixel_number(CCD_PIXELS + 0x40)
        timing = list(TIMING)
        if step > 1:  # the vendor's t[7..9] (registers 0xA5 to 0xA7) at 300 dpi: 0, 0, 2 for a strip, 3, 3, 3 for a slide
            timing[7:10] = (0, 0, 2) if self.film == "strip" else (3, 3, 3)
        self.set_ccd_timing(timing)
        self.set_adc(gains, offsets, channel if grey else None)
        self.write_table(self._scan_table(step))
        self.set_image_buffer_range(scan_pixels(step) * (depth // 8 if grey else 6 if depth == 16 else 3), depth, grey)
        self._need(0x02, "the chip setup")
        self.set_motor_type(True)
        self.set_scan_acc_dec(lines * step + 2, depth, grey, step)
        self._need(0x02, "the motor and line setup")
        self.set_auto_scan(depth, grey, channel, step)

    def scan(self, lines=1800, approach=1550, gains=None, offsets=None, warmup=10.0, progress=None, depth=8, grey=False,
             channel=GREY_CHANNEL, step=1):
        """Lamp on, reference, move to `approach`, then `lines` motor-driven lines of 8 or 16 bits per sample (`depth`); with
        `grey` single-channel lines of the `channel` colour (16-bit 98 lines/s against 48 for 16-bit colour, 8-bit 148
        against 74; the level is about a third of colour at the same gain, see grey_gain()). Returns a Frame; the carriage
        is left `lines * step + 2` steps further on (park() or close() brings it home). `step` PROOF_STEP is the 300 dpi proof
        scan (grey only): every 6th pixel, 6 motor steps per line."""
        if step not in (1, PROOF_STEP) or (step > 1 and not (grey and depth == 8)):
            raise ScannerError(f"step must be 1, or {PROOF_STEP} for an 8-bit grey proof scan")
        if depth not in (8, 16):
            raise ScannerError("depth must be 8 or 16")
        gains = gains or DEFAULT_LEVELS[depth][0]
        offsets = offsets or DEFAULT_LEVELS[depth][1]
        line_bytes = scan_pixels(step) * (depth // 8 if grey else 6 if depth == 16 else 3)
        if not 9 <= lines <= 1800 // step:
            raise ScannerError(f"lines must be 9..{1800 // step}")
        if approach + lines * step + 2 > MAX_POSITION:
            raise ScannerError(f"the scan would end at {approach + lines * step + 2} steps, beyond {MAX_POSITION}")
        self.lamp(True)
        if not self.dry_run:
            time.sleep(warmup)
        if self.position is None:
            self.reference()
        if self.focus is not None and getattr(self, "_lens_at", None) != self.focus:
            self.lens_home()  # the stored focus step, counted from lens home, before every scan
            self.lens_move(self.focus)
            self._lens_at = self.focus
        self.go_to(approach)
        self._setup_scan(lines, gains, offsets, depth, grey, channel, step)
        stream = bytearray()
        sizes = 0
        while len(stream) < lines * line_bytes:
            chunk = self.tr.stream_chunk(line_bytes if step > 1 else CHUNK)  # the vendor reads one line per transfer
            if chunk is None:
                break
            stream += chunk
            sizes += 1
            if progress:
                progress(min(1.0, len(stream) / (lines * line_bytes)))
        self.safe_initial_chip()  # StopAutoScan
        self.position += lines * step + 2
        if self.dry_run:
            return Frame([], depth, (gains, offsets), 1 if grey else 3, step)
        if step > 1:
            self.last_stream = bytes(stream)  # the proof stream as read, for working out its line layout
            got = sync_proof_lines(stream, lines)
        elif grey and depth == 16:
            got, slips = repair_slips(stream, lines, line_bytes)
            for line, d in slips:
                self.log(f"byte slip at line {line} repaired (stream re-split {d:+d} bytes on)")
        else:
            got = [bytes(stream[i * line_bytes : (i + 1) * line_bytes]) for i in range(lines)]
        self.log(f"{lines} lines in {sizes} transfers")
        return Frame(got, depth, (gains, offsets), 1 if grey else 3, step)

    def calibrate(self, depth=16, gains=None, offsets=None, warmup=10.0, grey=False):
        """Flat-field references for host-side correction: CAL_LINES motor-driven lines in the lit calibration window
        (open light, as the vendor's shading calibration) and in the dark carriage body, with the scan's own settings.
        About 10 s after the warm-up. Returns a Calibration (None in a dry run)."""
        light = self.scan(CAL_LINES, CAL_OPEN_AT, gains, offsets, warmup=warmup, depth=depth, grey=grey)
        dark = self.scan(CAL_LINES, CAL_DARK_AT, gains, offsets, warmup=0.0, depth=depth, grey=grey)
        if not light.lines or not dark.lines:
            return None
        return Calibration.from_frames(dark, light)

    def _open_light(self, code, warmup=0.0, step=1):
        """The open-light level of a grey scan at front-end gain `code`: the 99.5th percentile of CAL_LINES grey lines in
        the lit calibration window (as many lines as cover the same steps at `step`). None in a dry run."""
        import numpy as np

        depth = 16 if step == 1 else 8
        frame = self.scan(max(9, -(-CAL_LINES // step)), CAL_OPEN_AT, (code,) * 3, None, warmup=warmup, depth=depth,
                          grey=True, step=step)
        if not frame.lines:
            return None
        return float(np.percentile(np.frombuffer(b"".join(frame.lines), "<u2" if depth == 16 else "u1"), 99.5)) * (
            1 if depth == 16 else 257)

    def grey_gain(self, warmup=10.0):
        """The front-end gain code for grey scans: the highest whose open light stays under GREY_TARGET. Reads open light
        at code 0, scales it by GREY_GAIN_STEPS to pick a code, reads again at that code and steps down from the reading
        while it is over (at most 4 readings in all, each about 4 s with the carriage moves). 0 in a dry run."""
        level, code = self._open_light(0, warmup), 0
        if level is None:
            self._open_light(8)  # shows the second reading in the trace
            return 0
        self.log(f"grey open light at gain 0: {level:.0f}")
        for _ in range(3):
            pick = max((c for c in range(16) if level * grey_gain_factor(c) / grey_gain_factor(code) <= GREY_TARGET),
                       default=0)
            if pick == code:
                break
            code, level = pick, self._open_light(pick)
            self.log(f"grey open light at gain {code}: {level:.0f}")
            if level <= GREY_TARGET:
                break
        if level > GREY_TARGET and code > 0:  # the last reading was still over: one more code down, unmeasured
            code -= 1
        return code

    def proof_frame(self, approach, lines=1800 // PROOF_STEP):
        """A 300 dpi grey proof of the frame in the gate at PROOF_GAIN, scanned over the same steps as the 1800-line frame
        scan."""
        return self.scan(lines, approach, (PROOF_GAIN,) * 3, None, 0.0, depth=8, grey=True, step=PROOF_STEP)

    def proof_gain(self, proof):
        """The gain code for the frame `proof` at 1800 dpi: the highest at which the frame's brightest part (the 99.5th
        percentile, which the proof's sideways row jumps do not change) is expected under PROOF_TARGET."""
        import numpy as np

        base = float(np.percentile(np.frombuffer(b"".join(proof.aligned()), "u1"), 99.5)) * 257
        unit = base * PROOF_SCALE / grey_gain_factor(PROOF_GAIN)  # the expected 1800 dpi level per unit of gain factor
        pick = max((c for c in range(16) if unit * grey_gain_factor(c) <= PROOF_TARGET), default=0)
        self.log(f"proof: brightest {base:.0f} at gain {PROOF_GAIN}; gain {pick}, {unit * grey_gain_factor(pick):.0f} "
                 f"expected at 1800 dpi")
        return pick

    def frame_black(self, frame):
        """The black level of a 16-bit grey frame: the median of BLACK_COLUMNS of its lines as read (before any Frame.black
        is taken off). None in a dry run, or for other scans."""
        import numpy as np

        if self.dry_run or not frame.lines or frame.depth != 16 or frame.channels != 1 or frame.step != 1:
            return None
        rows = np.array([np.frombuffer(line, "<u2")[::-1][:WIDTH] for line in frame.lines])
        return float(np.median(rows[:, BLACK_COLUMNS]))

    def warm_lamp(self, least=10.0):
        """Light the lamp and wait until its open light is steady (LAMP_SETTLE), at least `least` seconds. Returns the last
        open_level(), or None in a dry run."""
        self.lamp(True)
        if self.dry_run:
            return None
        t0 = time.time()
        time.sleep(least)
        level = self.open_level()
        while level and time.time() - t0 < LAMP_MOST:
            time.sleep(LAMP_EVERY)
            new = self.open_level()
            self.log(f"lamp warming: open light {new:.0f} after {time.time() - t0:.0f} s")
            if not new or new < level * (1 + LAMP_SETTLE):
                return new
            level = new
        return level

    def open_level(self, channel=GREY_CHANNEL):
        """Open light in the lit calibration window: the OPEN_PERCENTILE of a 16-bit mono scan of CAL_LINES lines at
        CAL_OPEN_AT, gain 0, in `channel`. From the home sensor each time (scan() then takes a new reference): the tracked
        position drifts outward by a few steps with each long trip home, and at 100 steps (roll40) the lines run past the
        lit window, so the reading collapses. None in a dry run."""
        import numpy as np

        if self.position is not None:
            self.seek_home()
        frame = self.scan(CAL_LINES, CAL_OPEN_AT, (0, 0, 0), None, 0.0, depth=16, grey=True, channel=channel)
        self.open_columns = None
        if self.dry_run or not frame.lines:
            return None
        self.open_columns = frame_columns(frame)
        return float(np.percentile(np.frombuffer(b"".join(frame.lines), "<u2"), OPEN_PERCENTILE))

    def black_offset(self, gains, approach, target=BLACK_TARGET):
        """(red offset code, counts per code) that puts the black level of 16-bit red grey scans at `gains` at `target`:
        two short scans (BLACK_LINES) at the BLACK_PROBE codes, a straight line through their black levels. A code whose
        black level reads 0 (clipped) is left out and probe54's 165 counts per code used. None in a dry run."""
        _, (_, g0, b0) = DEFAULT_LEVELS[16]
        levels = {}
        for code in BLACK_PROBE:
            frame = self.scan(BLACK_LINES, approach, gains, (code, g0, b0), 0.0, depth=16, grey=True)
            level = self.frame_black(frame)
            if level is None:
                return None
            levels[code] = level
        (c1, b1), (c2, b2) = levels.items()
        slope = (b2 - b1) / (c1 - c2) if b1 > 0 and b2 > 0 else 165.0
        c, b = (c1, b1) if b1 > 0 else (c2, b2)
        code = int(round(c - (target - b) / slope))
        code = min(max(code, 0), 255)
        self.log(f"black level {b1:.0f} at offset {c1}, {b2:.0f} at {c2} ({slope:.0f} per code): offset {code} for "
                 f"{target}")
        return code, slope

    def align_gain(self, frame, exposure=1.0):
        """The gain code for strip frames at 1800 dpi from an alignment scan of the strip (`frame`, see align_scan()): the
        highest at which the brightest part (the film base, in the gaps) is expected under PROOF_TARGET, as proof_gain()
        but with no extra pass. Open light past the film's end is left out (OPEN_LIGHT: after a rewind the strip's cut end
        lies in the gate, which set roll40's gain to 6 against roll39's 14; film base reads at most 111 here, open light
        161 and up). `exposure` is the frame scans' light against the alignment scan's line (line_exposure()). None when
        most of the scan is open light, or in a dry run."""
        import numpy as np

        if self.dry_run or frame is None or not frame.lines:
            return None
        samples = np.frombuffer(b"".join(frame.aligned()), "u1")
        film = samples[samples < OPEN_LIGHT]
        if film.size < samples.size / 2:
            return None
        level = float(np.percentile(film, 99.5))
        unit = level * 257 * ALIGN_SCALE * exposure / grey_gain_factor(ALIGN_GAIN)  # expected 1800 dpi level per unit of gain
        pick = max((c for c in range(16) if unit * grey_gain_factor(c) <= PROOF_TARGET), default=0)
        self.log(f"alignment: brightest {level:.0f} at gain {ALIGN_GAIN}; gain {pick}, {unit * grey_gain_factor(pick):.0f} "
                 f"expected at 1800 dpi")
        return pick

    # ---- film strips --------------------------------------------------------------------------------------------------

    def film_sensors(self):
        """Status pages 0x0A and 0x0B as the vendor reads them: page A bit 7 and page B bits 7, 6 and 0 see film along the
        path (entrance, entrance, inner, exit side); page B bit 7 clear is the vendor's end of film. None in a dry run."""
        self.set_gpio()
        self.tr.write_regs((0x97, 0x0A), (0xA3, 0x1E))
        a = self.tr.read_reg(0x8B, twice=True)
        self.tr.write_regs((0x97, 0x0B), (0xA4, 0xFF))
        b = self.tr.read_reg(0x8B, twice=True)
        return None if a is None or b is None else (a, b)

    def _feed(self, cmds):
        """One feed unit with the carriage parked over the hole row. EndOfFilm once page B bits 7 and 6 are both clear
        (the strip's tail has passed the inner sensor too; the vendor stops at bit 7, which loses the last frame)."""
        if self.position is None:
            self.reference()
        self.go_to(STRIP_PARK)
        for cmd in cmds:
            self.mcu_command(cmd)
        sensors = self.film_sensors()
        if sensors is None:
            return
        film = (sensors[0] & 0x80, sensors[1] & 0x80, sensors[1] & 0x40, sensors[1] & 0x01)
        if film != getattr(self, "_film_seen", film):
            self.log("film sensors A7 B7 B6 B0: " + " ".join("1" if f else "0" for f in film))
        self._film_seen = film
        if not sensors[1] & 0xC0:
            raise EndOfFilm("page B bits 7 and 6 cleared: the end of the strip has passed the inner sensor")

    def _hole_read(self, warmup=0.0):
        """The uncut sprocket holes in one stationary line at STRIP_PARK (about 1.4 s, as the vendor reads them); None in
        a dry run."""
        self.go_to(STRIP_PARK)
        self._setup_stationary()
        if warmup and not self.dry_run:
            time.sleep(warmup)
        lines = [self._read_stationary(n > 0) for n in range(2)]  # the first line after a setup is the vendor's discard
        self.safe_initial_chip()
        self._rest_holes = None if lines[1] is None else whole_holes(line_red(lines[1]))
        return self._rest_holes

    def _measure(self, last, expect, what):
        """A hole read and the travel since `last` (expected about `expect`); the nominal travel when no hole shows."""
        now = self._hole_read()
        d = hole_travel(last, now, expect)
        if d is None:
            self.log(f"{what}: no sprocket hole in the line; assuming the nominal {-expect:.0f} px")
            d = float(expect)
        else:
            self.log(f"{what}: {-d:.1f} px (holes {[round(h, 1) for h in now]})")
        return now, d  # with no hole now, the next read is measured nominally too

    def feed_forward(self, px, tolerance=FEED_TOLERANCE, warmup=0.0, over_ok=None, short_ok=None):
        """Feed the strip `px` pixels forward, closed loop, with the carriage parked over the sprocket holes: coarse units
        (command 2 + 7), two at a time with a hole read after each pair (one when only one fits), while more than
        COARSE_LEFT remain, then fine units (command 1) in up to FINE_ROUNDS rounds of FINE_MAX to within `tolerance`.
        Never feeds back, so it can overshoot by up to one fine round's error. Returns the measured travel (None in a dry run).

        Fine units stall with the holes in one band of the pitch, and only a coarse unit is sure to pass it. Then, when
        `over_ok` and `short_ok` are given (how far the feed may overshoot or fall short before something is lost), one more
        coarse unit is fed if it loses less than stopping short does; otherwise the feed stops where it is."""
        last = self._rest_holes or self._hole_read(warmup)  # scans do not move the film, so the read before them still holds
        if self.dry_run:
            self._feed((2, 7))
            self._feed((1,))
            self._hole_read()
            return None
        moved, units = 0.0, 0
        while px - moved > COARSE_LEFT:
            k = 2 if px - moved - COARSE_PX > COARSE_LEFT else 1
            units += k
            if units > 12:
                raise ScannerError(f"12 coarse units moved the film only {moved:.0f} of {px:.0f} px")
            for _ in range(k):
                self._feed((2, 7))
            last, d = self._measure(last, -k * COARSE_PX, f"{k} coarse unit(s)")
            moved -= d
        for _ in range(FINE_ROUNDS):
            remaining = px - moved
            if remaining <= tolerance:
                break
            n = max(1, min(FINE_MAX, round(remaining / FINE_PX)))
            for _ in range(n):
                self._feed((1,))
            last, d = self._measure(last, -n * FINE_PX, f"{n} fine units")
            moved -= d
            if -d < FINE_STALL * n * FINE_PX:
                remaining = px - moved
                self.log(f"fine units stalled ({-d:.0f} px for {n} units), {remaining:.0f} px short")
                if over_ok is not None and short_ok is not None:
                    lost_short = max(0.0, remaining - short_ok)
                    lost_over = max(0.0, COARSE_PX - remaining - over_ok)
                    if lost_over < lost_short:
                        self._feed((2, 7))
                        last, d = self._measure(last, -COARSE_PX, "one coarse unit instead")
                        moved -= d
                break
        self.log(f"fed {moved:.0f} of {px:.0f} px")
        return moved

    def align_scan(self, warmup=0.0, grey=ALIGN_GREY, lines=ALIGN_SCAN[0]):
        """The scan the frame alignment looks at (ALIGN_SCAN, or its first `lines`): 8-bit colour, or 8-bit grey of
        ALIGN_CHANNEL at ALIGN_GAIN."""
        if grey:
            return self.scan(lines, ALIGN_SCAN[1], (ALIGN_GAIN,) * 3, warmup=warmup, depth=8, grey=True,
                             channel=ALIGN_CHANNEL)
        return self.scan(lines, ALIGN_SCAN[1], warmup=warmup)

    def align_frame(self, edge_at=EDGE_AT, at_least=None, warmup=0.0, short=False):
        """Find the first frame edge (the right end of an inter-frame gap) at or right of `at_least` (default `edge_at`)
        in a scan of the picture rows and feed it to CCD pixel `edge_at`. Returns the edge found (None in a dry run);
        EndOfFilm when there is none (the end of the strip), with that scan kept in `self.failed_align` for a look. The
        alignment scan is kept in `self.last_align` either way. With `short` the scan is first only ALIGN_SHORT lines long
        (see there) and redone in full when it finds no plausible edge."""
        self.failed_align = None
        near = edge_at if at_least is None else at_least
        frame = self.last_align = self.align_scan(warmup, lines=ALIGN_SHORT if short and not self.dry_run else ALIGN_SCAN[0])
        if self.dry_run:
            self.feed_forward(1000)
            return None
        edge = frame_edge(align_plane(frame), near)
        if short and edge is None:
            self.log("short alignment scan found no edge; scanning in full")
            frame = self.last_align = self.align_scan(0.0)
            edge = frame_edge(align_plane(frame), near)
        if edge is None:
            self.failed_align = frame
            raise EndOfFilm("no frame edge in the gate")
        if -OVER_OK <= edge < 0:
            self.log(f"frame edge found at {edge}: the frame is already in the gate, its first {-edge} px are lost")
            return edge
        at = self.place_edge(edge, edge_at)
        self.log(f"frame edge found at {edge}, now at {'?' if at is None else at}")
        return edge

    def place_edge(self, edge, edge_at=EDGE_AT):
        """Feed a frame edge now at CCD pixel `edge` to `edge_at`, each feed checked by a short alignment scan and repeated
        while the frame would not fit the gate (ALIGN_FEEDS). The frame fits while its left edge stays within 0 ..
        GATE_END - FRAME_PX; an edge already left of `edge_at` but no more than OVER_OK before the gate start is left where
        it is. An edge further out (fed too far: a coarse unit instead of stalled fine units, or the advance) is backed off
        (_back_off(), up to BACK_TIMES times) and fed forward again, but only when the hole reads say so too: a check scan
        can be fooled, the holes measure the film (probe52/53: to 1 to 3 px). Where a check scan cannot tell, the holes'
        measure stands. Returns where the edge ends up, None when neither can tell (a dry run)."""
        at, holes, fits, backed = edge, edge, GATE_END - FRAME_PX - edge_at, 0
        for n in range(ALIGN_FEEDS + BACK_TIMES):
            if at is None:
                break
            if at < -OVER_OK and holes < -OVER_OK and backed < BACK_TIMES and not self.dry_run:
                at = holes = self._back_off(holes)
                backed += 1
            elif at - edge_at <= (FEED_TOLERANCE if n == 0 else fits):
                break
            # while a back-off is left, an overshoot can be undone and stopping short cannot (the frame-scan rescan stalls
            # the same way): so when fine units stall, take the coarse unit whatever it overshoots
            over_ok = edge_at + COARSE_PX if backed < BACK_TIMES else edge_at
            moved = self.feed_forward(at - edge_at, over_ok=over_ok, short_ok=fits)
            holes = at if moved is None else round(at - moved)
            seen = self._check_edge(at)
            if seen is not None and moved is not None and abs(seen - holes) > CHECK_AGREE:
                self.log(f"check scan ({seen}) and hole reads ({holes}) disagree; going by the holes")
                seen = None
            at = seen if seen is not None else (None if moved is None else holes)
        return at

    def _back_off(self, edge):
        """Bring a frame edge at CCD pixel `edge`, before the gate start, back into the gate: coarse units back until it
        is BACK_TO in (at most BACK_MAX), then one coarse unit forward, which takes up the reversal slack and lets fine units
        move again. Every unit is measured on the holes. Returns the edge's new position."""
        self.log(f"frame edge at {edge:.0f}, fed too far: backing it off")
        last = self._hole_read()
        for _ in range(BACK_MAX):
            if edge >= BACK_TO:
                break
            self._feed((6, 7))
            last, d = self._measure(last, BACK_PX, "one coarse unit back")
            edge += d
        self._feed((2, 7))
        last, d = self._measure(last, -AFTER_BACK_PX, "one coarse unit forward after the back-off")
        edge += d
        self.log(f"frame edge now at about {edge:.0f}")
        return edge

    def early_frame(self, frame):
        """The frame's (negative) left edge when a strip frame scan shows it fed too far (early_edge()); None when it does
        not, or in a dry run."""
        return None if self.dry_run or not frame.lines else early_edge(align_plane(frame))

    def late_frame(self, frame):
        """The frame's left edge when a strip frame scan shows it lying too far in for the gate to hold it whole (late_edge());
        None when it is placed well enough, or in a dry run."""
        return None if self.dry_run or not frame.lines else late_edge(align_plane(frame))

    def _check_edge(self, before):
        """Where the frame edge lies after a feed that was to bring it from `before` to EDGE_AT: a short alignment scan
        (ALIGN_SHORT lines). None when that scan shows no edge between CUT_OK before the gate and `before` (the film cannot
        have moved back), so the feed's own measure stands."""
        self.last_check = self.align_scan(lines=ALIGN_SHORT)
        edge = frame_edge(align_plane(self.last_check), 0)
        if edge is None or not -CUT_OK <= edge <= before + FEED_TOLERANCE:
            self.log(f"check scan: no edge between {-CUT_OK} and {before} (found {edge}); the feed's measure stands")
            return None
        self.log(f"check scan: edge at {edge}")
        return edge

    def predict_edge(self, frame, pitch_mm=38.0):
        """Where the next frame's left edge lies now, from the frame just scanned: its own left edge (left_edge()) plus the
        frame pitch. On saved rolls this is within 30 px of the edge the alignment scan found for about 9 frames in 10
        (median pitch 2729 to 2733 px on six rolls, 38 mm is 2728). None when the scan shows no left edge near the gate start,
        when the gap after the frame (where it shows) disagrees by more than PREDICT_NEXT, or in a dry run."""
        if self.dry_run or not frame.lines:
            return None
        r = align_plane(frame)
        left = left_edge(r)
        if left is None or not 0 < left <= PREDICT_LEFT:
            return None
        edge = int(round(left + pitch_mm * PX_PER_MM))
        seen = next_edge(r)
        if seen is not None and abs(seen - edge) > PREDICT_NEXT:
            self.log(f"the next frame's gap in the last scan ({seen}) and the pitch ({edge}) disagree")
            return None
        return edge

    def _advance_predicted(self, edge):
        """Feed the next frame's predicted edge (predict_edge()) to EDGE_AT, closed loop on the holes, and check it with one
        short scan instead of an alignment scan. Returns the edge as the scan sees it, or None when the scan sees none or
        differs from the hole reads' measure by more than CHECK_AGREE: then neither is believed and the caller aligns."""
        self.log(f"the next edge is expected at {edge}: feeding {edge - PREDICT_AT} px")
        # never a coarse unit past the aim (an overshoot of the gate start is not seen by the check scan, and the alignment
        # then takes the next frame's gap): a stalled feed stops short, and place_edge() finishes it
        moved = self.feed_forward(edge - PREDICT_AT, over_ok=PREDICT_AT, short_ok=GATE_END - FRAME_PX - PREDICT_AT)
        holes = edge if moved is None else round(edge - moved)
        seen = self._check_edge(edge)
        if seen is None or abs(seen - holes) > CHECK_AGREE:
            self.log(f"the check scan ({seen}) and the hole reads ({holes}) do not agree; aligning")
            return None
        self.last_align = self.last_check
        at = self.place_edge(seen)  # leaves an edge near EDGE_AT (or early within OVER_OK) where it is
        self.log(f"frame edge at {seen}, now at {'?' if at is None else at}, after the predicted advance")
        return at

    def advance_frame(self, pitch_mm=38.0, margin=100, upto=None, frame=None):
        """The next frame: with the `frame` just scanned, feed to the edge its pitch predicts (predict_edge()) and check it
        with a short scan; where that is not possible or the check disagrees, feed one frame pitch minus `margin` pixels (so
        an overshoot cannot skip the gap), or less when the last frame scan showed the next frame's edge already at CCD
        pixel `upto` (next_frame_edge()), then align."""
        self.failed_align = None
        edge = None if frame is None else self.predict_edge(frame, pitch_mm)
        if edge is not None:
            at = self._advance_predicted(edge)
            if at is not None:
                return at
            return self.align_frame(at_least=0)  # the film has moved the pitch already
        px = pitch_mm * PX_PER_MM - margin
        if upto is not None and upto - EDGE_AT - margin < px:
            px = upto - EDGE_AT - margin
            self.log(f"the next frame starts at {upto} in the last scan: feeding {px:.0f} px, not a whole pitch")
        if px > FEED_TOLERANCE:
            self.feed_forward(px)
        return self.align_frame(at_least=0)

    def next_frame_edge(self, frame):
        """The next frame's edge as a strip frame scan shows it (next_edge()); None when it does not, or in a dry run."""
        return None if self.dry_run or not frame.lines else next_edge(align_plane(frame))

    def eject(self, bursts=8):
        """The vendor's eject (opcode 8): bursts of 8 coarse units forward while page B bit 6 sees film, then 8 more; the
        strip leaves through the exit slot. The carriage goes home first, as the vendor does. Returns the film sensors
        afterwards."""
        if self.position is not None:
            self.safe_initial_chip()
            self.seek_home()

        def burst():
            for _ in range(8):
                self.mcu_command(2)
                self.mcu_command(7)
            return self.film_sensors()

        sensors = self.film_sensors()
        for _ in range(bursts):
            if sensors is not None and not sensors[1] & 0x40:
                break
            sensors = burst()
            if sensors is None:
                break
        else:
            raise ScannerError(f"page B bit 6 still sees film after {bursts} bursts; the strip may be stuck")
        return burst()

    def rewind(self, max_units=REWIND_MAX):
        """Feed a caught strip back to where the feeder's auto-load holds it, to scan it again: coarse units back (6 + 7)
        one at a time until page B bit 6 has seen film and cleared behind the strip's leading end (the vendor's "back to
        start"), then REWIND_PAST more, as bit 6 clears short of where the auto-load holds a strip. With bit 6 clear and
        bit 7 set the leading end has not reached bit 6 yet (it does about 4 units past the load position), so the strip
        is at its start, or at most a few units in, and nothing moves; going back regardless pushed it out of the
        entrance (80 units). Refuses with no film on page B bits 7, 6 and 0; stops with an error if bit 6 sees no
        film within REWIND_FIND units, if the entrance sensor (bit 7) clears while going back, or after `max_units`.
        Returns the number of units fed back; 0 in a dry run."""
        sensors = self.film_sensors()
        if sensors is None:
            return 0
        if not sensors[1] & 0xC1:
            raise ScannerError("rewind: page B bits 7, 6 and 0 see no film; there is no caught strip to feed back")
        if not sensors[1] & 0xC0:
            seen = False  # the tail has passed bit 6 too (bit 0 still sees it): going back brings it over bit 6 first
        elif not sensors[1] & 0x40:
            self.log("rewind: page B bit 6 is clear with film at the entrance; the strip is at its start already")
            return 0
        else:
            seen = True
        if self.position is not None:
            self.safe_initial_chip()
            self.seek_home()
        entered = bool(sensors[1] & 0x80)
        for n in range(1, max_units + 1):
            self.mcu_command(6)
            self.mcu_command(7)
            b = self.film_sensors()[1]
            if b & 0x40:
                seen = True
            elif not seen and n >= REWIND_FIND:
                raise ScannerError(f"rewind: page B bit 6 saw no film within {n} units back; stopped")
            elif seen:
                for _ in range(REWIND_PAST):
                    self.mcu_command(6)
                    self.mcu_command(7)
                self.log(f"rewind: page B bit 6 cleared after {n} coarse units back, then {REWIND_PAST} more: at the start")
                return n + REWIND_PAST
            if b & 0x80:
                entered = True
            elif entered:
                raise ScannerError(f"rewind: the entrance sensor cleared after {n} units back before bit 6 did; stopped")
        raise ScannerError(f"rewind: page B bit 6 did not clear within {max_units} units back; stopped")
