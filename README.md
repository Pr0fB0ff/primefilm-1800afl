# PrimeFilm 1800 AFL driver

A userspace driver for the Pacific Image PrimeFilm 1800 AFL (USB `2770:0119`), a 1800 dpi 35mm film scanner with an automatic
strip feeder. The vendor's software no longer runs on current systems, and no [SANE](http://www.sane-project.org/) backend supports
this scanner.

It scans mounted slides and film strips, and it feeds a strip through frame by frame, autofocusing and placing each frame from the
sprocket holes and the gaps between frames. A whole uncut 36-exposure roll has been run through it.

- [`docs/protocol.md`](docs/protocol.md): the USB protocol, written so a driver can be built from it in any language.
- `pf1800.py`: the driver, in Python.
- `pf1800_scan.py`: a command-line scanner on top of it.

## Status

Experimental. It has been run on **one unit** (firmware version 4), on macOS. Black-and-white negatives are what it has mostly been
used for. Colour scanning works at default levels, but there is no gain for colour film and no handling of the orange mask.
Feed units differ between scanners, so strip placement may need tuning on yours. If you have this scanner or one of the related
USB IDs listed in the protocol document, a report of what happened would be very welcome.

The scanner has a hard stop beyond its home position. Review a new sequence with `--dry-run` before running it. If a strip gets
stuck, a power cycle backs it out.

## Install

You need Python 3 and libusb (`brew install libusb` on macOS).

```sh
pip install git+https://github.com/Pr0fB0ff/primefilm-1800afl@v0.1.0
```

To work on the driver, clone the repository and run `pip install -e .` inside it.

On Linux you will probably also need a udev rule to reach the device without root. This has not been tried.

## Use

```sh
pf1800-scan out/slide --dry-run      # print every USB transfer, move nothing
pf1800-scan out/slide --autofocus    # scan a mounted slide
pf1800-scan out/roll --strip --autofocus  # scan a strip the feeder has caught, then eject it
pf1800-scan --help
```

Push a strip into the entrance and the feeder takes it by itself. Cut off an uncut roll's fogged leader first. Keep the film
entering the top slot apart from the film leaving the bottom slot. Frames are saved as negatives, in 16-bit TIFF files.

Comments in the code name test runs (`roll36b`, `probe49`). They are labels from the development log, which is not published.

## Licence

GPL-2.0-or-later, see [LICENSE](LICENSE). The protocol was worked out for interoperability; this repository contains no vendor code.
Not affiliated with or endorsed by Pacific Image Electronics.
