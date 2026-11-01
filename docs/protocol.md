# Pacific Image PrimeFilm 1800 AFL: USB protocol

A description of how to drive the PrimeFilm 1800 AFL (USB `2770:0119`), a 1800 dpi 35mm film scanner with an automatic strip
feeder, a manual slide slot and autofocus. It is meant to be enough to write a driver in any language.

The protocol was worked out by studying the vendor's Windows driver and then testing each step on one unit. This document
describes the hardware's behaviour. It contains no vendor code. [`pf1800.py`](../pf1800.py) is a working Python implementation of everything
here, and running it with `dry_run=True` prints every USB transfer without moving anything.

**How far to trust it.** Everything below was run on one unit (firmware version 4), on macOS through libusb, unless marked:

- **[vendor]**: the value is the vendor's and works, but what it means is inferred or unknown.
- **[not tried]**: read from the vendor's driver and not tested on the hardware.
- Measured numbers (steps, pixels, levels, line rates) are from this one unit. Expect others to differ, the feed units most.

Bit numbers count from 0 (bit 7 = 0x80). All numbers are hexadecimal when written `0x..`.

**Safety.** The carriage has a hard stop beyond home, so bound every move and every wait. A power cycle recovers from almost anything, including a strip stuck in the feeder (see [Strip transport](#strip-transport)).

## Hardware

| | |
|---|---|
| USB | 1.1 full speed, `bcdDevice` 1.00, product string "AFL Film Scanner", vendor class |
| Endpoints | bulk IN 0x81, bulk OUT 0x02 (64 bytes), interrupt IN 0x83 (1 byte, not needed) |
| Sensor | tri-linear colour CCD, 2752 pixels read per line, cold-cathode lamp |
| Resolution | 1800 dpi optical (1 pixel = 1 motor step = about 14.1 µm); 300 dpi proof mode |
| Samples | 12 significant bits (the low 4 bits of 16-bit samples are 0; ceiling 65520) |
| Controller | "SQ" scanner ASIC behind a parallel-port-style interface carried over USB |
| Front end | looks like a Wolfson WM8196/WM8199 by its register map [vendor] |
| Feed and lens | a separate microcontroller (MCU), reached through the ASIC's GPIO pins |

The vendor's INF for this model also lists `2770:011a`, `2770:011b`, `05da:00cd` and `1606:f000`. None has been tested.

Two separate mechanisms move things. The **carriage** motor is driven from ASIC registers and carries the CCD optics past the
gate, which gives the scan axis. The **feeder** moves a film strip along the CCD line, at right angles to the scan axis, and
belongs to the MCU, as does the lens. So on a strip the frame's 36 mm side lies along the CCD line and its 24 mm side along the
carriage travel.

## Transport

Everything is a vendor control transfer, plus bulk transfers for tables and image data.

**Control OUT**: `bmRequestType 0x40`, `bRequest 0x04` when the data is longer than 1 byte, otherwise `0x0C`. `wValue` selects
the operation:

| `wValue` | `wIndex` | data | meaning |
|---|---|---|---|
| 0x0090 | 0 | 1 byte | parallel port data (used for the open and close handshakes) |
| 0x00B0 | 0 | pairs `reg, val, reg, val, ...` | write ASIC registers, in order |
| 0x00B0 | 0 | `reg` | select a register for reading |
| 0x00C0 | 0x8406, then 0x0406 | `00` | clean the buffer: always this pair, in this order |
| 0x0001 | 0 | `01` | start streaming image data |
| 0x0002 | byte count N | `02` | N bytes follow on bulk OUT |
| 0x0003 | byte count N | `00` | N bytes follow on bulk IN |

`pf1800.py` pauses 5 ms after each control OUT. That value has never been tuned.

**Read a register**: select it, then control IN `bmRequestType 0xC0, bRequest 0x0C, wValue 0x0007, wIndex 0`, 1 byte.

**Status pages** are read through register 0x8B: write `0x97 = page`, then read 0x8B. Pages 0x0A and 0x0B need an extra GPIO
write first and the read done twice, the second value being the answer:

```
page 0     97=00                 read 8B
page 0x0A  97=0A A3=1E           read 8B twice
page 0x0B  97=0B A4=FF           read 8B twice      (write the GPIO setup below first)
```

**Small bulk read of N bytes**: control OUT `wValue 3, wIndex N`, then bulk IN. Read at least 64 bytes (the endpoint's packet
size) and keep N of them, or libusb reports an overflow.

**Bulk write of N bytes** (the pixel table):

```
OUT wValue C0 wIndex 0406 data 00
OUT wValue B0             data 8F      (select 0x8F)
OUT wValue C0 wIndex 8406 data 8F
OUT wValue C0 wIndex 0406 data 8F
OUT wValue 02 wIndex N    data 02
bulk OUT N bytes
```

**Image stream**: at the first read of each scan session, clean the buffer and send `wValue 1`, then read bulk IN until the
data is in. Any size that is a multiple of 64 and longer than a line works (`pf1800.py` uses 8192), except for proof scans
(below). Each new scan session must send the start again.

## Session

**Open**: claim interface 0, write these four bytes one at a time through `wValue 0x0090`, then do the chip reset.

```
open handshake   64 65 44 45
close handshake  64 65 16 17
```

**Chip reset** (the vendor's `SafeInitialChip`, also how a scan is stopped):

```
90=00 80=00 84=00 BE=00 C0=00 86=00 89=00 94=00 01=02    then clean the buffer
```

Skipping the reset after the open makes the first MCU command after a power cycle time out.

**Close**: stop the motor, chip reset, lens home if the lens moved, carriage home, lamp off, close handshake. The handshake does
not turn the lamp off.

**GPIO setup**, needed before motor moves, film sensor reads and the MCU:

```
83=00 A3=02 A4=1E A1=FF A2=F1
```

**Drive capability**: `CE=00 CF=0C`. This matters more than it looks: with `CF=04` every line is one flat whole-width signal and
the scanner appears to work but produces no picture.

**Lamp**: on and off differ in the last write. `0x8D` is written through two banks of register 0x83.

```
A8=80 83=20 8D=FA 83=00 8D=AA 93=06    on
A8=80 83=20 8D=FA 83=00 8D=AA 93=05    off
```

`93=00` does not turn it off. The lamp rises about 60 % over its first 4 to 5 minutes, then holds within 1 % at idle. A fixed
10 s warm-up gives a picture, but its level is still rising.

## Page 0 status

| bit | meaning |
|---|---|
| 7 | a second position sensor the vendor seeks for strips [vendor]; `pf1800.py` does not need it |
| 6 | carriage in the home zone |
| 5, 4 | scan state flags: bit 4 set after a scan starts, bit 5 when lines have been read. They survive a session; ignore them |
| 1 | checked set after each stage of scan setup; a clear bit there is an error [vendor] |
| 1, 0 | both set: the motor has finished a move |

Typical values: 0x57 just after power-up, 0x4F and 0x6F later.

## Carriage

Positions below are motor steps counted from a reference edge, which is where the home zone ends going away from home.

**Move**: chip reset, GPIO setup, drive capability, then:

```
C0=00  80=E1 (away from home) or C1 (toward home)  84=B1  C8=00  88=speed  89=steps low  86=steps high
```

Wait until page 0 `& 3 == 3`, then stop with the same writes using `80=C1`, steps 0. Speed 0xDC gives about 390 steps/s and 0xE4
about 490. 0xE8 also worked, 0xEC lost steps once and 0xF0 stalled. Keep to 0xDC going away from home, where a stall would lose
the position.

**Home**: a move toward home of up to 4000 steps, polling until page 0 bit 6 is set. Only do this from past the home zone.

**Reference**: from inside the home zone, a 200-step move away from home, polling until bit 6 clears (about 85 steps), then stop.
That point is position 0.

**Backlash**: reversing leaves the carriage about 45 steps further out than commanded, so reach every position moving away from
home: move toward home 100 steps too far, then come back out.

| position (steps) | what is there |
|---|---|
| 0 to 130 | lit calibration window: open light, falling to dark by 180 |
| 275 to 1550 | carriage body, dark. A dark reference can be taken at 700 |
| 1420 | over the middle of the sprocket hole row (strips) |
| 1555 to 3308 | the open film gate |
| 1595 to 3252 | a mounted slide's opening (CCD pixels 178 to 2720) |
| 1550 to 3260 | a strip frame |
| 3500 | furthest position tried |

## Scanning

### Setup

After a scan the carriage has moved `lines * step + 2` steps, and it must be sent to the start of the next one. The sequence, for
1800 lines of 8-bit colour, with `lamp on` and the GPIO and drive writes as above:

```
chip reset
lamp on; wait 0.5 s; check page 0 bit 1
GPIO setup
drive capability
83=22 87=00 83=02 87=0B                                  CCD pixel number 0xB00 (0xAC0 + 0x40)
83=00 A0=03 9C=C0 9D=40 9E=00 9F=30 83=20 A5=00 A6=00 A7=00    CCD timing
80=00 80=01 04=80 04=00 00=00 01=03 03=12 02=04 05=00    front end setup
28=00 29=0F 2A=0F                                        gains R, G, B
20=C8 21=D0 22=D0                                        offsets R, G, B
24=00 25=00 26=00
B0=00 B1=10 B2=1F B3=FF B4=FF B5=1F                      pixel table: see below
94=41 B0=00 B1=40 B2=1F B3=FF B4=FF B5=1F; bulk write 0x4000 bytes
94=41 B0=00 B1=80 B2=1F B3=FF B4=FF B5=1F; bulk write 0x4000 bytes
94=41 B0=00 B1=C0 B2=1F B3=FF B4=FF B5=1F; bulk write 0x4000 bytes
94=41 B0=00 B1=00 B2=00 B3=FF B4=0F B5=07                image buffer
B6=01 B7=00 B8=end low B9=end high
check page 0 bit 1
C0=00 84=00 84=B1 80=E1                                  motor on
83=00 85=00 83=20 85=01 BC=00 BD=00                      motor steps per line = 1
88=CB C1=00 C2=00 C3=01 C6=CB C7=00 C8=04                line period
C0=C0 89=count low 86=count high                         count = lines * steps per line + 2
check page 0 bit 1
BB=03 9B=14 8E=00 8B=00 BF=00 90=00 91=00 83=82
clean the buffer
small bulk read of 1 byte
BE=0D                                                    start
```

Then start the stream and read. The carriage starts moving by itself. When enough data is in, chip reset stops the scan.

`end` is `(0x70FFE - line_bytes) >> 5`, 16 bits.

The pixel table is 0xC000 bytes of little-endian 16-bit words, sent as three 0x4000 byte chunks to RAM addresses 0x1F4000,
0x1F8000 and 0x1FC000. Bit 13 of a word marks a CCD pixel as read, so `table[2 * i + 1] |= 0x20` for `i` from 1 to 2752 in the
first chunk. All other bytes are zero. The vendor reads only 2586 pixels, which cuts a slide's far edge.

### Modes

Only these writes differ from the sequence above:

| mode | 94 (buffer) | 01 (front end) | 03 (front end) | 85 | 88, C6 | C8 | BB | 90 | BE | line bytes | lines/s |
|---|---|---|---|---|---|---|---|---|---|---|---|
| 8-bit colour | 41 | 03 | 12 | 01 | CB | 04 | 03 | 00 | 0D | 8256 | 74 |
| 16-bit colour | 61 | 03 | 12 | 01 | CB | 05 | 04 | 40 | 0D | 16512 | 48 |
| 8-bit grey | 41 | 07 | 12 + channel | 01 | CB | 04 | 03 | 80 | 0F | 2752 | 148 |
| 16-bit grey | 41 | 07 | 12 + channel | 01 | CB | 06 | 05 | C0 | 0D | 5504 | 98 |
| 300 dpi proof, 8-bit grey | 41 | 07 | 12 + channel | 06 | E7 | 02 | 01 | 80 | 0F | 448 + (below) | |

- "Channel" in front end register 3 is `channel << 6`: 0x12 red, 0x52 green, 0x92 blue. Also write it to `BF` (0, 1 or 2).
- `C8` is always `BB + 1`. A line takes `1 + BB` base periods.
- Grey needs both halves: the front end in mono mode (`01=07`) makes it convert one colour, and bit 7 of 0x90 makes the
  controller send one sample per pixel. Mono reads about a third of the level of colour at the same gain, so grey needs more
  gain. 16-bit grey is not something the vendor's software ever uses.
- 16-bit samples are little-endian. Register 0x90 must stay 0 for 8-bit colour: `0x20` there makes the chip send a `4B 2D AA` fill
  pattern, and it also enables a look-up table that is not uploaded.

**Bus limit.** In colour the controller holds the line rate to what USB can take. In 16-bit grey it does not, and the data
turns to noise part way down a frame if it falls behind: `BB=3` (148 lines/s) always overruns, `BB=4` (118 lines/s) overran once in
seven frames, `BB=5` is safe.

**Byte slips.** Even at `BB=5`, now and then the controller inserts or drops a few bytes in 16-bit grey. An odd number turns every
later sample into noise. A slip shows as a clean line followed by lines that no longer correlate with their neighbours; re-split
the rest of the stream at the offset between -3 and +3 bytes that makes the lines correlate again (`repair_slips()` in `pf1800.py`).

### Line format

- 8-bit colour: `R G B` per pixel. 16-bit colour: the same with 2 bytes per sample. Grey: one sample per pixel.
- **Lines run right to left**: reverse each line for an upright picture.
- CCD pixel 0 carries a counter, not image data: drop the first 2 pixels, leaving 2750.
- In colour the three CCD rows are 4 lines apart: G for a picture line comes from line `n + 4`, B from `n + 8`, so 8 lines are
  lost. Measured G 3.89 and B 7.80 lines, so the whole numbers are right to about 0.2 line. Grey lines need no such step.
- A slide at 1800 lines from 1550 fits with a little dark mount on all four sides.

### 300 dpi proof

`A5 A6 A7` become `00 00 02` for a strip or `03 03 03` for a slide. The table marks every 6th pixel: 448 samples. The count is
`lines * 6 + 2`. Lines must be read one per bulk transfer: 8192-byte chunks scramble them. Each line is 448 samples plus 0 to 30
extra at its start, and ends with a marker of 11 zero samples, so take the samples before each marker (`sync_proof_lines()`).
16-bit mono at 300 dpi gave noise.

### Front end levels

Gains are 4-bit codes (`28 29 2A`) and offsets 8-bit (`20 21 22`). A lower offset code raises the black level.

| | gains R, G, B | offsets R, G, B |
|---|---|---|
| 8-bit colour | 0, 15, 15 | 200, 208, 208 |
| 16-bit colour | 0, 0, 0 | 200, 208, 200 |

Colour film needs its own gains: these are chosen for black-and-white film and for slides.

Relative level by gain code, 16-bit mono: 0: 1.0, 2: 2.14, 4: 2.9, 8: 4.5, 15: 6.3. At gain 15 the offset moves the black level about
165 counts per code, and at the fixed offsets the black level sat at or below 0 and drifted down along a roll. Choosing the
offset for a black level around 6000 and subtracting it from the frame stops the negative's densest parts from reading 0.

### Reading without the motor

Used for autofocus and sprocket holes. The same setup in 8-bit colour, with these changes: the motor is off (`84=00 80=01` in
place of `C0=00 84=00 84=B1 80=E1`), the count is 3, and `C3=00` is written after the count. Read 8256 bytes for one line. The first
line after a setup is to be thrown away.

To read another line without the whole setup:

```
BE=0C
gains and offsets as in the setup
B0=00 B1=10 B2=1F B3=FF B4=FF B5=1F B0=00 B1=00 B2=00
small bulk read of 1 byte
BE=0D
start the stream again
```

## MCU commands

The feed and the lens belong to a microcontroller, which takes a 3-bit code over the GPIO pins. The GPIO setup has to be in
place. To send code `c`, with `v = c << 2`:

```
83=00 A3=1E A1=v                     (one control transfer)
A1=v|2
A1=v
poll page 0x0A until bit 0 is 0      keep this first reading: it is the reply
83=00 A3=1E A1=v|2
poll page 0x0A until bit 0 is 1
```

Each poll is the page 0x0A read, `97=0A A3=1E`, then 0x8B twice. The reply code is bits 4 to 2 of the kept reading: 5 is busy
(send the command again; a long lens move keeps it busy for a while), 2 is "go on" (for command 0, send the sub-command as a second
code).

| code | action |
|---|---|
| 0, then sub | lens: sub 0 = +1, sub 1 = +5, sub 2 = +10 units from home, sub 6 = home, sub 5 = one step, used between lines of a sweep [vendor] |
| 1 / 5 | fine feed forward / back |
| 2 / 6 | coarse feed forward / back |
| 3 | stops a feed that found nothing, after the vendor's load [not tried] |
| 4 | firmware version (below) |
| 7 | finishes a coarse unit, and starts the load [vendor] |

**Firmware version**: command 4 sent twice. The first time is a normal command. The second writes `83=00` after the strobe, and the
version is bits 4 to 2 of the reading that follows. The vendor's feed arithmetic differs below
version 3 (fine units half the size) [not tried].

**The first MCU command** of a session sometimes goes unanswered, even for 30 s, after a power cycle, a reconnect or a new strip. Closing the
session, waiting 5 s, opening again and retrying has always worked.

## Film sensors

Read after the GPIO setup, each page twice as above.

| reading | meaning |
|---|---|
| A bit 7 | film at the entrance |
| A bit 6 | lens flag: drops at about +60 to +70 units |
| A bit 5 | **clear** = a mounted slide is in |
| A bits 4 to 2, bit 0 | MCU reply and ready |
| B bit 7 | film at the entrance. Set or bit 6 set means a strip |
| B bit 6 | film at the inner sensor |
| B bit 0 | film on the exit side |
| B bits 3 to 1 | toggle between feed units with the carriage parked at 1420; meaning unknown |

Typical readings, A then B: `7F 10` empty, `5F 10` slide, `FF 90` strip just loaded.

## Autofocus

The lens moves in units from its home position, and the vendor's default is home. Autofocus sends the lens home, then reads stationary lines
at a series of lens positions, keeping the sharpest (`pf1800.py` uses normalised gradient energy of the red plane).

A slide focuses at about +40. A strip usually focuses at about +0 to +1, though one roll was still getting sharper at +15, so sweep on while the last position is the sharpest. The carriage should stand in the middle of a frame, at a place
with picture detail. Red and green focus several units apart, so focus on the colour you scan. Send the lens home again before the session
ends.

## Strip transport

### Loading and moving

**The feeder takes a strip by itself**, with no command and even with USB unplugged, as soon as the entrance sensors see film. After that it cannot
be pulled out by hand. **A power cycle backs out a strip** that is still at the entrance.

The film runs along the CCD line, and feeding **forward moves the picture toward CCD pixel 0**. Feed with the carriage parked at 1420.

- Coarse unit: command 2 then **always command 7**. Without 7, the fine units stall later. About 312 to 340 px, varying.
- Fine unit: command 1. About 15 px (the vendor's nominal figure is 17.7), 15 to 22 in practice.
- 341 px is one perforation (4.75 mm), so 8 coarse units are about one frame (38 mm).

Three behaviours shape everything else:

1. **Reversing loses a varying slack** of 90 to 190 px. After a reversal fine units hardly move at all until a coarse unit forward takes up the
   slack. Position by feeding forward only, and recover a frame fed too far by going back past it, one coarse unit forward, then fine units.
2. **Fine units stall at random** along a long roll, and once the strip's tail has left the entrance roller, which is all they drive. The last
   frames may need a coarse unit, with whatever overshoot it brings.
3. **Hole reads are exact only modulo the pitch.** Expect under half a pitch per round of fine units (about 7 units), or a stall reads as a
   whole perforation.

### Measuring the film

Sprocket holes are visible in a stationary line with the carriage at 1420. In image columns (line reversed, first 2 pixels dropped) the hole
window is 960 to 1670. A hole is a bright run of 125 to 160 px (142 typical) in the red plane, using a threshold of half way between the
window's maximum and median. Ignore runs that touch the window's ends. The window is wider than two pitches, so there is always a
whole hole. Compare hole positions before and after a feed to get the travel. A read takes about 1.4 s, far quicker than a motor scan.

### Placing a frame

A **frame** is about 2605 px wide, and the gate shows about 0 to 2686. There is little to spare: place the frame's left edge at about 45.

To find it, scan 600 lines from step 2000 in 8-bit grey (red at gain 5) and look for the clear gap between frames: bright, even columns over
all rows, at least 60 px wide. Pale, even picture beside a gap can look like one in a shorter scan, so use all 600 rows. A gap ending at the gate's
end may just be running into the gate's mask. See `frame_edge()`.

Then a whole roll runs as:

1. Align frame 1 from the scan and feed its edge to 45, checking with a short scan (150 lines) after each feed, up to 3 times. Where the check and
   the holes disagree by more than 60 px, trust the holes.
2. Autofocus on frame 1. Set the gain from the film base in that first scan.
3. Scan 1800 lines from step 1500.
4. Check the frame's own scan: where the gap really is over all rows shows a frame too far in or too far on. If so, place it and scan again, once.
5. Feed about 38 mm less a margin, stopping short if the last scan shows the next frame's edge, and align again.
6. Stop at the end (below).

### The end of a strip

The vendor stops when page B bit 7 clears, which loses the last frame. Page B bits 7 **and** 6 both clear is the real end. An alignment
that finds no edge only means the end if the gate shows open light: with film there, the frame may just be blank or pale, so scan it as it lies.

An uncut roll starts with a fogged black leader. Its alignment scan is dark in every column. Cut it off before loading.

**Eject**: carriage home, then bursts of 8 coarse units forward while page B bit 6 sees film, then one more burst. The strip leaves through the
exit slot, on the same side as the entrance: keep the entering film and the exiting film apart, since a likely cause of stalls is one
dragging on the other.

**Rewind** to scan again: coarse units back (6 then 7) one at a time. Only start from a sensor state that says where the strip is: pressing on past
the start pushes the strip out of the entrance. Bit 6 clears about 760 px in from the auto-load position, so feed 3 more units back after it
clears. With bit 6 clear and bit 7 set, the strip is at its start already, so nothing should move.

## Picture corrections

None of this is protocol, but a driver will want it. Each is the host's job, since the scanner applies no calibration of its own.

- **Flat-field**: open light from the calibration window at step 30 and dark at step 700, 64 lines each, per pixel, at the same settings as the scan.
- **Response sag along a roll**: open light fell 14 % over 39 frames, caused by scanning and not lamp age, and each frame's clear film tracked it. Read open
  light before each frame and scale the frame to frame 1. Seek home before each reading: the tracked position drifts over a long run.
- **Falloff along the CCD**: about 0.95 of the middle at a frame's left end and 0.88 at its right.

## Not known

- What the registers without a label do. The writes above are the vendor's, and were enough.
- Page 0 bit 7 and the vendor's strip reference at that sensor; `pf1800.py` works without either.
- Page B bits 3 to 1; the interrupt endpoint's byte (read 0x57 once, may be the scan button); page 0x0D, which the vendor checks for 0xAA.
- Resolutions other than 1800 and 300 dpi (the vendor has settings for 600), and line art.
- Colour negative gains, and any unit other than this one, in particular other firmware versions.
