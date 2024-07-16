# DTM Faller Car System

## Nano Extension Board
The extension board don't have a 5VDC voltage regulator, it uses the voltage regulator on the Nano Board, and sends that 5VDC to all the pin-header (v).

The extension board have a 3V3 voltage regulator, there gets it's power from the Nano voltage regulator, and sends the 3V3 to all 3V3 pins on the extensions board.

### Nano on board voltage regulator (clon-board)
The Nano voltage regulator is a 1117 5VDC voltage regulator. operations values, Vin: 7 - 15VDC max. (absolut max. 20VDC)   Iout: 800mA max. (1A peak).

https://www.ti.com/lit/ds/symlink/tlv1117.pdf?ts=1721071857392&ref_url=https%253A%252F%252Fwww.mouser.com%252F
