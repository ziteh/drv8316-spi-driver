# DRV8516 SPI Driver

A platform independent library for TI [DRV8316](https://www.ti.com/product/DRV8316)/[DRV8316C](https://www.ti.com/product/DRV8316C) 3-phase motor driver SPI communication.

## SPI

### Condition

- SPI Mode = 1.
  - CPOL = 0: Clock is low when idle.
  - CPHA = 1: Data is captured on the second edge (i.e. falling edge) and propagated on the rising edge.
- CS (nSCS) pin active low.
- Data size (data frame format) is 16-bit.
- Bit order is MSB first.
- Max clock rates up to 10 MHz (period=100ns).

### Data Format

MOSI format:

| MSB | B14 | B13 | B12 | B11 | B10 | B9  | B8  | B7  | B6  | B5  | B4  | B3  | B2  | B1  | LSB |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| RW  | A5  | A4  | A3  | A2  | A1  | A0  | PA  | D7  | D6  | D5  | D4  | D3  | D2  | D1  | D0  |

- RW: Read or write. =0 for write, =1 for read.
- A*x*: 6 bit address.
- PA: Even parity.
- D*x*: 8 bit data.

MISO format:

| MSB | B14 | B13 | B12 | B11 | B10 | B9  | B8  | B7  | B6  | B5  | B4  | B3  | B2  | B1  | LSB |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| S7  | S6  | S5  | S4  | S3  | S2  | S1  | S0  | D7  | D6  | D5  | D4  | D3  | D2  | D1  | D0  |

- S*x*: 8 bit status.
- D*x*: 8 bit data.
