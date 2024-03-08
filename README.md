# ch32x0-hal

This hal crates supports [embassy](https://github.com/embassy-rs/embassy) for CH32X0 series of microcontrollers.

This will be the standard HAL skelton for CH32X0/CH32V003/CH32V103/CH32V20x/CH32V30x/CH32L103 series of microcontrollers.

## TODOs

- [x] Embassy timer driver using Systick
- [x] SDI Debug
- [x] GPIO
  - [x] Interrupts (async)
- [x] UART
  - [x] Half-duplex - I need this for my personal project
  - [ ] UART async (DMA)
- [x] SPI
  - [x] blocking API
  - [ ] async API
- [x] ADC
- [x] OPA
- [ ] USBPD
- [ ] USB
- [ ] PIOC
- [ ] I2C - not working. First batch of CH32X035 chips doesn't have I2C

The ADC channels 3, 7, 11, and 15, as well as the I2C function are not applicable for
products with the fifth-to-last digit of the batch number being 0.
