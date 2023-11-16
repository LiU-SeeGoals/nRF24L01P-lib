# nrf-tx-h563
Minimal example for a transmitter using an [NUCLEO-H563ZI](https://www.st.com/resource/en/user_manual/um3115-stm32h5-nucleo144-board-mb1404-stmicroelectronics.pdf).

Reset the board to send a new message. It prints on UART (115200 baud rate) when it sends the payload + some status info.

## Pins
Import the IOC file (or the project) to see these pins.

| Zio pin (CN7) | MCU pin | STM32 function | NRF pin | Label     |
|---------------|---------|----------------|---------|-----------|
| 1             | PC6     | GPIO_Output    | CE      | NRF_CE    |
| 2             | PB8     | GPIO_Output    | CSN     | NRF_CSN   |
| 6             | VDD     | VDD            | VDD     | -         |
| 8             | GND     | GND            | GND     | -         |
| 10            | PA5     | SPI1_SCK       | SCK     | NRF_SCK   |
| 12            | PG9     | SPI1_MISO      | M1      | NRF_MISO  |
| 14            | PB5     | SPI1_MOSI      | M0      | NRF_MOSI  |
| -             | PD9     | USART3_RX      | -       | USART3_RX |
| -             | PD8     | USART3_TX      | -       | USART3_TX |

### SPI
SPI1 is used. You'll have to set the data size to 8 bits and the prescaler for the clock parameters to 32.

### USART3
We're using the `Asynchronous` mode, but other than that the default settings are used. Make sure the correct MCU pins are being used though.
