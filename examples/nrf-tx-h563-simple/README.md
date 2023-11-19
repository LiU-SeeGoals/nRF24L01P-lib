# nrf-tx-h563-simple
Simple example for a transmitter using an [NUCLEO-H563ZI](https://www.st.com/resource/en/user_manual/um3115-stm32h5-nucleo144-board-mb1404-stmicroelectronics.pdf).

Press the user button (blue) to send a message, the [simple receiver](https://github.com/LiU-SeeGoals/nRF24L01P-lib/tree/main/examples/nrf-rx-h755-simple) can handle receiving it.

It prints info on UART (115200 baud rate).

All code of interest is within `/* USER CODE BEGIN 0 */` inside of `main.c`, generally all other code is generated from STM32CubeMX. You can use the provided .ioc file to generate it.

## Pins
Import the IOC file (or the project) to see these pins.

| Zio pin (CN7) | MCU pin | STM32 function | NRF pin | Label       |
|---------------|---------|----------------|---------|-------------|
| 1             | PC6     | GPIO_Output    | CE      | NRF_CE      |
| 2             | PB8     | GPIO_Output    | CSN     | NRF_CSN     |
| 6             | VDD     | VDD            | VDD     | -           |
| 8             | GND     | GND            | GND     | -           |
| 10            | PA5     | SPI1_SCK       | SCK     | NRF_SCK     |
| 12            | PG9     | SPI1_MISO      | M1      | NRF_MISO    |
| 14            | PB5     | SPI1_MOSI      | M0      | NRF_MOSI    |
| -             | PD9     | USART3_RX      | -       | USART3_RX   |
| -             | PD8     | USART3_TX      | -       | USART3_TX   |
| -             | PC13    | GPIO_EXTI13    | -       | USER_BUTTON |

### SPI
SPI1 is used in Full-Duplex Master mode. You'll have to change Data Size to 8 Bits and the Prescaler to 32.

### USART3
Asynchronous mode.

### User button
We catch an interrupt when the user button is pressed.
