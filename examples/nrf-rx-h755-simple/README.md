# nrf-rx-h755-simple
Minimal example for a receiver using an [NUCLEO-H755ZI-Q](https://www.st.com/resource/en/user_manual/um2408-stm32h7-nucleo144-boards-mb1363-stmicroelectronics.pdf).  

When the board start ups it recieves messages forever and prints the received payload on UART (115200 baud rate). Send the messages from the [simple transmitter](https://github.com/LiU-SeeGoals/nRF24L01P-lib/tree/main/examples/nrf-tx-h563-simple).

If the user button (blue) is pressed the status and FIFO status registers will be printed.

All code of interest is within `/* USER CODE BEGIN 0 */` inside of `main.c` of the `CM7` core, generally all other code is generated from STM32CubeMX. You can use the provided .ioc file to generate it.

## Clock
You might have to resolve clock issues from the MX view. Go into "Clock Configuration" and press "Resolve Clock Issues".

## Pins

| Zio pin (CN7) | MCU pin | STM32 function | NRF pin | Label       |
|---------------|---------|----------------|---------|-------------|
| 1             | PC6     | GPIO_Output    | CE      | NRF_CE      |
| 2             | PB8     | GPIO_Output    | CSN     | NRF_CSN     |
| 6             | VDD     | VDD            | VDD     | -           |
| 8             | GND     | GND            | GND     | -           |
| 10            | PA5     | SPI1_SCK       | SCK     | NRF_SCK     |
| 12            | PA6     | SPI1_MISO      | M1      | NRF_MISO    |
| 14            | PB5     | SPI1_MOSI      | M0      | NRF_MOSI    |
| -             | PD8     | USART3_RX      | -       | USART3_RX   |
| -             | PD9     | USART3_TX      | -       | USART3_TX   |
| -             | PC13    | GPIO_EXTI13    | -       | USER_BUTTON |

### SPI
SPI1 is used in Full-Duplex Master mode. You'll have to change Data Size to 8 Bits and the Prescaler to 32.

### USART3
Asynchronous mode.

### User button
We catch an interrupt when the user button is pressed.
