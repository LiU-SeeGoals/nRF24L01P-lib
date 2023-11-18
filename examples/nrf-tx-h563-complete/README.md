# nrf-tx-h563-complete
A more complete example for the usage of the library with an [NUCLEO-H563ZI](https://www.st.com/resource/en/user_manual/um3115-stm32h5-nucleo144-board-mb1404-stmicroelectronics.pdf).  

## Pins
| Zio pin (CN7) | MCU pin | STM32 function | NRF pin | Label     |
|---------------|---------|----------------|---------|-----------|
| 1             | PC6     | GPIO_Output    | CE      | NRF_CE    |
| 2             | PB8     | GPIO_Output    | CSN     | NRF_CSN   |
| 6             | VDD     | VDD            | VDD     | -         |
| 8             | GND     | GND            | GND     | -         |
| 10            | PA5     | SPI1_SCK       | SCK     | NRF_SCK   |
| 12            | PG9     | SPI1_MISO      | M1      | NRF_MISO  |
| 14            | PB5     | SPI1_MOSI      | M0      | NRF_MOSI  |
| 20            | PF3     | GPIO_EXTI3     | IRQ     | NRF_IRQ   |
| -             | PC13    | GPIO_EXTI13    | -       | BTN_USER  |
| -             | PD9     | USART3_RX      | -       | USART3_RX |
| -             | PD8     | USART3_TX      | -       | USART3_TX |
