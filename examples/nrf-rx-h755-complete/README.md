# nrf-rx-h755-complete
A more complete example for the usage of the library with an [NUCLEO-H755ZI-Q](https://www.st.com/resource/en/user_manual/um2408-stm32h7-nucleo144-boards-mb1363-stmicroelectronics.pdf).  

All code of interest is within `/* USER CODE BEGIN 0 */` inside of `main.c` of the `CM7` core, generally all other code is generated from STM32CubeMX. You can use the provided .ioc file to generate it.

## Pins
| Zio pin (CN7) | MCU pin | STM32 function | NRF pin | Label     |
|---------------|---------|----------------|---------|-----------|
| 1             | PC6     | GPIO_Output    | CE      | NRF_CE    |
| 2             | PB8     | GPIO_Output    | CSN     | NRF_CSN   |
| 3             | PB15    | GPIO_EXTI15    | IRQ     | NRF_IRQ   |
| 6             | VDD     | VDD            | VDD     | -         |
| 8             | GND     | GND            | GND     | -         |
| 10            | PA5     | SPI1_SCK       | SCK     | NRF_SCK   |
| 12            | PA6     | SPI1_MISO      | M1      | NRF_MISO  |
| 14            | PB5     | SPI1_MOSI      | M0      | NRF_MOSI  |
| -             | PC13    | GPIO_EXTI13    | -       | BTN_USER  |
| -             | PD8     | USART3_RX      | -       | USART3_RX |
| -             | PD9     | USART3_TX      | -       | USART3_TX |
