# nRF24L01+ library
This is a library for using the wireless module nRF24L01+ through the STMicroelectronics HAL layer.

Please note that if a device other than `STM32H563xx` and `STM32H755xx` is to be used, then the HAL library includes in `nrf24l01.h` has to be extended.

To have the print functions to work you'll have to forward printf to UART, see the examples for one way to do it.

## Documentation
Run `doxygen` in project root to generate documentation.

[Datasheet](https://www.sparkfun.com/datasheets/Components/SMD/nRF24L01Pluss_Preliminary_Product_Specification_v1_0.pdf) ([sharepoint](https://liuonline.sharepoint.com/:b:/r/sites/ToeBiters/Shared%20Documents/Hardware/datasheets/nRF24L01Pluss_Preliminary_Product_Specification_v1_0.pdf?csf=1&web=1&e=uNih4T)).

## Examples
Beneath `./examples/` there is a tranciever example where a `STM32H563ZIT6U` is used as the transmitter and an `STM32H755ZIT6U` is used as the receiver. More information is available in the README of each projects folder.
