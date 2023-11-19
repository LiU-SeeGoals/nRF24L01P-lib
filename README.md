# nRF24L01+ library
This is a library for using the wireless module nRF24L01+ through the STMicroelectronics HAL layer.

Please note that if a device other than `STM32H563xx` and `STM32H755xx` is to be used, then the HAL library includes in `nrf24l01.h` has to be extended.

To get the print functions to work you'll have to forward printf to UART, see the examples for one way to do it.

## Documentation
Run `doxygen` in project root to generate documentation.

[Datasheet](https://www.sparkfun.com/datasheets/Components/SMD/nRF24L01Pluss_Preliminary_Product_Specification_v1_0.pdf) ([sharepoint](https://liuonline.sharepoint.com/:b:/r/sites/ToeBiters/Shared%20Documents/Hardware/datasheets/nRF24L01Pluss_Preliminary_Product_Specification_v1_0.pdf?csf=1&web=1&e=uNih4T)).

## Examples
Beneath `./examples/` there are two examples of transmitting from a `STM32H563ZIT6U` and recieving on an `STM32H755ZIT6U`. One is a very simple to show the basics. One is a more full fledged example, meant to showcase more features of the NRF24L01+ device.

More information is available in the README of each projects folder.
