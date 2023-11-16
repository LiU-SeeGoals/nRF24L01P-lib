#ifndef NRF24L01_H
#define NRF24L01_H

#include <stdint.h>

#if defined STM32H563xx
#include "stm32h5xx_hal.h"
#elif defined STM32H755xx
#include "stm32h7xx_hal.h"
#endif

/**
 * @name Defines
 * @anchor defines
 *
 * Ungrouped defines
 */
//!@{

//! Timeout for all SPI communication attempts.
#define NRF_SPI_TIMEOUT     10
//!@}

/**
 * @name Commands
 * @anchor commands
 *
 * Available commands for the device.
 *
 * See table 19 on page 48 in @datasheet.
 */
//!@{
#define NRF_CMD_READ                      0x00
#define NRF_CMD_WRITE                     0x20
#define NRF_CMD_R_RX_PAYLOAD              0x61
#define NRF_CMD_W_TX_PAYLOAD              0xA0
#define NRF_CMD_W_TX_PAYLOAD_NO_ACK       0xB0
#define NRF_CMD_FLUSH_TX                  0xE1
#define NRF_CMD_FLUSH_RX                  0xE2
#define NRF_CMD_NOP                       0xFF
//!@}

/**
 * @name Registers
 * @anchor registers
 * @anchor register
 *
 * Available registers on the device.
 *
 * See table 27 on page 54 in @datasheet.
 */
//!@{
#define NRF_REG_CONFIG      0x00
#define NRF_REG_EN_AA       0x01
#define NRF_REG_EN_RXADDR   0x02
#define NRF_REG_RF_CH       0x05
#define NRF_REG_STATUS      0x07
#define NRF_REG_TX_ADDR     0x10
#define NRF_REG_RX_PW_P0    0x11
#define NRF_REG_RX_PW_P1    0x12
#define NRF_REG_RX_PW_P2    0x13
#define NRF_REG_RX_PW_P3    0x14
#define NRF_REG_RX_PW_P4    0x15
#define NRF_REG_RX_PW_P5    0x16
#define NRF_REG_FIFO_STATUS 0x17
#define NRF_REG_FEATURE     0x1D
#define NRF_REG_RX_ADDR_P0  0x0A
#define NRF_REG_RX_ADDR_P1  0x0B
#define NRF_REG_RX_ADDR_P2  0x0C
#define NRF_REG_RX_ADDR_P3  0x0D
#define NRF_REG_RX_ADDR_P4  0x0E
#define NRF_REG_RX_ADDR_P5  0x0F
//!@}

typedef HAL_StatusTypeDef NRF_Status;

/* Misc */
void NRF_CSN_set();
void NRF_CSN_reset();
void NRF_CE_set();
void NRF_CE_reset();

/**
 * \brief Initalises the library and make the device entery standby-I mode.
 *
 * \param handle A pointer at the SPI_HandleTypeDef which HAL creates for the
 *          SPI communication.
 * \param PortCSN A pointer at the GPIO_TypeDef for the port that the device CSN
 *          pin is on.
 * \param PinCSN The pin that the devices CSN is connected to.
 * \param PortCE A pointer at the GPIO_TypeDef far the port that the device CE
 *          pin is on.
 * \param PinCE The pin that the devices CE is connected to.
 * \return NRF_Status
 */
NRF_Status NRF_Init(SPI_HandleTypeDef *handle,
                    GPIO_TypeDef *PortCSN,
                    uint16_t PinCSN,
                    GPIO_TypeDef *PortCE,
                    uint16_t PinCE);

/**
 * \brief Sends a command to the device.
 *
 * \param cmd One of the available @ref commands.
 * \return NRF_Status
 */
NRF_Status NRF_SendCommand(uint8_t cmd);

/**
 * \brief Writes a buffer to a register.
 *
 * \param reg One of the available @ref registers.
 * \param write A pointer to the buffer to write.
 * \param length The length of the buffer.
 * \return NRF_Status
 */
NRF_Status NRF_WriteRegister(uint8_t reg, uint8_t *write, uint8_t length);

/**
 * \brief Reads a register to a buffer.
 *
 * \param reg One of the available @ref registers.
 * \param read A pointer to the buffer to read to.
 * \param length The length of the buffer.
 * \return NRF_Status
 */
NRF_Status NRF_ReadRegister(uint8_t reg, uint8_t *read, uint8_t length);

/**
 * \brief Verifies that SPI communication with the device works.
 *
 * We write a predetermined value to the TX_ADDR @ref register.
 * Then we read the value of the register.
 * If the two values are the same then SPI is working.
 *
 * \return NRF_Status
 */
NRF_Status NRF_VerifySPI();

NRF_Status NRF_PrintStatus();
NRF_Status NRF_PrintFIFOStatus();

/* Setters */
NRF_Status NRF_ToggleBitReg(uint8_t reg, uint8_t bit);
NRF_Status NRF_WriteBitReg(uint8_t reg, uint8_t bit);
NRF_Status NRF_PowerUp();
NRF_Status NRF_SetModeRx();
NRF_Status NRF_WritePayload(uint8_t *payload, uint8_t length);
NRF_Status NRF_ReadPayload(uint8_t *payload, uint8_t length);

/* Getters */
NRF_Status NRF_GetStatus(uint8_t *status);

#endif /* NRF24L01_H */
