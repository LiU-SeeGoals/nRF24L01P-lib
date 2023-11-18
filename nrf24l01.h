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
 * @anchor command
 *
 * Available commands for the device.
 *
 * See table 19 on page 48 in @datasheet.
 */
//!@{
#define NRF_CMD_R_REGISTER                0x00
#define NRF_CMD_W_REGISTER                0x20
#define NRF_CMD_R_RX_PAYLOAD              0x61
#define NRF_CMD_W_TX_PAYLOAD              0xA0
#define NRF_CMD_FLUSH_TX                  0xE1
#define NRF_CMD_FLUSH_RX                  0xE2
#define NRF_CMD_REUSE_TX_PL               0xE3
#define NRF_CMD_R_RX_PL_WID               0x60
#define NRF_CMD_W_ACK_PAYLOAD             0xA8
#define NRF_CMD_W_TX_PAYLOAD_NO_ACK       0xB0
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
#define NRF_REG_SETUP_AW    0x03
#define NRF_REG_SETUP_RETR  0x04
#define NRF_REG_RF_CH       0x05
#define NRF_REG_RF_SETUP    0x06
#define NRF_REG_STATUS      0x07
#define NRF_REG_OBSERVE_TX  0x08
#define NRF_REG_RPD         0x09
#define NRF_REG_RX_ADDR_P0  0x0A
#define NRF_REG_RX_ADDR_P1  0x0B
#define NRF_REG_RX_ADDR_P2  0x0C
#define NRF_REG_RX_ADDR_P3  0x0D
#define NRF_REG_RX_ADDR_P4  0x0E
#define NRF_REG_RX_ADDR_P5  0x0F
#define NRF_REG_TX_ADDR     0x10
#define NRF_REG_RX_PW_P0    0x11
#define NRF_REG_RX_PW_P1    0x12
#define NRF_REG_RX_PW_P2    0x13
#define NRF_REG_RX_PW_P3    0x14
#define NRF_REG_RX_PW_P4    0x15
#define NRF_REG_RX_PW_P5    0x16
#define NRF_REG_FIFO_STATUS 0x17
#define NRF_REG_DYNPD       0x1C
#define NRF_REG_FEATURE     0x1D
//!@}


/**
 * \brief Currently we're just copying HAL's status.
 * @anchor NRF_Status
 */
typedef HAL_StatusTypeDef NRF_Status;


/**
 * @name Main
 * @anchor mains
 * @anchor main
 *
 * These functions are the main way to communicate directly
 * with the device.
 */
//!@{
/**
 * \brief Initalises the library and make the device enter standby-I mode.
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
 * \brief Sends a @ref command to the device.
 *
 * \param cmd A @ref command.
 * \return NRF_Status
 */
NRF_Status NRF_SendCommand(uint8_t cmd);

/**
 * \brief Sends a write @ref command to the device along with
 * a buffer to write.
 *
 * \param cmd A write @ref command.
 * \param write Buffer to write.
 * \param length Length of buffer.
 * \return NRF_Status
 */
NRF_Status NRF_SendWriteCommand(uint8_t cmd, uint8_t *write, uint8_t length);

/**
 * \brief Sends a read @ref command to the device along with
 * a buffer to read to.
 *
 * \param cmd A read @ref command.
 * \param write Buffer to read to.
 * \param length Length of buffer.
 * \return NRF_Status
 */
NRF_Status NRF_SendReadCommand(uint8_t cmd, uint8_t *read, uint8_t length);

void NRF_CSN_set();
void NRF_CSN_reset();
void NRF_CE_set();
void NRF_CE_reset();
//!@}


/**
 * @name Helper
 * @anchor helpers
 *
 * These functions are built with the help of the @ref main functions
 * to make interfacing with the device nicer.
 */
//!@{

/**
 * \brief Write a buffer to a @ref register.
 *
 * \param reg One of the available @ref registers.
 * \param write A pointer to the buffer to write.
 * \param length The length of the buffer.
 * \return NRF_Status
 */
NRF_Status NRF_WriteRegister(uint8_t reg, uint8_t *write, uint8_t length);

/**
 * \brief Write a byte to a @ref register.
 *
 * \param reg A @ref register.
 * \param byte The byte to write to the register.
 * \return NRF_Status
 */
NRF_Status NRF_WriteRegisterByte(uint8_t reg, uint8_t byte);

/**
 * \brief Set a bit to 1 in a @ref register.
 *
 * \param reg A @ref register.
 * \param bit A particular bit within the register.
 * \return NRF_Status
 */
NRF_Status NRF_SetRegisterBit(uint8_t reg, uint8_t bit);

/**
 * \brief Set a bit to 0 in a @ref register.
 *
 * \param reg A @ref register.
 * \param bit A particular bit within the register.
 * \return NRF_Status
 */
NRF_Status NRF_ResetRegisterBit(uint8_t reg, uint8_t bit);

NRF_Status NRF_WritePayload(uint8_t *payload, uint8_t length);

/**
 * \brief Read a register to a buffer.
 *
 * \param reg A @ref register.
 * \param read A pointer to the buffer to read to.
 * \param length The length of the buffer.
 * \return NRF_Status
 */
NRF_Status NRF_ReadRegister(uint8_t reg, uint8_t *read, uint8_t length);

/**
 * \brief Read a byte from a register.
 *
 * \param reg A @ref register.
 * \return The byte in the register.
 */
uint8_t NRF_ReadRegisterByte(uint8_t reg);

/**
 * \brief Read status by issuing a NOP.
 *
 * More efficient since only a single SPI transmit/recieve is issued.
 *
 * \return The status register byte.
 */
uint8_t NRF_ReadStatus();

NRF_Status NRF_ReadPayload(uint8_t *payload, uint8_t length);

/**
 * \brief Make the device enter TX mode.
 */
NRF_Status NRF_EnterModeTX();

/**
 * \brief Make the device enter RX mode.
 */
NRF_Status NRF_EnterModeRX();
//!@}

/**
 * @name Misc
 * @anchor misc
 *
 * These can be useful for debugging.
 */
//!@{

/**
 * \brief Verifies that SPI communication with the device works.
 *
 * We write a predetermined value to the TX_ADDR @ref register.
 * Then we read the value of the register.
 * If the two values are the same then SPI is working.
 *
 * \return HAL_OK on success.
 */
NRF_Status NRF_VerifySPI();

/**
 * \brief Reset all registers to reset values
 * specified in @datasheet table 27 on page 59.
 *
 * This is pretty overkill but useful for testing since the device
 * doesn't reset on a fast enough off-on power switch.
 */
void NRF_Reset();

void NRF_PrintStatus();
void NRF_PrintFIFOStatus();
//!@}

#endif /* NRF24L01_H */
