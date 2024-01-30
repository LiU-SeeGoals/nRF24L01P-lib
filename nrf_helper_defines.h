/*
 * This files just have a lot of helper definitions for
 * bits and masks which can be used to make calls
 * more readable, but which aren't neccessery.
 */

/**
 * @name NRF register bit numbers
 *
 * Name of particular bits. Used to make code easier to read.
 * See page 54 in datasheet for NRF.
 */
//!@{
#define STATUS_BIT_RX_DR          6
#define FEATURE_BIT_EN_ACK_PAY    1
#define FEATURE_BIT_EN_DPL        2
//!@}

/**
 * @name NRF register masks
 *
 * Are used to mask out wanted bits from registers in the device. Makes code easier to read.
 */
//!@{
#define STATUS_MASK_BIT_RX_DR      0x40       /**< Data ready in RX FIFO interrupt. */
#define STATUS_MASK_BIT_TX_DS      0x20
#define STATUS_MASK_BIT_MAX_RT     0x10
#define STATUS_MASK_BITS_RX_P_NO   0x0E       /**< Data pipe number for payload available for readin from RX_FIFO. */
//!@}
