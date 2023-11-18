#include "nrf24l01.h"

/* Local defines */
// Config bits
#define CFG_PRIM_RX_BIT 0x01
#define CFG_PWR_UP_BIT  0x02
#define CFG_EN_CRC_BIT  0x08

// Status bits
#define STAT_TX_FULL_BIT  0x01
#define STAT_RX_P_NO_BITS 0x0e
#define STAT_MAX_RT_BIT   0x10
#define STAT_TX_DS_BIT    0x20
#define STAT_RX_DR_BIT    0x40

// FIFO status bits
#define FIFO_RX_EMPTY 0x01
#define FIFO_RX_FULL  0x02
#define FIFO_TX_EMPTY 0x10
#define FIFO_TX_FULL  0x20
#define FIFO_TX_REUSE 0x40

/* Local globals */
SPI_HandleTypeDef *HSPI;
GPIO_TypeDef *NRF_CSN_Port;
uint16_t NRF_CSN_Pin;
GPIO_TypeDef *NRF_CE_Port;
uint16_t NRF_CE_Pin;

/*
 * Main functions
 */

NRF_Status NRF_Init(SPI_HandleTypeDef *handle, GPIO_TypeDef *PortCSN, uint16_t PinCSN, GPIO_TypeDef *PortCE, uint16_t PinCE) {
  HSPI = handle;
  NRF_CSN_Port = PortCSN;
  NRF_CSN_Pin = PinCSN;
  NRF_CE_Port = PortCE;
  NRF_CE_Pin = PinCE;

  // Reset state
  NRF_CSN_set();
  NRF_CE_reset();

  // Power on reset
  HAL_Delay(100);

  return HAL_OK;
}

NRF_Status NRF_SendCommand(uint8_t cmd) {
  NRF_Status ret = HAL_OK;
  uint8_t status;

  NRF_CSN_reset();
  ret = HAL_SPI_TransmitReceive(HSPI, &cmd, &status, 1, NRF_SPI_TIMEOUT);
  if (ret != HAL_OK) {
    return ret;
  }
  NRF_CSN_set();

  return ret;
}

NRF_Status NRF_SendWriteCommand(uint8_t cmd, uint8_t *write, uint8_t length) {
  NRF_Status ret = HAL_OK;
  uint8_t status;

  NRF_CSN_reset();
  ret = HAL_SPI_TransmitReceive(HSPI, &cmd, &status, 1, NRF_SPI_TIMEOUT);
  if (ret != HAL_OK) {
    return ret;
  }
  ret = HAL_SPI_Transmit(HSPI, write, length, NRF_SPI_TIMEOUT);
  if (ret != HAL_OK) {
    return ret;
  }
  NRF_CSN_set();

  return ret;
}

NRF_Status NRF_SendReadCommand(uint8_t cmd, uint8_t *read, uint8_t length) {
  NRF_Status ret = HAL_OK;
  uint8_t status;

  NRF_CSN_reset();
  ret = HAL_SPI_TransmitReceive(HSPI, &cmd, &status, 1, NRF_SPI_TIMEOUT);
  if(ret != HAL_OK) {
    return ret;
  }
  ret = HAL_SPI_Receive(HSPI, read, length, NRF_SPI_TIMEOUT);
  if(ret != HAL_OK) {
    return ret;
  }
  NRF_CSN_set();

  return ret;
}

void NRF_CSN_set() {
  HAL_GPIO_WritePin(NRF_CSN_Port, NRF_CSN_Pin, GPIO_PIN_SET);
}

void NRF_CSN_reset() {
  HAL_GPIO_WritePin(NRF_CSN_Port, NRF_CSN_Pin, GPIO_PIN_RESET);
}

void NRF_CE_set() {
  HAL_GPIO_WritePin(NRF_CE_Port, NRF_CE_Pin, GPIO_PIN_SET);
}

void NRF_CE_reset() {
  HAL_GPIO_WritePin(NRF_CE_Port, NRF_CE_Pin, GPIO_PIN_RESET);
}


/*
 * Helper functions
 */


/* Writing */
NRF_Status NRF_WriteRegister(uint8_t reg, uint8_t *write, uint8_t length) {
  return NRF_SendWriteCommand(NRF_CMD_W_REGISTER | reg, write, length);
}

NRF_Status NRF_WriteRegisterByte(uint8_t reg, uint8_t byte) {
  uint8_t write = byte;
  return NRF_WriteRegister(reg, &write, 1);
}

NRF_Status NRF_SetRegisterBit(uint8_t reg, uint8_t bit) {
  NRF_Status ret = HAL_OK;
  uint8_t cfg = 0x00;

  ret = NRF_ReadRegister(reg, &cfg, 1);
  if (ret != HAL_OK) {
    return ret;
  }
  cfg = cfg | (1 << bit);
  return NRF_WriteRegister(reg, &cfg, 1);
}

NRF_Status NRF_ResetRegisterBit(uint8_t reg, uint8_t bit) {
  NRF_Status ret = HAL_OK;
  uint8_t cfg = 0x00;

  ret = NRF_ReadRegister(reg, &cfg, 1);
  if (ret != HAL_OK) {
    return ret;
  }
  cfg = cfg & ~(1 << bit);
  return NRF_WriteRegister(reg, &cfg, 1);
}

NRF_Status NRF_WritePayload(uint8_t *write, uint8_t length) {
  return NRF_SendWriteCommand(NRF_CMD_W_TX_PAYLOAD, write, length);
}


/* Reading */
NRF_Status NRF_ReadRegister(uint8_t reg, uint8_t *read, uint8_t length) {
  return NRF_SendReadCommand(NRF_CMD_R_REGISTER | reg, read, length);
}

uint8_t NRF_ReadRegisterByte(uint8_t reg) {
  uint8_t read;
  NRF_SendReadCommand(NRF_CMD_R_REGISTER | reg, &read, 1);
  return read;
}

uint8_t NRF_ReadStatus() {
  uint8_t status;
  uint8_t cmd = NRF_CMD_NOP;

  NRF_CSN_reset();
  HAL_SPI_TransmitReceive(HSPI, &cmd, &status, 1, NRF_SPI_TIMEOUT);
  NRF_CSN_set();

  return status;
}

NRF_Status NRF_ReadPayload(uint8_t *read, uint8_t length) {
  return NRF_SendReadCommand(NRF_CMD_R_RX_PAYLOAD, read, length);
}

NRF_Status NRF_EnterModeTX() {
  NRF_Status ret = HAL_OK;

  // unset PRIM_RX
  ret = NRF_ResetRegisterBit(NRF_REG_CONFIG, 0);
  if(ret != HAL_OK) {
    return ret;
  }

  // set PWR_UP
  ret = NRF_SetRegisterBit(NRF_REG_CONFIG, 1);
  if(ret != HAL_OK) {
    return ret;
  }

  // When in power down mode NRF must settle
  // for ~1.5ms before entering TX/RX modes
  HAL_Delay(2);
  NRF_CE_set();

  // Flush FIFO
  ret = NRF_SendCommand(NRF_CMD_FLUSH_TX);
  if(ret != HAL_OK) {
    return ret;
  }

  return ret;
}

NRF_Status NRF_EnterModeRX() {
  NRF_Status ret = HAL_OK;

  ret = NRF_SetRegisterBit(NRF_REG_CONFIG, 0); // set PRIM_RX
  if(ret != HAL_OK) {
    return ret;
  }

  ret = NRF_SetRegisterBit(NRF_REG_CONFIG, 1); // set PWR_UP
  if(ret != HAL_OK) {
    return ret;
  }

  // When in power down mode NRF must settle
  // for ~1.5ms before entering TX/RX modes
  HAL_Delay(2);
  NRF_CE_set();

  // Flush FIFO
  ret = NRF_SendCommand(NRF_CMD_FLUSH_RX);
  if(ret != HAL_OK) {
    return ret;
  }

  return ret;
}


/*
 * Misc functions
 */

NRF_Status NRF_VerifySPI() {
  NRF_Status ret = HAL_OK;
  uint8_t write[5] = "0x57!";
  uint8_t read[5];

  ret = NRF_WriteRegister(NRF_REG_TX_ADDR, write, 5);
  if (ret != HAL_OK) {
    return ret;
  }
  ret = NRF_ReadRegister(NRF_REG_TX_ADDR, read, 5);
  if (ret != HAL_OK) {
    return ret;
  }

  for(int i = 0; i < 5; i++) {
    if (write[i] != read[i]) {
      return HAL_ERROR;
    }
  }

  return ret;
}

void NRF_Reset() {
  NRF_WriteRegisterByte(NRF_REG_CONFIG,       0x08);
  NRF_WriteRegisterByte(NRF_REG_EN_AA,        0x3f);
  NRF_WriteRegisterByte(NRF_REG_EN_RXADDR,    0x03);
  NRF_WriteRegisterByte(NRF_REG_SETUP_AW,     0x03);
  NRF_WriteRegisterByte(NRF_REG_SETUP_RETR,   0x03);
  NRF_WriteRegisterByte(NRF_REG_RF_CH,        0x02);
  NRF_WriteRegisterByte(NRF_REG_RF_SETUP,     0x0e);
  NRF_WriteRegisterByte(NRF_REG_STATUS,       0x70); // clear eventual bits

  uint8_t address[5] = {0xE7, 0xE7, 0xE7, 0xE7, 0xE7};
  uint8_t address2[5] = {0xC2, 0xC2, 0xC2, 0xC2, 0xC2};
  NRF_WriteRegister(NRF_REG_RX_ADDR_P0, address, 5);
  NRF_WriteRegister(NRF_REG_RX_ADDR_P1, address2, 5);
  NRF_WriteRegisterByte(NRF_REG_RX_ADDR_P2,   0xC3);
  NRF_WriteRegisterByte(NRF_REG_RX_ADDR_P3,   0xC4);
  NRF_WriteRegisterByte(NRF_REG_RX_ADDR_P4,   0xC5);
  NRF_WriteRegisterByte(NRF_REG_RX_ADDR_P5,   0xC6);
  NRF_WriteRegister(NRF_REG_TX_ADDR, address, 5);
  NRF_WriteRegisterByte(NRF_REG_RX_PW_P0,     0x00);
  NRF_WriteRegisterByte(NRF_REG_RX_PW_P1,     0x00);
  NRF_WriteRegisterByte(NRF_REG_RX_PW_P2,     0x00);
  NRF_WriteRegisterByte(NRF_REG_RX_PW_P3,     0x00);
  NRF_WriteRegisterByte(NRF_REG_RX_PW_P4,     0x00);
  NRF_WriteRegisterByte(NRF_REG_RX_PW_P5,     0x00);

  NRF_WriteRegisterByte(NRF_REG_FIFO_STATUS,  0x00);
  NRF_WriteRegisterByte(NRF_REG_DYNPD,        0x00);
  NRF_WriteRegisterByte(NRF_REG_FEATURE,      0x00);
}

void NRF_PrintStatus() {
  uint8_t status = NRF_ReadStatus();

  printf("Status register: %02X\r\n", status);
  printf("CE: %d\r\n", HAL_GPIO_ReadPin(NRF_CE_Port, NRF_CE_Pin));
  printf("CSN: %d\r\n", HAL_GPIO_ReadPin(NRF_CSN_Port, NRF_CSN_Pin));
  printf("TX_FULL:  %1X\r\n", status & STAT_TX_FULL_BIT);
  printf("RX_P_NO:  %1X\r\n", (status & STAT_RX_P_NO_BITS)  >> 1);
  printf("MAX_RT:   %1X\r\n", (status & STAT_MAX_RT_BIT)    >> 4);
  printf("TX_DS:    %1X\r\n", (status & STAT_TX_DS_BIT)     >> 5);
  printf("RX_DR:    %1X\r\n", (status & STAT_RX_DR_BIT)     >> 6);
  printf("\r\n");
}

void NRF_PrintFIFOStatus() {
  uint8_t reg = NRF_ReadRegisterByte(NRF_REG_FIFO_STATUS);

  printf("FIFO status register: %02X\r\n", reg);
  printf("RX_EMPTY:   %2X\r\n", reg & FIFO_RX_EMPTY);
  printf("RX_FULL:    %2X\r\n", (reg & FIFO_RX_FULL)     >> 1);
  printf("TX_EMPTY:   %2X\r\n", (reg & FIFO_TX_EMPTY)    >> 4);
  printf("TX_FULL:    %2X\r\n", (reg & FIFO_TX_FULL)     >> 5);
  printf("TX_REUSE:   %2X\r\n", (reg & FIFO_TX_REUSE)    >> 6);
  printf("\r\n");
}
