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

// Local globals
SPI_HandleTypeDef *hspi;

// Functions
void NRF_CSN_set() {
  HAL_GPIO_WritePin(NRF_CSN_GPIO_Port, NRF_CSN_Pin, GPIO_PIN_SET);
}

void NRF_CSN_reset() {
  HAL_GPIO_WritePin(NRF_CSN_GPIO_Port, NRF_CSN_Pin, GPIO_PIN_RESET);
}

void NRF_CE_set() {
  HAL_GPIO_WritePin(NRF_CE_GPIO_Port, NRF_CE_Pin, GPIO_PIN_SET);
}

void NRF_CE_reset() {
  HAL_GPIO_WritePin(NRF_CE_GPIO_Port, NRF_CE_Pin, GPIO_PIN_RESET);
}

// Puts the device in standby-1 mode (table 15, p.23)
NRF_Status NRF_Init(SPI_HandleTypeDef *handle) {
  hspi = handle;

  // Reset state
  NRF_CSN_set();
  NRF_CE_reset();

  // Power on reset
  HAL_Delay(100);

  return NRF_PowerUp();
}

NRF_Status NRF_SendCommand(uint8_t cmd) {
  NRF_Status ret = HAL_OK;
  uint8_t status;

  NRF_CSN_reset();
  ret = HAL_SPI_TransmitReceive(hspi, &cmd, &status, 1, NRF_SPI_TIMEOUT);
  if (ret != HAL_OK) {
    return ret;
  }
  NRF_CSN_set();

  return ret;
}

NRF_Status NRF_WriteRegister(uint8_t reg, uint8_t *write, uint8_t length) {
  NRF_Status ret = HAL_OK;
  uint8_t status;
  uint8_t cmd = NRF_CMD_WRITE | reg;

  NRF_CSN_reset();
  ret = HAL_SPI_TransmitReceive(hspi, &cmd, &status, 1, NRF_SPI_TIMEOUT);
  if (ret != HAL_OK) {
    return ret;
  }
  ret = HAL_SPI_Transmit(hspi, write, length, NRF_SPI_TIMEOUT);
  if (ret != HAL_OK) {
    return ret;
  }
  NRF_CSN_set();

  return ret;
}

NRF_Status NRF_ReadRegister(uint8_t reg, uint8_t *read, uint8_t length) {
  NRF_Status ret = HAL_OK;
  uint8_t status;
  uint8_t cmd = NRF_CMD_READ | reg;

  NRF_CSN_reset();
  ret = HAL_SPI_TransmitReceive(hspi, &cmd, &status, 1, NRF_SPI_TIMEOUT);
  if(ret != HAL_OK) {
    return ret;
  }
  ret = HAL_SPI_Receive(hspi, read, length, NRF_SPI_TIMEOUT);
  if(ret != HAL_OK) {
    return ret;
  }
  NRF_CSN_set();

  return ret;
}

// Verifies SPI by setting the TX address and reading
// it from the device.
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

/*
 * Setters
 */

NRF_Status NRF_ToggleBitReg(uint8_t reg, uint8_t bit) {
  NRF_Status ret = HAL_OK;
  uint8_t cfg = 0x00;

  ret = NRF_ReadRegister(reg, &cfg, 1);
  if (ret != HAL_OK) {
    return ret;
  }
  cfg = cfg ^ (1 << bit);
  return NRF_WriteRegister(reg, &cfg, 1);
}

NRF_Status NRF_WriteBitReg(uint8_t reg, uint8_t bit) {
  NRF_Status ret = HAL_OK;
  uint8_t cfg = 0x00;

  ret = NRF_ReadRegister(reg, &cfg, 1);
  if (ret != HAL_OK) {
    return ret;
  }
  cfg = cfg & (1 << bit);
  return NRF_WriteRegister(reg, &cfg, 1);
}

// According to figure 3 it takes ~1.5ms to powerup
// should probably be pulling config or status
// to see if we're running.
NRF_Status NRF_PowerUp() {
  NRF_Status ret = HAL_OK;
  uint8_t cfg = 0x00;
  ret = NRF_ReadRegister(NRF_REG_CONFIG, &cfg, 1);
  if (ret != HAL_OK) {
    return ret;
  }
  cfg = cfg|CFG_PWR_UP_BIT;
  ret = NRF_WriteRegister(NRF_REG_CONFIG, &cfg, 1);
  HAL_Delay(2);
  return ret;
}

// See table 15 (p.23)
NRF_Status NRF_SetModeRx() {
  NRF_Status ret = HAL_OK;
  uint8_t cfg = 0x00;
  ret = NRF_ReadRegister(NRF_REG_CONFIG, &cfg, 1);
  if (ret != HAL_OK) {
    return ret;
  }
  cfg = cfg|CFG_PRIM_RX_BIT;
  ret = NRF_WriteRegister(NRF_REG_CONFIG, &cfg, 1);
  HAL_Delay(1);
  return ret;
}

NRF_Status NRF_WritePayload(uint8_t *payload, uint8_t length) {
  NRF_Status ret = HAL_OK;
  uint8_t status;
  uint8_t cmd = NRF_CMD_W_TX_PAYLOAD;

  NRF_CSN_reset();
  ret = HAL_SPI_TransmitReceive(hspi, &cmd, &status, 1, NRF_SPI_TIMEOUT);
  if(ret != HAL_OK) {
    return ret;
  }
  ret = HAL_SPI_Transmit(hspi, payload, length, NRF_SPI_TIMEOUT);
  if(ret != HAL_OK) {
    return ret;
  }
  NRF_CSN_set();

  return ret;
}

NRF_Status NRF_ReadPayload(uint8_t *payload, uint8_t length) {
  NRF_Status ret = HAL_OK;
  uint8_t status;
  uint8_t cmd = NRF_CMD_R_RX_PAYLOAD;

  NRF_CSN_reset();
  ret = HAL_SPI_TransmitReceive(hspi, &cmd, &status, 1, NRF_SPI_TIMEOUT);
  if(ret != HAL_OK) {
    return ret;
  }
  ret = HAL_SPI_Receive(hspi, payload, length, NRF_SPI_TIMEOUT);
  if(ret != HAL_OK) {
    return ret;
  }
  NRF_CSN_set();

  return ret;
}

/*
 * Getters
 */
NRF_Status NRF_GetStatus(uint8_t *status) {
  NRF_Status ret = HAL_OK;
  uint8_t cmd = NRF_CMD_NOP;

  NRF_CSN_reset();
  ret = HAL_SPI_TransmitReceive(hspi, &cmd, status, 1, NRF_SPI_TIMEOUT);
  if(ret != HAL_OK) {
    return ret;
  }
  NRF_CSN_set();

  return ret;
}

/*
 * Misc functions
 */
NRF_Status NRF_PrintStatus() {
  NRF_Status ret = HAL_OK;
  uint8_t status;

  ret = NRF_GetStatus(&status);
  if(ret != HAL_OK) {
    return ret;
  }

  printf("Status register: %02X\r\n", status);
  printf("CE: %d\r\n", HAL_GPIO_ReadPin(NRF_CE_GPIO_Port, NRF_CE_Pin));
  printf("CSN: %d\r\n", HAL_GPIO_ReadPin(NRF_CSN_GPIO_Port, NRF_CSN_Pin));
  printf("TX_FULL:  %1X\r\n", status & STAT_TX_FULL_BIT);
  printf("RX_P_NO:  %1X\r\n", (status & STAT_RX_P_NO_BITS) >> 1);
  printf("MAX_RT:   %1X\r\n", (status & STAT_MAX_RT_BIT) >> 4);
  printf("TX_DS:    %1X\r\n", (status & STAT_TX_DS_BIT) >> 5);
  printf("RX_DR:    %1X\r\n", (status & STAT_RX_DR_BIT) >> 6);
  printf("\r\n");

  return ret;
}

NRF_Status NRF_PrintFIFOStatus() {
  NRF_Status ret = HAL_OK;
  uint8_t status;
  uint8_t reg = NRF_REG_FIFO_STATUS;

  NRF_ReadRegister(reg, &status, 1);

  printf("FIFO status register: %02X\r\n", status);
  printf("RX_EMPTY:   %2X\r\n", status & FIFO_RX_EMPTY);
  printf("RX_FULL:    %2X\r\n", status & FIFO_RX_FULL);
  printf("TX_EMPTY:   %2X\r\n", (status & FIFO_TX_EMPTY) >> 4);
  printf("TX_FULL:    %2X\r\n", status & FIFO_TX_FULL);
  printf("TX_REUSE:   %2X\r\n", status & FIFO_TX_REUSE);
  printf("\r\n");

  return ret;
}
