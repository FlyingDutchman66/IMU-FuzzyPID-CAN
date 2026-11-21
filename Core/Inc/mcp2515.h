/* Minimal MCP2515 driver: HAL SPI, polled, TX buffer 0, standard 11-bit IDs. Main context only:
   never call it from the TIM3 control ISR. */
#ifndef MCP2515_H
#define MCP2515_H

#include "main.h"

/* ---- Hardware map. SPI1 = PA5 SCK (D13) / PA6 MISO (D12) / PA7 MOSI (D11), mode 0, 72 MHz / 16 =
   4.5 MHz: all set in IMU-FuzzyPID-CAN.ioc. CS is the .ioc GPIO label MCP2515_CS = PB6 (D10). ---- */
#define MCP2515_SPI       hspi1
#define MCP2515_CS_PORT   MCP2515_CS_GPIO_Port
#define MCP2515_CS_PIN    MCP2515_CS_Pin

/* Crystal on the module: read the can (8.000 or 16.000). The wrong one still decodes, at 2x the
   bit rate, but no ACK ever gets back: the sender repeats one frame forever. This bench's modules: 16.000. */
// #define MCP2515_XTAL_8MHZ
#define MCP2515_XTAL_16MHZ

HAL_StatusTypeDef MCP2515_Init(void);             // reset + 500 kbps timing, stays in config mode
HAL_StatusTypeDef MCP2515_SetLoopbackMode(void);  // both return HAL_ERROR if CANSTAT doesn't confirm within 10 ms
HAL_StatusTypeDef MCP2515_SetNormalMode(void);
HAL_StatusTypeDef MCP2515_SelfTest(void);         // loopback round trip, ends in normal mode
HAL_StatusTypeDef MCP2515_Send(uint16_t id, const uint8_t *data, uint8_t len);  // HAL_BUSY: previous frame still pending

#endif
