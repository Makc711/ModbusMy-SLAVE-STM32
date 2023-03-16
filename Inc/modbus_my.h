/**
  ******************************************************************************
  * @file           : modbus_my.h
  * @author         : Rusanov M.N.
  * @version        : V1.0.0
  * @date           : 11-March-2023
  * @brief          : Header for modbus_my.c file.
  *                   This file contains functions for working with the UART
  *                   interface over a truncated Modbus RTU protocol.
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef MODBUS_MY_H
#define MODBUS_MY_H

/* Includes ------------------------------------------------------------------*/
#include <stdbool.h>
#include <stdint.h>

/* Exported defines ----------------------------------------------------------*/
#define DEVICE_ADDRESS 0x01
  
#if ((DEVICE_ADDRESS < 0x01) || (DEVICE_ADDRESS > 0x0F))
#error "DEVICE_ADDRESS is out of the allowed range!"
#endif

/* Exported functions prototypes ---------------------------------------------*/
void uartStartReceive(void);
void checkTimeOutReception(void);
void setTemperature(int16_t temperature);
void setVoltage(uint16_t voltage);
void setButton1State(bool buttonState);

#endif // MODBUS_MY_H
