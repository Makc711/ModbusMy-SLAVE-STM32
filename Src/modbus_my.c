/* Private includes ----------------------------------------------------------*/
#include "modbus_my.h"
#include "usart.h"

/* Private define ------------------------------------------------------------*/
#define RX_BUF_SIZE 3
#define RECEPTION_TIME 10

enum
{
  UART_COMMAND_SEND_DATA      = 0x01,
  UART_COMMAND_UPDATE_OUTPUTS = 0x02,

  UART_COMMAND_ERROR_RESET    = 0x0F
};

enum
{
  ERROR_NONE                   = 0x00,
  ERROR_UART_PARITY            = 0x01,
  ERROR_UART_NOISE             = 0x02,
  ERROR_UART_FRAME             = 0x03,
  ERROR_UART_OVERRUN           = 0x04,
  ERROR_UART_DMA_TRANSFER      = 0x05,
  ERROR_INCORRECT_UART_COMMAND = 0x06
};

/* Private variables ---------------------------------------------------------*/
static bool isWaitFirstByte_ = true;

static union
{
  struct
  {
    uint8_t address : 4;
    uint8_t command : 4;
    bool isLed1On : 1;
    bool isLed2On : 1;
    bool isLed3On : 1;
    bool isLed4On : 1;
    uint8_t reserved : 4;
    uint8_t crc;
  } field;
  uint8_t rxBuf[RX_BUF_SIZE];
} rxData_;
_Static_assert(sizeof(rxData_.rxBuf) == sizeof(rxData_.field), "Error: Structure size does not match RX_BUF_SIZE");

static struct
{
  int16_t temperature;
  uint16_t voltage;
  bool led1State : 1;
  bool led2State : 1;
  bool led3State : 1;
  bool led4State : 1;
  bool button1State : 1;
  uint8_t reserve : 3;
  uint8_t error;
  uint16_t crc;
} txData_;

/* Private function prototypes -----------------------------------------------*/
static uint8_t crc8(uint8_t *data, uint8_t size);
static uint16_t crc16(uint8_t *data, uint8_t size);
static void clearBuffer(uint8_t *data, uint8_t size);
static void turnOnLed1(bool isLedOn);
static void turnOnLed2(bool isLedOn);
static void turnOnLed3(bool isLedOn);
static void turnOnLed4(bool isLedOn);

/* Functions -----------------------------------------------------------------*/
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if (huart == &huart1)
  {
    __HAL_UART_DISABLE_IT(huart, UART_IT_IDLE);
    HAL_UART_DMAStop(huart);
    const uint32_t error = HAL_UART_GetError(huart);

    switch (error)
    {
    case HAL_UART_ERROR_PE:
      txData_.error = ERROR_UART_PARITY;
      __HAL_UART_CLEAR_PEFLAG(huart);
      huart->ErrorCode = HAL_UART_ERROR_NONE;
      break;

    case HAL_UART_ERROR_NE:
      txData_.error = ERROR_UART_NOISE;
      __HAL_UART_CLEAR_NEFLAG(huart);
      huart->ErrorCode = HAL_UART_ERROR_NONE;
      break;

    case HAL_UART_ERROR_FE:
      txData_.error = ERROR_UART_FRAME;
      __HAL_UART_CLEAR_FEFLAG(huart);
      huart->ErrorCode = HAL_UART_ERROR_NONE;
      break;

    case HAL_UART_ERROR_ORE:
      txData_.error = ERROR_UART_OVERRUN;
      __HAL_UART_CLEAR_OREFLAG(huart);
      huart->ErrorCode = HAL_UART_ERROR_NONE;
      break;

    case HAL_UART_ERROR_DMA:
      txData_.error = ERROR_UART_DMA_TRANSFER;
      huart->ErrorCode = HAL_UART_ERROR_NONE;
      break;

    default:
      break;
    }
  }
}

/**
  * @brief This function must be called before the main loop to start receiving
  *        UART messages.
  */
void uartStartReceive(void)
{
  clearBuffer(rxData_.rxBuf, RX_BUF_SIZE);
  isWaitFirstByte_ = true;
  HAL_UART_Receive_IT(&huart1, rxData_.rxBuf, 1);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart == &huart1)
  {
    if (isWaitFirstByte_ == true) 
    {
      isWaitFirstByte_ = false;
      HAL_UART_Receive_IT(&huart1, rxData_.rxBuf + 1, RX_BUF_SIZE - 1);
    }
    else
    {
      const bool isAddressCorrect = (rxData_.field.address == DEVICE_ADDRESS);
      const bool isCrcCorrect = (rxData_.field.crc == crc8(rxData_.rxBuf, sizeof(rxData_) - sizeof(rxData_.field.crc)));
      if (isAddressCorrect && isCrcCorrect)
      {
        switch (rxData_.field.command)
        {
        case UART_COMMAND_SEND_DATA:
          break;

        case UART_COMMAND_UPDATE_OUTPUTS:
          turnOnLed1(rxData_.field.isLed1On);
          turnOnLed2(rxData_.field.isLed2On);
          turnOnLed3(rxData_.field.isLed3On);
          turnOnLed4(rxData_.field.isLed4On);
          break;

        case UART_COMMAND_ERROR_RESET:
          txData_.error = ERROR_NONE;
          break;

        default:
          txData_.error = ERROR_INCORRECT_UART_COMMAND;
          break;
        }

        txData_.crc = crc16((uint8_t *)&txData_, sizeof(txData_) - sizeof(txData_.crc));
        HAL_UART_Transmit_IT(&huart1, (uint8_t *)(&txData_), sizeof(txData_));
      }
      uartStartReceive();
    }
  }
}

/**
  * @brief This function must be run in the SysTick_Handler(void) interrupt
  *        handler with a period of 1 ms.
  */
void checkTimeOutReception(void)
{
  static uint8_t timeOut = 0;
  if (isWaitFirstByte_ == true)
  {
    timeOut = 0; 
  }
  else 
  {
    timeOut++;
    if (timeOut >= RECEPTION_TIME) 
    {
      HAL_UART_AbortReceive_IT(&huart1);
      timeOut = 0;
      uartStartReceive();
    }
  }
}

static uint8_t crc8(uint8_t *data, uint8_t size)
{
  uint8_t crc = 0xFF;
  while (size-- > 0)
  {
    crc ^= *data++;
    for (uint8_t i = 0; i < 8; i++)
    {
      crc = ((crc & 0x80) != 0)
              ? (uint8_t)((crc << 1) ^ 0x31)
              : (uint8_t)(crc << 1);
    }
  }
  return crc;
}

static uint16_t crc16(uint8_t *data, uint8_t size)
{
  uint16_t crc = 0xFFFF;
  while (size-- > 0)
  {
    crc ^= *data++ << 8;
    for (uint8_t i = 0; i < 8; i++)
    {
      crc = ((crc & 0x8000) != 0)
              ? (uint16_t)((crc << 1) ^ 0x1021)
              : (uint16_t)(crc << 1);
    }
  }
  return crc;
}

static void clearBuffer(uint8_t *data, uint8_t size)
{
  while (size-- > 0)
  {
    *data++ = 0x00;
  }
}

static void turnOnLed1(bool isLedOn)
{
  HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, !isLedOn);
  txData_.led1State = isLedOn;
}

static void turnOnLed2(bool isLedOn)
{
  HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, !isLedOn);
  txData_.led2State = isLedOn;
}

static void turnOnLed3(bool isLedOn)
{
  HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, !isLedOn);
  txData_.led3State = isLedOn;
}

static void turnOnLed4(bool isLedOn)
{
  HAL_GPIO_WritePin(LED4_GPIO_Port, LED4_Pin, !isLedOn);
  txData_.led4State = isLedOn;
}

void setTemperature(int16_t temperature)
{
  txData_.temperature = temperature;
}

void setVoltage(uint16_t voltage)
{
  txData_.voltage = voltage;
}

void setButton1State(bool buttonState)
{
  txData_.button1State = buttonState;
}
