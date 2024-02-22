/*****************************************************************
  File:             BM25S3421-1.h
  Author:           BESTMODULES
  Description:      Define classes and required variables
  Version:          V1.0.1    -- 2023-10-27
******************************************************************/
#ifndef _BM25S3421_1_H_
#define _BM25S3421_1_H_

#include <Arduino.h>
#include <SoftwareSerial.h>

#define BAUDRATE      9600
#define CHECK_OK      0
#define CHECK_ERROR   1
#define TIMEOUT_ERROR 2
#define AUTO_MODE     8 // Set the module to automatically output data: writeCommand(0xe0, 0x1e, AUTO_MODE)
#define CMD_MODE      0 // Set the module not to automatically output data: writeCommand(0xe0, 0x1e, CMD_MODE)
#define HIGH_LEVEL    8 // Set the alarm level to high: writeCommand(0xe0, 0x1f, HIGH_LEVEL)
#define LOW_LEVEL     0 // Set the alarm level to low: writeCommand(0xe0, 0x1f, LOW_LEVEL)
#define LEVEL1        1
#define LEVEL2        2
#define LEVEL3        3
#define LEVEL4        4

class BM25S3421_1
{
public:
  BM25S3421_1(uint8_t statusPin, HardwareSerial *theSerial = &Serial);
  BM25S3421_1(uint8_t statusPin, uint8_t rxPin, uint8_t txPin);
  void begin();
  uint8_t getSTATUS();
  uint8_t readVOCLevel();
  uint16_t readADValue();
  uint8_t readParam(uint8_t cmd, uint8_t addr);
  bool isInfoAvailable();
  void readInfoPackage(uint8_t array[]);
  uint8_t writeCommand(uint8_t cmd, uint8_t addr, uint8_t param);
  uint8_t reset();
  uint8_t requestInfoPackage(uint8_t array[]);
  uint8_t restoreDefault();
  uint8_t calibrateModule(uint8_t calibrateParam);

private:
  uint8_t _statusPin, _rxPin, _txPin;
  uint8_t _receiveBuffer[14]; // Array for storing received data
  void clear_UART_FIFO();
  void writeBytes(uint8_t wBuf[], uint8_t wLen);
  uint8_t readBytes(uint8_t rBuf[], uint8_t rLen, uint16_t timeout = 10);
  HardwareSerial *_hardSerial = NULL;
  SoftwareSerial *_softSerial = NULL;
};

#endif
