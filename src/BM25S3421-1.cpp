/*****************************************************************
File:           BM22S3421-1.cpp
Author:         BESTMODULES
Description:    None
History：       None
V1.0.1   -- initial version; 2023-09-21; Arduino IDE : v1.8.19
******************************************************************/
#include "BM25S3421-1.h"

/**********************************************************
Description: Constructor
Parameters: statusPin: Status pin connection with Arduino or BMduino
            *theSerial: Serial object, if your board has multiple UART interfaces
Return: None
Others: None
**********************************************************/
BM25S3421_1::BM25S3421_1(uint8_t statusPin, HardwareSerial *theSerial)
{
  _softSerial = NULL;
  _statusPin = statusPin;
  _hardSerial = theSerial;
}

/**********************************************************
Description:  Constructor
Parameters: statusPin: Status pin connection with Arduino or BMduino
            rxPin : Receiver pin of the UART
            txPin : Send signal pin of UART
Return: None
Others: None
**********************************************************/
BM25S3421_1::BM25S3421_1(uint8_t statusPin, uint8_t rxPin, uint8_t txPin)
{
  _hardSerial = NULL;
  _statusPin = statusPin;
  _rxPin = rxPin;
  _txPin = txPin;
  _softSerial = new SoftwareSerial(_rxPin, _txPin);
}

/**********************************************************
Description: Module initial
Parameters: None
Return: Void
Others: None
**********************************************************/
void BM25S3421_1::begin()
{
  if (_softSerial != NULL)
  {
    _softSerial->begin(BAUDRATE); // baud rate:9600
  }
  if (_hardSerial != NULL)
  {
    _hardSerial->begin(BAUDRATE); // baud rate:9600
  }
  pinMode(_statusPin, INPUT);
}

/**********************************************************
Description: Get STATUS pin level
Parameters: None
Return: STATUS pin level(HIGH/LOW)
Others: None
**********************************************************/
uint8_t BM25S3421_1::getSTATUS()
{
  return digitalRead(_statusPin);
}

/**********************************************************
Description: Read the current VOC concentration level
Parameters: None
Return: 1:Level1; 2:Level2; 3:Level3; 4:Level4
Others: None
**********************************************************/
uint8_t BM25S3421_1::readVOCLevel()
{
  return readParam(0xD2, 0x42);
}

/**********************************************************
Description: Query VOC concentration AD value
Parameters: None
Return: VOC concentration AD value(12-bit)
Others: None
**********************************************************/
uint16_t BM25S3421_1::readADValue()
{
  uint16_t ADValue = 0, ADValueH = 0, ADValueL = 0;
  ADValueH = readParam(0xD2, 0x40);
  ADValueL = readParam(0xD2, 0x41);
  ADValue = (ADValueH << 8) + ADValueL;
  return ADValue;
}

/**********************************************************
Description: Read parameter from module
Parameters: cmd: Command code(8-bit)
            addr: Address code(8-bit)
Return: Module parameter
Others: 1. Consult the command table of the BM22S3421-1 datasheet
        2. Not applicable to "Common Command"
**********************************************************/
uint8_t BM25S3421_1::readParam(uint8_t cmd, uint8_t addr)
{
  uint8_t retFlag;
  uint8_t sendBuf[4] = {cmd, addr, 0x00, 0x00};
  uint8_t recBuf[8] = {0};
  sendBuf[3] = ~(sendBuf[0] + sendBuf[1] + sendBuf[2]) + 1;
  writeBytes(sendBuf, 4);
  delay(50); // Wait for the module to reply data
  if (readBytes(recBuf, 8) == CHECK_OK)
  {
    if (recBuf[4] == cmd && recBuf[5] == addr)
    {
      return recBuf[6];
    }
    else
    {
      return 0;
    }
  }
  else
  {
    return 0;
  }
}

/**********************************************************
Description: Query whether the 14-byte data automatically uploaded by the module is received
Parameters: None
Return: true(1): Received
        false(0): Not received
Others: Only used in module automatic upload mode
**********************************************************/
bool BM25S3421_1::isInfoAvailable()
{
  uint8_t header[5] = {0xAA, 0x0E, 0x41, 0x01, 0xAC}, headerLen = 5; // Fixed code for first 5 bytes of 14-byte data
  uint8_t recBuf[14] = {0}, recLen = 14;
  uint8_t i, num = 0, readCnt = 0, failCnt = 0;
  uint8_t checkSum = 0;
  bool isHeader = false, result = false;

  for (i = 0; i < recLen; i++)
  {
    _receiveBuffer[i] = 0;
  }

  /* Select hardSerial or softSerial according to the setting */
  if (_softSerial != NULL)
  {
    num = _softSerial->available();
  }
  else if (_hardSerial != NULL)
  {
    num = _hardSerial->available();
  }

  /* Serial buffer contains at least one 14-byte data */
  if (num >= recLen)
  {
    while (failCnt < 3) // Didn't read the required data twice, exiting the loop
    {
      /* Find 5-byte data header */
      for (i = 0; i < headerLen;)
      {
        if (_softSerial != NULL)
        {
          recBuf[i] = _softSerial->read();
        }
        else if (_hardSerial != NULL)
        {
          recBuf[i] = _hardSerial->read();
        }
        if (recBuf[i] == header[i])
        {
          isHeader = true; // Fixed code is correct
          i++;             // Next byte
        }
        else if ((recBuf[i] != header[i]) && (i > 0))
        {
          isHeader = false; // Next fixed code error
          failCnt++;
          break;
        }
        else if ((recBuf[i] != header[i]) && (i == 0))
        {
          readCnt++; // First byte "0XAA" not found, continue
        }
        if (readCnt > (num - recLen)) // 14-byte data not found
        {
          return false;
        }
      }

      /* Find the correct fixed code */
      if (isHeader)
      {
        for (checkSum = 0, i = 0; i < headerLen; i++)
        {
          checkSum += recBuf[i]; // Sum checkSum
        }
        for (i = headerLen; i < recLen; i++) // Read subsequent 10-byte data
        {
          if (_softSerial != NULL)
          {
            recBuf[i] = _softSerial->read();
          }
          else if (_hardSerial != NULL)
          {
            recBuf[i] = _hardSerial->read();
          }
          checkSum += recBuf[i]; // Sum checkSum
        }

        /* Calculate checkSum */
        checkSum = checkSum - recBuf[recLen - 1];
        checkSum = (~checkSum) + 1;

        /* Compare whether the check code is correct */
        if (checkSum == recBuf[recLen - 1])
        {
          for (i = 0; i < recLen; i++)
          {
            _receiveBuffer[i] = recBuf[i]; // True, assign data to _recBuf[]
          }
          result = true;
          break; // Exit "while (failCnt < 3)" loop
        }
        else
        {
          failCnt++; // Error, failCnt++, return "while (failCnt < 3)" loop
        }
      }
    }
  }
  return result;
}

/**********************************************************
Description: Read the 14-byte data of sent by the module
Parameters: array[]: The array for storing the 14-byte module information
                     (refer to datasheet for meaning of each bit)
Return: Void
Others: Use after "isInfoAvailable() == true"
**********************************************************/
void BM25S3421_1::readInfoPackage(uint8_t array[])
{
  for (uint8_t i = 0; i < 14; i++)
  {
    array[i] = _receiveBuffer[i];
    _receiveBuffer[i] = 0;
  }
}

/**********************************************************
Description: Write command to module
Parameters: cmd: Command code(8-bit)
            addr: Address code(8-bit)
            param: Parameters to be written(8-bit)
Return: 0: Write Successful
        1: Check error
        2: Timeout error
Others: 1. Consult the command table of the BM22S3021-1 datasheet
        2. Not applicable to "Common Command"
**********************************************************/
uint8_t BM25S3421_1::writeCommand(uint8_t cmd, uint8_t addr, uint8_t param)
{
  uint8_t sendBuf[4] = {cmd, addr, param, 0x00};
  uint8_t recBuf[8] = {0};
  sendBuf[3] = ~(sendBuf[0] + sendBuf[1] + sendBuf[2]) + 1;
  writeBytes(sendBuf, 4);
  delay(50); // Wait for the module to reply data
  return readBytes(recBuf, 8);
}

/**********************************************************
Description: Software reset module
Parameters: None
Return: 0: Reset ok
        1: Check error
        2: Timeout error
Others: After the command is executed, the module needs to be preheated again
**********************************************************/
uint8_t BM25S3421_1::reset()
{
  uint8_t retFlag;
  uint8_t sendBuf[4] = {0xAF, 0x00, 0x00, 0x51}; // Used to store CMD to be sent
  uint8_t recBuf[8];
  writeBytes(sendBuf, 4);
  delay(50); // Wait for the module to reply data
  retFlag = readBytes(recBuf, 8);
  if (retFlag == CHECK_OK)
  {
    if ((recBuf[0] == 0xAF) && (recBuf[1] == 0x00))
    {
      return CHECK_OK;
    }
    else
    {
      return CHECK_ERROR;
    }
  }
  else
  {
    return retFlag;
  }
}

/**********************************************************
Description: Get the current status and data of the module
Parameters: array: The array for storing the 14-byte module information
                  (refer to datasheet for meaning of each bit)
Return: 0: Request ok
        1: Check error
        2: Timeout error
Others: None
**********************************************************/
uint8_t BM25S3421_1::requestInfoPackage(uint8_t array[])
{
  uint8_t i, retFlag;
  uint8_t sendBuf[4] = {0xAC, 0x00, 0x00, 0x54};
  uint8_t recBuf[18];
  writeBytes(sendBuf, 8);
  delay(50); // Wait for the module to reply data
  retFlag = readBytes(recBuf, 14);
  if (retFlag == CHECK_OK)
  {
    for (i = 0; i < 14; i++)
    {
      array[i] = recBuf[i];
    }
  }
  else
  {
    for (i = 0; i < 14; i++)
    {
      array[i] = 0;
    }
  }
  return retFlag;
}

/**********************************************************
Description: Restore module parameters to factory default values
Parameters: None
Return: 0: Restore ok
        1: Check error
        2: Timeout error
Others: 1.After the command is executed, the module needs to be preheated again.
        2.Restore factory settings.
          After sending this command, all parameter configurations are restored to factory settings.
        3.Factory set data description:
          Alarm level: Level3;
          Alarm exit level: Level1;
          Preheating time: 180s.
**********************************************************/
uint8_t BM25S3421_1::restoreDefault()
{
  uint8_t sendBuf[4] = {0xA0, 0x00, 0x00, 0x60};
  uint8_t recBuf[8];
  writeBytes(sendBuf, 4);
  delay(50); // Waiting for module to receive data and reply
  return readBytes(recBuf, 8);
}

/**********************************************************
Description: Calibrate module
Parameters: calibrateParam: Refer to BM22S3421-1 data sheet
Return: 0: Write Successful
        1: Check error
        2: Timeout error
Others: Refer to BM22S3421-1 datasheet
**********************************************************/
uint8_t BM25S3421_1::calibrateModule(uint8_t calibrateParam)
{
  return writeCommand(0xAB, calibrateParam, 0x00);
}

/**********************************************************
Description: Clear UART Receive FIFO
Parameters: None
Return: Void
Others: None
**********************************************************/
void BM25S3421_1::clear_UART_FIFO()
{
  if (_softSerial != NULL)
  {
    while (_softSerial->available() > 0)
    {
      _softSerial->read();
    }
  }
  else
  {
    while (_hardSerial->available() > 0)
    {
      _hardSerial->read();
    }
  }
}
/**********************************************************
Description: Write data through UART
Parameters: wBuf: The array for storing data to be sent(9 bytes)
            wLen:Length of data sent
Return: Void
Others: None
**********************************************************/
void BM25S3421_1::writeBytes(uint8_t wbuf[], uint8_t wLen)
{
  clear_UART_FIFO();
  /*Select hardSerial or softSerial according to the setting*/
  if (_softSerial != NULL)
  {
    _softSerial->write(wbuf, wLen);
    _softSerial->flush(); // Wait for the end of serial port data transmission
  }
  else
  {
    _hardSerial->write(wbuf, wLen);
    _hardSerial->flush(); // Wait for the end of serial port data transmission
  }
}

/**********************************************************
Description: Read data through UART
Parameters: rBuf: The array for storing Data to be sent
            rlen: Length of data to be read
            timeout: Receive timeout(unit: ms)
Return: 0: Verification succeeded
        1: Verification failed
        2: Timeout error
Others: None
**********************************************************/
uint8_t BM25S3421_1::readBytes(uint8_t rBuf[], uint8_t rLen, uint16_t timeout)
{
  uint16_t delayCnt = 0;
  uint8_t i = 0, checkSum = 0;

  /* Select SoftwareSerial Interface */
  if (_softSerial != NULL)
  {
    for (i = 0; i < rLen; i++)
    {
      delayCnt = 0;
      while (_softSerial->available() == 0)
      {
        if (delayCnt > timeout)
        {
          return TIMEOUT_ERROR; // Timeout error
        }
        delay(1); // delay 1ms
        delayCnt++;
      }
      rBuf[i] = _softSerial->read();
    }
  }
  /* Select HardwareSerial Interface */
  if (_hardSerial != NULL)
  {
    for (i = 0; i < rLen; i++)
    {
      delayCnt = 0;
      while (_hardSerial->available() == 0)
      {
        if (delayCnt > timeout)
        {
          return TIMEOUT_ERROR; // Timeout error
        }
        delay(1);
        delayCnt++;
      }
      rBuf[i] = _hardSerial->read();
    }
  }

  /* check Sum */
  for (i = 0; i < (rLen - 1); i++)
  {
    checkSum += rBuf[i];
  }
  checkSum = ~checkSum + 1;
  if (checkSum == rBuf[rLen - 1])
  {
    return CHECK_OK; // Check correct
  }
  else
  {
    return CHECK_ERROR; // Check error
  }
}
