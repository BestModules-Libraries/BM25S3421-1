/*************************************************
File:         readVOCLevelAndADValue
Description:  Receive the information automatically output by the module every second,
               and prints the VOC level and VOC A/D value to the serial port monitor
Note:
**************************************************/
#include <BM25S3421-1.h>

uint8_t moduleInfo[14] = {0};
uint16_t ADValue, VOCLevel;

BM25S3421_1 VOC(8, 2, 3); // Softeware serial: 8:STATUS, 2:RX, 3:TX
// BM25S3421_1 VOC(8, &Serial); // 8: STATUS, Hardware serial：Serial
// BM25S3421_1 VOC(STATUS1, &Serial1); // 22(STATUS1): STATUS, Hardware serial: Serial1
// BM25S3421_1 VOC(STATUS2, &Serial2); // 25(STATUS2): STATUS, Hardware serial: Serial2

void setup()
{
  VOC.begin();        // Initialize SW/HW serial, BR: 9600bps, STATUS pin: input mode
  Serial.begin(9600); // Initialize Serial, BR: 9600bps
  Serial.println("Module power on  preheating...(about 3 mins)");
  preheatCountdown(); // Wait for the module to warm up
  Serial.println("End of preheating.");
  Serial.println();
  // VOC.writeCommand(0xe0, 0x1e, AUTO_MODE); // Set the module to automatically output data(default)
}
void loop()
{
  if (VOC.isInfoAvailable() == true) // Scaning the serial port received buffer to receive the information sent by the module
  {
    VOC.readInfoPackage(moduleInfo);
    printInfo(); // Print some information of the module
  }
}

void printInfo()
{
  /*Print VOC level*/
  Serial.print("VOC level: ");
  VOCLevel = moduleInfo[7];
  Serial.println(VOCLevel);

  /*Print VOC A/D Value*/
  Serial.print("VOC A/D Value: ");
  ADValue = ((uint16_t)moduleInfo[5] << 8) + moduleInfo[6];
  Serial.println(ADValue);
  Serial.println();
}
void preheatCountdown()
{
  int16_t time = 180;
  delay(1200);
  if (VOC.isInfoAvailable() == true)
  {
    while (time > 0) // AUTO_MODE
    {
      if (VOC.isInfoAvailable() == true)
      {
        VOC.readInfoPackage(moduleInfo);
        time = moduleInfo[10];
        Serial.print("time:");
        Serial.println(time);
      }
    }
  }
  else
  {
    while (time > 0) // CMD_MODE
    {
      time--;
      delay(1030);
    }
  }
}