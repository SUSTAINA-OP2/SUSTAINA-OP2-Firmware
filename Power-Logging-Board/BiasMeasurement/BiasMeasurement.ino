#include "./src/INA226/INA226.h"
#include <vector>
#include "Wire.h"


constexpr uint32_t serial1Baudrate = 1000000;
std::vector<INA226> INA;

constexpr uint8_t lowLimit_Address = 0b1000000;   //! 0x40
constexpr uint8_t upperLimit_Address = 0b1001111; //! 0x4F

void initializeSerial(const uint32_t &serialBaudrate)
{
  Serial.begin(serialBaudrate);
  while (!Serial)
  {
    ; //! wait for serial port to connect. Needed for native USB
  }
}

//! get address of connected INA226
std::vector<uint8_t> readable_Addresses; //! readable INA226 addresses
void I2cScanner()
{
  uint8_t error = 0;
  readable_Addresses.clear();
  for (uint8_t address = lowLimit_Address; address < upperLimit_Address; address++)
  {
    Wire.beginTransmission(address);
    error = Wire.endTransmission();
    if (error == 0)
    {
      readable_Addresses.push_back(address);
    }
  }
}

void setupINA226s()
{
  INA.clear();
  for (size_t i = 0; i < readable_Addresses.size(); i++)
  {
    uint8_t address = readable_Addresses.at(i);
    INA.emplace_back(address);

    if (INA.back().begin())
    {
      INA.back().setMaxCurrentShunt(38.73, 0.002);
      INA.back().setShuntVoltageConversionTime(4); 
      INA.back().setAverage(2);  // センサ値のsample数を決定する ここでのsample数は16
    }
  }
}


void setup()
{
  initializeSerial(serial1Baudrate);
  Wire.begin();
  Wire.setClock(4000000L); //! I2C Set clock change 100kHz to 400kHz

  I2cScanner();
  setupINA226s();
}


void loop()
{
  int inkey;
  if (Serial.available() > 0) {
    inkey = Serial.read();
    switch (inkey) {
      case 'a':
        for (auto& ina_sensor : INA)
        {
          if (ina_sensor.begin())
          {
            Serial.println(ina_sensor.getAddress(), HEX);

            for (int i = 0; i < 5; i++)
            {
              Serial.print(ina_sensor.getCurrent_mA(), 3);
              Serial.println();
              delay(1000);
            }
            Serial.println();
          }
        }
        break;

    }
  }
}
