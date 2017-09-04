/*
 *  DHT Library for  Digital-output Humidity and Temperature sensors
 *
 *  Works with DHT11, DHT22
 *             SEN11301P,  Grove - Temperature&Humidity Sensor     (Seeed Studio)
 *             SEN51035P,  Grove - Temperature&Humidity Sensor Pro (Seeed Studio)
 *             AM2302   ,  temperature-humidity sensor
 *             HM2303   ,  Digital-output humidity and temperature sensor
 *
 *  Copyright (C) Wim De Roeve
 *                based on DHT22 sensor library by HO WING KIT
 *                Arduino DHT11 library
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documnetation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to  whom the Software is
 * furished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS OR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#include "DHT.h"

#define DHT_DATA_BIT_COUNT 40

DHT::DHT(PinName pin, eType DHTtype)
{
  _pin = pin;
  _DHTtype = DHTtype;
  _firsttime = true;
}

DHT::~DHT()
{

}

eError DHT::stall(DigitalInOut &io, int const level, int const max_time)
{
  int cnt = 0;
  while (level == io)
  {
    if (cnt > max_time)
    {
      return ERROR_NO_PATIENCE;
    }
    cnt++;
    wait_us(1);
  }
  return ERROR_NONE;
}

eError DHT::readData()
{
  uint8_t i = 0, j = 0, b = 0, data_valid = 0;
  uint32_t bit_value[DHT_DATA_BIT_COUNT] = {0};

  eError err = ERROR_NONE;
  //time_t currentTime = time(NULL);

  DigitalInOut DHT_io(_pin);

  // IO must be in hi state to start
  if (ERROR_NONE != stall(DHT_io, 0, 250))
  {
    return BUS_BUSY;
  }

  // start the transfer
  DHT_io.output();
  DHT_io = 0;
  // only 500uS for DHT22 but 18ms for DHT11
  (_DHTtype == 22) ? wait_ms(18) : wait(1);
  DHT_io = 1;
  wait_us(30);
  DHT_io.input();
  // wait till the sensor grabs the bus
  if (ERROR_NONE != stall(DHT_io, 1, 40))
  {
    return ERROR_NOT_PRESENT;
  }
  // sensor should signal low 80us and then hi 80us
  if (ERROR_NONE != stall(DHT_io, 0, 100))
  {
    return ERROR_SYNC_TIMEOUT;
  }
  if (ERROR_NONE != stall(DHT_io, 1, 100))
  {
    return ERROR_NO_PATIENCE;
  }
  // capture the data
  for (i = 0; i < 5; i++)
  {
    for (j = 0; j < 8; j++)
    {
      if (ERROR_NONE != stall(DHT_io, 0, 75))
      {
        return ERROR_DATA_TIMEOUT;
      }
      // logic 0 is 28us max, 1 is 70us
      wait_us(40);
      bit_value[i * 8 + j] = DHT_io;
      if (ERROR_NONE != stall(DHT_io, 1, 50))
      {
        return ERROR_DATA_TIMEOUT;
      }
    }
  }
  // store the data
  for (i = 0; i < 5; i++)
  {
    b = 0;
    for (j = 0; j < 8; j++)
    {
      if (bit_value[i * 8 + j] == 1)
      {
        b |= (1 << (7 - j));
      }
    }
    DHT_data[i] = b;
  }

  // uncomment to see the checksum error if it exists
  //printf(" 0x%02x + 0x%02x + 0x%02x + 0x%02x = 0x%02x \n", DHT_data[0], DHT_data[1], DHT_data[2], DHT_data[3], DHT_data[4]);
  data_valid = DHT_data[0] + DHT_data[1] + DHT_data[2] + DHT_data[3];
  if (DHT_data[4] == data_valid)
  {
    //_lastReadTime = currentTime;
    _lastTemperature = CalcTemperature();
    _lastHumidity = CalcHumidity();

  }
  else
  {
    err = ERROR_CHECKSUM;
  }

  return err;

}

float DHT::CalcTemperature()
{
  int v;

  switch (_DHTtype)
  {
  case DHT11:
    v = DHT_data[2];
    return float(v);
  case DHT22:
    v = DHT_data[2] & 0x7F;
    v *= 256;
    v += DHT_data[3];
    v /= 10;
    if (DHT_data[2] & 0x80)
      v *= -1;
    return float(v);
  }
  return 0;
}

float DHT::ReadHumidity()
{
  return _lastHumidity;
}

float DHT::ConvertCelciustoFarenheit(float const celsius)
{
  return celsius * 9 / 5 + 32;
}

float DHT::ConvertCelciustoKelvin(float const celsius)
{
  return celsius + 273.15;
}

// dewPoint function NOAA
// reference: http://wahiduddin.net/calc/density_algorithms.htm
float DHT::CalcdewPoint(float const celsius, float const humidity)
{
  float A0 = 373.15 / (273.15 + celsius);
  float SUM = -7.90298 * (A0 - 1);
  SUM += 5.02808 * log10(A0);
  SUM += -1.3816e-7 * (pow(10, (11.344 * (1 - 1 / A0))) - 1) ;
  SUM += 8.1328e-3 * (pow(10, (-3.49149 * (A0 - 1))) - 1) ;
  SUM += log10(1013.246);
  float VP = pow(10, SUM - 3) * humidity;
  float T = log(VP / 0.61078); // temp var
  return (241.88 * T) / (17.558 - T);
}

// delta max = 0.6544 wrt dewPoint()
// 5x faster than dewPoint()
// reference: http://en.wikipedia.org/wiki/Dew_point
float DHT::CalcdewPointFast(float const celsius, float const humidity)
{
  float a = 17.271;
  float b = 237.7;
  float temp = (a * celsius) / (b + celsius) + log(humidity / 100);
  float Td = (b * temp) / (a - temp);
  return Td;
}

float DHT::ReadTemperature(eScale Scale)
{
  if (Scale == FARENHEIT)
    return ConvertCelciustoFarenheit(_lastTemperature);
  else if (Scale == KELVIN)
    return ConvertCelciustoKelvin(_lastTemperature);
  else
    return _lastTemperature;
}

float DHT::CalcHumidity()
{
  int v;

  switch (_DHTtype)
  {
  case DHT11:
    v = DHT_data[0];
    return float(v);
  case DHT22:
    v = DHT_data[0];
    v *= 256;
    v += DHT_data[1];
    v /= 10;
    return float(v);
  }
  return 0;
}


