/*
    SlimeVR Code is placed under the MIT license
    Copyright (c) 2021 Eiren Rain

    Permission is hereby granted, free of charge, to any person obtaining a copy
    of this software and associated documentation files (the "Software"), to deal
    in the Software without restriction, including without limitation the rights
    to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
    copies of the Software, and to permit persons to whom the Software is
    furnished to do so, subject to the following conditions:

    The above copyright notice and this permission notice shall be included in
    all copies or substantial portions of the Software.

    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
    IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
    FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
    AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
    LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
    OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
    THE SOFTWARE.
*/

#include "Wire.h"
#include "ota.h"
#include "sensors/sensorfactory.h"
#include "configuration.h"
#include "network/network.h"
#include "globals.h"
#include "credentials.h"
#include <i2cscan.h>
#include "serial/serialcommands.h"
#include "batterymonitor.h"
#include "UI\UI.h"



SensorFactory sensors {};
int sensorToCalibrate = -1;
bool blinking = false;
unsigned long blinkStart = 0;
unsigned long loopTime = 0;
unsigned long last_rssi_sample = 0;
bool secondImuActive = false;
BatteryMonitor battery;



void setup()
{

    pinMode(INT_PIN_1, INPUT_PULLUP);
    pinMode(INT_PIN_2, INPUT_PULLUP);




UI::Setup();
UI::DrawSplash();
delay(1500);
UI::MainUIFrame();
UI::SetMessage(1);





    Serial.begin(serialBaudRate);
    SerialCommands::setUp();
    Serial.println();
    Serial.println();
    Serial.println();
    I2CSCAN::clearBus(PIN_IMU_SDA, PIN_IMU_SCL); // Make sure the bus isn't suck when reseting ESP without powering it down
    Wire.begin(PIN_IMU_SDA, PIN_IMU_SCL);
    Wire.setClockStretchLimit(150000L); // Default stretch limit 150mS
    Wire.setClock(I2C_SPEED);

    getConfigPtr();

    delay(500);
    
    sensors.create();
    sensors.motionSetup();
    
    Network::setUp();
    OTA::otaSetup(otaPassword);
    battery.Setup();
    loopTime = micros();
}

void loop()
{

//Serial.println(int2bin(INT_Handler.digitalRead(INT_Handler.eGPA)));

    SerialCommands::update();
    OTA::otaUpdate();
    Network::update(sensors.IMUs);
    sensors.motionLoop();
    sensors.sendData();
    battery.Loop();
    if(micros() - last_rssi_sample >= 2000) 
    {
        last_rssi_sample = micros();
        uint8_t signalStrength = WiFi.RSSI();
        Network::sendSignalStrength(signalStrength);
    }
}


char * int2bin(uint8_t x)
{
  static char buffer[9];
  for (int i=0; i<8; i++) buffer[7-i] = '0' + ((x & (1 << i)) > 0);
  buffer[8] ='\0';
  return buffer;
}