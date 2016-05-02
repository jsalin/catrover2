/**
 * Pass traffic between PC and HC-05 bluetooth module to configure it manually
 * Copyright (C) 2016 Jussi Salin <salinjus@gmail.com>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 * 
 * Available AT commands of HC-05:
 *
 * AT+UART=<baudrate>,1,0   Set baud rate by direct value and N-8-1.
 *                          Note that 57600bps is maximum of SoftSerial.
 * AT+NAME=<short text>     Set name for discovery mode list of other devices
 * AT+VERSION?              Print version of the module
 * AT+PWSD=<four number>    Set pin code needed to pair to the device
 * AT+ORGL                  Restore default values
 * 
 * Be sure to have New Line and Carriage Return both enabled on your terminal.
 * Connect VDD to HC-05 only after powering up Arduino and the "Ready" text.
 */
 
#include <SoftwareSerial.h>

SoftwareSerial mySerial(4, 3);

#define PC_BPS 38400
#define HC_BPS 38400 // Default speed in command mode

void setup()
{
  // Use some pin of Arduino to set KEY pin of HC-05 high so it goes to programming mode
  pinMode(11, OUTPUT);
  digitalWrite(11, HIGH);

  mySerial.begin(HC_BPS);

  // Connection to PC over main UART port
  Serial.begin(PC_BPS);
  Serial.println("Ready.");
}

void loop()
{
  // Pass traffic between the two serial ports
  if (mySerial.available()) Serial.write(mySerial.read());
  if (Serial.available()) mySerial.write(Serial.read());
}
