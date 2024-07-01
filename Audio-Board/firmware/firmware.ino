/***************************************************
Firmware for SUSTAINA-Audio-Board to be installed in SUSTAINA-OP2™
  <https://github.com/SUSTAINA-OP/SUSTAINA-OP2>
  <https://github.com/SUSTAINA-OP/SUSTAINA-OP2-Electronics>
  <https://github.com/SUSTAINA-OP/SUSTAINA-OP2-Firmware>
 ****************************************************/

/***************************************************
 DFPlayer - A Mini MP3 Player For Arduino
 <https://www.dfrobot.com/product-1121.html>
 
 ***************************************************
 This example shows the basic function of library for DFPlayer.
 
 Created 2016-12-07
 Modified 2018-08-15
 By [Angelo qiao](Angelo.qiao@dfrobot.com)
 
 GNU Lesser General Public License.
 See <http://www.gnu.org/licenses/> for details.
 All above must be included in any redistribution
 ****************************************************/

/***********Notice and Trouble shooting***************
 1.Connection and Diagram can be found here
 <https://www.dfrobot.com/wiki/index.php/DFPlayer_Mini_SKU:DFR0299#Connection_Diagram>
 2.This code is tested on Arduino Uno, Leonardo, Mega boards.
 ****************************************************/

#include "Arduino.h"
#include "src/DFRobotDFPlayerMini/DFRobotDFPlayerMini.h"

#define FPSerial Serial1

DFRobotDFPlayerMini myDFPlayer;
void printDetail(uint8_t type, int value);

void setup() {
  FPSerial.begin(9600, SERIAL_8N1, /*rx =*/D4, /*tx =*/D3);

  Serial.begin(115200);

  if (!myDFPlayer.begin(FPSerial, /*isACK = */ true, /*doReset = */ true)) {  //Use serial to communicate with mp3.
    while (true) {
      delay(0);  // Code to compatible with ESP8266 watch dog.
    }
  }

  myDFPlayer.volume(10);  //Set volume value. From 0 to 30
  myDFPlayer.play(1);     //Play the first mp3
}

void loop() {

  if (Serial.available()) {
    int16_t receivedCommand = Serial.parseInt();
    myDFPlayer.play(receivedCommand);
  }
}