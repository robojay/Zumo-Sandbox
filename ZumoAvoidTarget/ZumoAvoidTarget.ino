/*
 * Zumo Avoid Target
 * 
 * Use the active proximity sensors to turn away from a target
 * 
 */

#include <Wire.h>
#include <Zumo32U4.h>

Zumo32U4Motors motors;
Zumo32U4ProximitySensors proxSensors;

const byte LeftDetect = 0x08;
const byte LeftCenterDetect = 0x04;
const byte RightCenterDetect = 0x02;
const byte RightDetect = 0x01;
const byte NoDetect = 0x00;

const unsigned int LeftMotorIndex = 1;
const unsigned int RightMotorIndex = 0;
const unsigned int DelayIndex = 2;

uint16_t MoveTable[16][3];

void defineMove(unsigned int index, uint16_t left, uint16_t right, uint16_t delay) {
  MoveTable[index][RightMotorIndex] = right;
  MoveTable[index][LeftMotorIndex] = left;
  MoveTable[index][DelayIndex] = delay;
}

void initMoveTable() {
  // LLRR   , Left, Right, Delay
  //  CC
  defineMove(0b0000,    0,    0, 10);
  defineMove(0b0001,  200,  400, 10);
  defineMove(0b0010,  200,  400, 10);
  defineMove(0b0011,  200,  400, 10);
  defineMove(0b0100,  400,  200, 10);
  defineMove(0b0101,  400,  200, 10);
  defineMove(0b0110,  400,  200, 10);
  defineMove(0b0111,  200,  400, 10);
  defineMove(0b1000,  400,  200, 10);
  defineMove(0b1001,  400,  400, 10);
  defineMove(0b1010,  200,  400, 10);
  defineMove(0b1011,  200,  400, 10);
  defineMove(0b1100,  400,  200, 10);
  defineMove(0b1101,  400,  200, 10);
  defineMove(0b1110,  400,  200, 10);
  defineMove(0b1111,  400, -400, 10);
}

byte decodeSensor() {
  byte result = 0;
  
  // converts proximity sensor readings to a value
  // between 0 and 7, bit mapped as
  // 2 1 0
  // | | |
  // | | +-- Right
  // | +---- Center
  // +------ Left


  proxSensors.read();

  result |= (proxSensors.countsLeftWithLeftLeds() > 0) ? LeftDetect : 0;
  result |= (proxSensors.countsFrontWithLeftLeds() > 0) ? LeftCenterDetect : 0;
  result |= (proxSensors.countsFrontWithRightLeds() > 0) ? RightCenterDetect : 0;
  result |= (proxSensors.countsRightWithRightLeds() > 0) ? RightDetect : 0;
  
  return(result);
}

void setup() {
  proxSensors.initThreeSensors();
  // In a small ring, using low settings seems to work best
  // otherwise, the robot will respond to objects or people outside
  // the ring.
  uint16_t levels[] = {4, 15}; //, 32, 55, 85, 120 };
  proxSensors.setBrightnessLevels(levels, sizeof(levels)/2);

  motors.setSpeeds(0, 0);

  initMoveTable();
}

void loop() {
  static unsigned long loopTimer = 0;
  unsigned long now = millis();
  unsigned int move;

  if (now >= loopTimer) {
    move = decodeSensor();
    motors.setSpeeds(MoveTable[move][LeftMotorIndex], MoveTable[move][RightMotorIndex]);
    loopTimer = now + (unsigned long)MoveTable[move][DelayIndex];
  }  
}
