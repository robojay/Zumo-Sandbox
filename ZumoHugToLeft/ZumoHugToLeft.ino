/*
 * Zumo hug the line assuming it is on the left
 * 
 * Drive forward until sensing the line, then align to it
 * 
 */

#include <Wire.h>
#include <Zumo32U4.h>

Zumo32U4LCD lcd;
Zumo32U4Motors motors;
Zumo32U4LineSensors lineSensors;

// The speed that the robot uses when driving forward
const uint16_t forwardSpeed = 400;  // 300

// The speed that the robot uses when aligning to the line
const uint16_t alignSpeed = 400;   // 300

const uint16_t backSpeedL = 200;  // 150
const uint16_t backSpeedR = 400;  // 300

const unsigned long LoopDelay = 10UL;

uint16_t lineSensorValues[3] = { 0, 0, 0 };

const unsigned int SensorLeft = 0;
const unsigned int SensorCenter = 1;
const unsigned int SensorRight = 2;

const byte LeftDetect = 0x04;
const byte CenterDetect = 0x02;
const byte RightDetect = 0x01;
const byte NoDetect = 0x00;

const unsigned int LeftMotorIndex = 1;
const unsigned int RightMotorIndex = 0;
const unsigned int DelayIndex = 2;

const unsigned int SensorThreshold = 500;

int16_t MoveTable[8][3];

void defineMove(unsigned int index, uint16_t left, uint16_t right, uint16_t delay) {
  MoveTable[index][RightMotorIndex] = right;
  MoveTable[index][LeftMotorIndex] = left;
  MoveTable[index][DelayIndex] = delay;
}

void WorksinitMoveTable() {
  defineMove(NoDetect, 300, 300, 10);                     // = Forward;  // 0
  defineMove(RightDetect, 0, -300, 10);                   // = Right;     // 1
  defineMove(CenterDetect, -300, -400, 10);               // = Back;      // 2
  defineMove(CenterDetect | RightDetect, -300, -400, 10); // = Back;     // 3
  defineMove(LeftDetect, 0, -300, 10);                    // = Right;    // 4
  defineMove(LeftDetect | RightDetect, 0, -300, 10);      // = Right;    // 5
  defineMove(LeftDetect | CenterDetect, -300, -400, 10);  // = Back;    // 6
  defineMove(LeftDetect | CenterDetect | RightDetect, -300, -400, 10); // = Back;    // 7
}

void initMoveTable() {
  defineMove(NoDetect, 400, 400, 10);                     // = Forward;  // 0
  defineMove(RightDetect, 0, -400, 10);                   // = Right;     // 1
  defineMove(CenterDetect, -300, -400, 250);               // = Back;      // 2
  defineMove(CenterDetect | RightDetect, -300, -400, 250); // = Back;     // 3
  defineMove(LeftDetect, 0, -400, 10);                    // = Right;    // 4
  defineMove(LeftDetect | RightDetect, 0, -400, 10);      // = Right;    // 5
  defineMove(LeftDetect | CenterDetect, -300, -400, 250);  // = Back;    // 6
  defineMove(LeftDetect | CenterDetect | RightDetect, -300, -400, 250); // = Back;    // 7
}

// (from Zumo example code)
// This function calibrates the line sensors for about 10
// seconds.  During this time, you should move the robot around
// manually so that each of its line sensors sees a full black
// surface and a full white surface.  For the best calibration
// results, you should also avoid exposing the sensors to
// abnormal conditions during this time.
void calibrateLineSensors()
{
  // To indicate we are in calibration mode, turn on the yellow LED
  // and print "Line cal" on the LCD.
  ledYellow(1);
  lcd.clear();
  lcd.print(F("Line cal"));

  for (uint16_t i = 0; i < 400; i++)
  {
    lcd.gotoXY(0, 1);
    lcd.print(i);
    lineSensors.calibrate();
  }

  ledYellow(0);
  lcd.clear();
}

byte decodeSensor() {
  byte result = 0;

  // converts thresholded sensor readings to a value
  // between 0 and 7, bit mapped as
  // 2 1 0
  // | | |
  // | | +-- Right
  // | +---- Center
  // +------ Left

  if (lineSensorValues[SensorLeft] < SensorThreshold) {
    result |= LeftDetect;
  }

  if (lineSensorValues[SensorCenter] < SensorThreshold) {
    result |= CenterDetect;
  }
  
  if (lineSensorValues[SensorRight] < SensorThreshold) {
    result |= RightDetect;
  }
  return(result);
}

void setup() {
  motors.setSpeeds(0, 0);
  lineSensors.initThreeSensors();
  calibrateLineSensors();
  initMoveTable();
  lcd.clear();
}

void loop() {
  static unsigned long loopTimer = 0;
  unsigned long now = millis();
  unsigned int move;

  if (now >= loopTimer) {
    lineSensors.readCalibrated(lineSensorValues);
    move = decodeSensor();
    motors.setSpeeds(MoveTable[move][LeftMotorIndex], MoveTable[move][RightMotorIndex]);
    loopTimer = now + (unsigned long)MoveTable[move][DelayIndex];
  }  

}
