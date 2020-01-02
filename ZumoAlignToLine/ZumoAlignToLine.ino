/*
 * Zumo Align To Line
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
const uint16_t forwardSpeed = 125;

// The speed that the robot uses when aligning to the line
const uint16_t alignSpeed = 75;

const unsigned long LoopDelay = 100UL;

uint16_t lineSensorValues[3] = { 0, 0, 0 };

const unsigned int SensorLeft = 0;
const unsigned int SensorCenter = 1;
const unsigned int SensorRight = 2;

const byte LeftDetect = 0x04;
const byte CenterDetect = 0x02;
const byte RightDetect = 0x01;
const byte NoDetect = 0x00;

const unsigned int SensorThreshold = 500;

enum Move {Forward, Left, Right, Stop};

Move MoveTable[8];

void initMoveTable() {
  MoveTable[NoDetect]                                = Forward;  // 0
  MoveTable[RightDetect]                             = Right;    // 1
  MoveTable[CenterDetect]                            = Forward;  // 2
  MoveTable[CenterDetect | RightDetect]              = Right;    // 3
  MoveTable[LeftDetect]                              = Left;     // 4
  MoveTable[LeftDetect | RightDetect]                = Forward;  // 5
  MoveTable[LeftDetect | CenterDetect]               = Left;     // 6
  MoveTable[LeftDetect | CenterDetect | RightDetect] = Stop;     // 7
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
  Move nextMove;

  if (now >= loopTimer) {
    loopTimer = now + LoopDelay;
    lineSensors.readCalibrated(lineSensorValues);
    nextMove = MoveTable[decodeSensor()];
    lcd.gotoXY(0, 0);
    switch (nextMove) {
      case Forward:
        motors.setSpeeds(forwardSpeed, forwardSpeed);
        lcd.print(F("Fwd  "));
        break;
      case Left:
        motors.setSpeeds(0, alignSpeed);
        lcd.print(F("Left "));
        break;
      case Right:
        motors.setSpeeds(alignSpeed, 0);
        lcd.print(F("Right"));
        break;
      case Stop:
      default:
        motors.setSpeeds(0, 0);
        lcd.print(F("Stop "));
        break;
    }
    
  }
  
}
