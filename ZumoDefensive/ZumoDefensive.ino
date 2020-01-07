/*
 * Zumo hug the line on the left
 * 
 * plus
 * 
 * Use the active proximity sensors to turn away from a target
 * 
 */

#include <Wire.h>
#include <Zumo32U4.h>

Zumo32U4LCD lcd;
Zumo32U4Motors motors;
Zumo32U4ProximitySensors proxSensors;
Zumo32U4LineSensors lineSensors;

bool Debug = false;

uint16_t lineSensorValues[3] = { 0, 0, 0 };
const unsigned int LineSensorLeft = 0;
const unsigned int LineSensorCenter = 1;
const unsigned int LineSensorRight = 2;

const byte LineLeftDetect = 0x04;
const byte LineCenterDetect = 0x02;
const byte LineRightDetect = 0x01;
const byte LineNoDetect = 0x00;

const unsigned int LineSensorThreshold = 500;

const byte ProxLeftDetect = 0x08;
const byte ProxLeftCenterDetect = 0x04;
const byte ProxRightCenterDetect = 0x02;
const byte ProxRightDetect = 0x01;
const byte ProxNoDetect = 0x00;

const int16_t DNC = -999;

const unsigned int LeftMotorIndex = 1;
const unsigned int RightMotorIndex = 0;
const unsigned int DelayIndex = 2;

// should probably be in a structure or object
unsigned long now;
byte proxSensorsNow;
byte lineSensorsNow;
int leftNow;
int rightNow;

int16_t AvoidTable[16][4];

void defineAvoid(unsigned int index, int16_t left, int16_t right, int16_t delay) {
  AvoidTable[index][RightMotorIndex] = right;
  AvoidTable[index][LeftMotorIndex] = left;
  AvoidTable[index][DelayIndex] = delay;
}

void initAvoidTable() {
  // LLRR   , Left, Right, Delay
  //  CC
  defineAvoid(0b0000,  DNC,  DNC, 10);
  defineAvoid(0b0001,  200,  400, 10);
  defineAvoid(0b0010, -400,  400, 10);
  defineAvoid(0b0011, -400,  400, 10);
  defineAvoid(0b0100,  400, -400, 10);
  defineAvoid(0b0101,  400, -400, 10);
  defineAvoid(0b0110,  400, -400, 10);
  defineAvoid(0b0111, -400,  400, 10);
  defineAvoid(0b1000,  400,  200, 10);
  defineAvoid(0b1001,  400,  400, 10);
  defineAvoid(0b1010, -400,  400, 10);
  defineAvoid(0b1011, -400,  400, 10);
  defineAvoid(0b1100,  400, -400, 10);
  defineAvoid(0b1101,  400, -400, 10);
  defineAvoid(0b1110,  400, -400, 10);
  defineAvoid(0b1111,  400, -400, 10);
}

int16_t LineTable[8][3];

void defineLine(unsigned int index, uint16_t left, uint16_t right, uint16_t delay) {
  LineTable[index][RightMotorIndex] = right;
  LineTable[index][LeftMotorIndex] = left;
  LineTable[index][DelayIndex] = delay;
}

void XinitLineTable() {
  // LCR   , Left, Right, Delay
  defineLine(0b000,  DNC,  DNC, 10);                   
  defineLine(0b001, -400,    0, 10);                   
  defineLine(0b010, -400, -400, 250);               
  defineLine(0b011, -400, -300, 250);
  defineLine(0b100,    0, -400, 10);
  defineLine(0b101, -400, -400, 100);
  defineLine(0b110, -300, -400, 250);
  defineLine(0b111, -300, -400, 250);
}

void initLineTable() {
  // LCR   , Left, Right, Delay
  defineLine(0b000,  DNC,  DNC, 10);                   
  defineLine(0b001, -400,    0, 10);                   
  defineLine(0b010, -400, -400, 100);               
  defineLine(0b011, -400,    0, 100);
  defineLine(0b100,    0, -400, 10);
  defineLine(0b101, -400, -400, 100);
  defineLine(0b110,    0, -400, 100);
  defineLine(0b111, -200, -400, 250);
}

byte decodeProxSensors() {
  static byte result = 0;
  const unsigned long ProxSensorDelay = 10UL;
  static unsigned long proxSensorTime = 0UL;
  
  // converts proximity sensor readings to a value
  // between 0 and 7, bit mapped as
  // 2 1 0
  // | | |
  // | | +-- Right
  // | +---- Center
  // +------ Left


  if (now >= proxSensorTime) {
    proxSensors.read();

    result = 0;
    result |= (proxSensors.countsLeftWithLeftLeds() > 0) ? ProxLeftDetect : 0;
    result |= (proxSensors.countsFrontWithLeftLeds() > 0) ? ProxLeftCenterDetect : 0;
    result |= (proxSensors.countsFrontWithRightLeds() > 0) ? ProxRightCenterDetect : 0;
    result |= (proxSensors.countsRightWithRightLeds() > 0) ? ProxRightDetect : 0;

    proxSensorTime = now + ProxSensorDelay;
  }
  
  return(result);
}


byte decodeLineSensors() {
  static byte result = 0;
  const unsigned long LineSensorDelay = 10UL;
  static unsigned long lineSensorTime = 0UL;

  // converts thresholded sensor readings to a value
  // between 0 and 7, bit mapped as
  // 2 1 0
  // | | |
  // | | +-- Right
  // | +---- Center
  // +------ Left

  if (now >= lineSensorTime) {
    lineSensors.readCalibrated(lineSensorValues);

    result = 0;
    
    if (lineSensorValues[LineSensorLeft] < LineSensorThreshold) {
      result |= LineLeftDetect;
    }
  
    if (lineSensorValues[LineSensorCenter] < LineSensorThreshold) {
      result |= LineCenterDetect;
    }
    
    if (lineSensorValues[LineSensorRight] < LineSensorThreshold) {
      result |= LineRightDetect;
    }

    lineSensorTime = now + LineSensorDelay;
  }
  
  return(result);
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

void avoidEnemyBehavior(int *left, int *right) {
  static unsigned long avoidTime = 0UL;
  static int aLeft = DNC;
  static int aRight = DNC;
  static int aD;

  if (now >= avoidTime) {
    aLeft = AvoidTable[proxSensorsNow][LeftMotorIndex];
    aRight = AvoidTable[proxSensorsNow][RightMotorIndex];
    aD = AvoidTable[proxSensorsNow][DelayIndex];
    avoidTime = now + aD;
  }
       
  if (aLeft != DNC) {
    *left = aLeft;
  }

  if (aRight != DNC) {
    *right = aRight;
  }
}

void avoidBorderBehavior(int *left, int *right) {
  static unsigned long avoidTime = 0UL;
  static int aLeft = DNC;
  static int aRight = DNC;
  static int aD;

  if (now >= avoidTime) {
    aLeft = LineTable[lineSensorsNow][LeftMotorIndex];
    aRight = LineTable[lineSensorsNow][RightMotorIndex];
    aD = LineTable[lineSensorsNow][DelayIndex];
    avoidTime = now + aD;
  }
       
  if (aLeft != DNC) {
    *left = aLeft;
  }

  if (aRight != DNC) {
    *right = aRight;
  }
}


void defaultBehavior(int *left, int *right) {
  *left = 300;
  *right = 300;
}

void failsafeBehavior(int *left, int *right) {
  if ( (*left == DNC) || (*right == DNC) ) {
    *left = 0;
    *right = 0;
  }
}


void setup() {
  proxSensors.initThreeSensors();
  // In a small ring, using low settings seems to work best
  // otherwise, the robot will respond to objects or people outside
  // the ring.
  uint16_t levels[] = {4, 15}; //, 32, 55, 85, 120 };
  proxSensors.setBrightnessLevels(levels, sizeof(levels)/2);

  motors.setSpeeds(0, 0);

  initAvoidTable();
  initLineTable();

  lineSensors.initThreeSensors();
  calibrateLineSensors();

  lcd.clear();

  Serial.begin(115200);
}

  
void loop() {
  static int left = 0;
  static int right = 0;
  
  now = millis();
  proxSensorsNow = decodeProxSensors();
  lineSensorsNow = decodeLineSensors();
  leftNow = left;
  rightNow = right;

  if (Debug) {
    Serial.print(proxSensorsNow);
    Serial.print(" ");
    Serial.print(leftNow);
    Serial.print(" ");
    Serial.println(rightNow);
  }
  
  // behaviors, lowest to highest priority

  defaultBehavior(&left, &right);
  avoidEnemyBehavior(&left, &right);
  avoidBorderBehavior(&left, &right);
  failsafeBehavior(&left, &right);

  // take action, if the speeds have changed
  if ( (left != leftNow) || (right != rightNow) ) {
    motors.setSpeeds(left, right);
  }

}
