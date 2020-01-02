/*
 * Zumo Find Target
 * 
 * Use the proximity sensors to face a target
 * 
 */

#include <Wire.h>
#include <Zumo32U4.h>

Zumo32U4LCD lcd;
Zumo32U4Motors motors;
Zumo32U4ProximitySensors proxSensors;

// The speed that the robot uses when turning towards a target
const uint16_t turnSpeed = 125;

// The speed that the robot uses when looking for a target
const uint16_t scanSpeed = 75;

const unsigned long LoopDelay = 100UL;

void setup() {
  // Not the best seed, but better than nothing
  randomSeed(readBatteryMillivolts());
  
  proxSensors.initThreeSensors();

  // In a small ring, using low settings seems to work best
  // otherwise, the robot will respond to objects or people outside
  // the ring.
  uint16_t levels[] = {4, 15}; //, 32, 55, 85, 120 };
  
  proxSensors.setBrightnessLevels(levels, sizeof(levels)/2);
  motors.setSpeeds(0, 0);

  lcd.clear();

}

enum Direction {Front, Left, Right, None};

void loop() {
  static unsigned long loopTimer = 0;
  static Direction lastSeen = None;
  static Direction lastMove = None;
  
  unsigned long now = millis();

  
  if (now >= loopTimer) {
    loopTimer = now + LoopDelay;
    proxSensors.read();

    int leftCount = (int)proxSensors.countsFrontWithLeftLeds() + (int)proxSensors.countsLeftWithLeftLeds();
    int rightCount = (int)proxSensors.countsFrontWithRightLeds() + (int)proxSensors.countsRightWithRightLeds();

    int sense = rightCount - leftCount;
    
    if (sense > 0) {
      // more readings on the right
      // turn right
      lastSeen = Right;
      lastMove = Right;
      motors.setSpeeds(turnSpeed, -turnSpeed);
    }
    else if (sense < 0) {
      // more readings on the left
      // turn left
      lastSeen = Left;
      lastMove = Left;
      motors.setSpeeds(-turnSpeed, turnSpeed);
    }
    else {
      // equal left/right, or nothing sensed
      if (proxSensors.countsFrontWithRightLeds() != 0) {
        // only need to check one, they're equal
        // if here, then there is something in front of us
        lastSeen = Front;
        lastMove = None;

        // do something smart
        motors.setSpeeds(0, 0);
        
      }
      else {
        // nothing seen
        
        // do something else smart

        // Turn in the direction the robot last saw something
        // or, keep turning in the direction we last turned

        switch (lastSeen) {
          case Right:
            lastMove = Right;
            motors.setSpeeds(turnSpeed, -turnSpeed);
            break;         
          case Left:
            lastMove = Left;
            motors.setSpeeds(-turnSpeed, turnSpeed);
            break;
          case Front:
          case None:
            switch (lastMove) {
              case Right:
                lastMove = Right;
                motors.setSpeeds(scanSpeed, -scanSpeed);
                break;         
              case Left:
                lastMove = Left;
                motors.setSpeeds(-scanSpeed, scanSpeed);
                break;
              case Front:
              case None:
                // Pick a random direction
                // will be a 0 or a 1
                if (random(0,2) == 0) {
                  lastMove = Right;
                  motors.setSpeeds(scanSpeed, -scanSpeed);
                }
                else {
                  lastMove = Left;
                  motors.setSpeeds(-scanSpeed, scanSpeed);                  
                }
                break;
              default:
                // this shouldn't happen...
                lastMove = None;
                motors.setSpeeds(0, 0);
                break;   
            }
            break;
          default:
            // this shouldn't happen...
            lastMove = None;
            motors.setSpeeds(0, 0);
            break;   
        }
        lastSeen = None;
      }
    }

    lcd.gotoXY(0, 0);
    switch (lastSeen) {
      case Left:
        lcd.print(F("Left "));
        break;
      case Right:
        lcd.print(F("Right"));
        break;
      case Front:
        lcd.print(F("Front"));
        break;
      case None:
        lcd.print(F("None "));
        break;
      default:
        lcd.print(F("?????"));
        break;      
    }

    lcd.gotoXY(0, 1);
    switch (lastMove) {
      case Left:
        lcd.print(F("Left "));
        break;
      case Right:
        lcd.print(F("Right"));
        break;
      case Front:
        lcd.print(F("Front"));
        break;
      case None:
        lcd.print(F("None "));
        break;
      default:
        lcd.print(F("?????"));
        break;      
    }
  }
}
