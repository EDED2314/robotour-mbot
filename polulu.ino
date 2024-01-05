/* This demo shows how the 3pi+ can use its gyroscope to detect
when it is being rotated, and use the motors to resist that
rotation.

Be careful to not move the robot for a few seconds after starting
it while the gyro is being calibrated.  During the gyro
calibration, the yellow LED is on and the words "Gyro cal" are
displayed on the display.

After the gyro calibration is done, press button A to start the
demo.  If you try to turn the 3pi+, or put it on a surface that
is turning, it will drive its motors to counteract the turning.

This demo only uses the Z axis of the gyro, so it is possible to
pick up the 3pi+, rotate it about its X and Y axes, and then put
it down facing in a new position. */

#include <Wire.h>
#include <Pololu3piPlus32U4.h>
#include <PololuMenu.h>

/* The IMU is not fully enabled by default since it depends on the
Wire library, which uses about 1400 bytes of additional code space
and defines an interrupt service routine (ISR) that might be
incompatible with some applications (such as our TWISlave example).

Include Pololu3piPlus32U4IMU.h in one of your cpp/ino files to
enable IMU functionality.
*/
#include <Pololu3piPlus32U4IMU.h>

using namespace Pololu3piPlus32U4;

// Change next line to this if you are using the older 3pi+
// with a black and green LCD display:
// LCD display;
OLED display;

Buzzer buzzer;
ButtonA buttonA;
ButtonB buttonB;
ButtonC buttonC;
Motors motors;
IMU imu;

#include "TurnSensor.h"

// This is the maximum speed the motors will be allowed to turn.
// A maxSpeed of 400 lets the motors go at top speed.  Decrease
// this value to impose a speed limit.
int16_t maxSpeed = 200;


void setup() {
  Serial.begin(9600);

  // Delay before calibrating the gyro.
  delay(1000);

  test();

  turnSensorSetup();
  turnSensorReset();

  display.clear();
  display.print(F("Try to"));
  display.gotoXY(0, 1);
  display.print(F("turn me!"));
}

void loop() {

  // Read the gyro to update turnAngle, the estimation of how far
  // the robot has turned, and turnRate, the estimation of how
  // fast it is turning.
  turnSensorUpdate();

  // Calculate the motor turn speed using proportional and
  // derivative PID terms.  Here we are a using a proportional
  // constant of 28 and a derivative constant of 1/40.
  int32_t turnSpeed = -(int32_t)turnAngle / (turnAngle1 / 28)
                      - turnRate / 40;


  Serial.println("----");
  Serial.println(turnSpeed);
  Serial.println(turnRate*0.07);

  turnSpeed = constrain(turnSpeed, -maxSpeed, maxSpeed);

  motors.setSpeeds(-turnSpeed, turnSpeed);
}

/*
We will be using the 180 and -180 thing again
*/
void turnCertainAngle(int angletoturnto) {
  int32_t angle = angletoturnto * (pow(2, 29) / 45);
  while (abs(turnAngle - angle) > 1000) {

    int32_t turnSpeed = -(int32_t)turnAngle / (turnAngle1 / 28)
                        - turnRate / 40;
    turnSpeed = constrain(turnSpeed, -maxSpeed, maxSpeed);

    motors.setSpeeds(-turnSpeed, turnSpeed);
  }
}

/* we have successfully determined that the speed ratio 400 speed unit/ 1.5 m/s works*/
void test() {
  int speed = 400;
  motors.setSpeeds(speed, speed);

  delay(200);

  motors.setSpeeds(0, 0);

  while (true) {
  }
}