/*
 * Created by ArduinoGetStarted.com
 *
 * This example code is in the public domain
 *
 * Tutorial page: https://arduinogetstarted.com/faq/how-to-control-speed-of-servo-motor
 */

#include <Servo.h>

#define PIN_SERVO 10

Servo myServo;
unsigned long MOVING_TIME = 30000; // moving time is 3 seconds
unsigned long moveStartTime;
int startAngle = 0; // 30°
int stopAngle  = 90; // 90°

void setup() {
  myServo.attach(PIN_SERVO);
  moveStartTime = millis(); // start moving

  myServo.write(startAngle); // Set position
  delay(500);
}

void loop() {
  unsigned long progress = millis() - moveStartTime;

  if (progress <= MOVING_TIME) {
    // while moving
    long angle = map(progress, 0, MOVING_TIME, startAngle, stopAngle);
    myServo.write(angle); 
  } else {
    // movement finished -> reverse direction
    int temp = startAngle;
    startAngle = stopAngle;
    stopAngle = temp;

    moveStartTime = millis(); // reset start time for next movement
  }
}
