#include <Servo.h>
#define PIN_SERVO 10

Servo myservo;

void setup() {
  myservo.attach(PIN_SERVO);
  myservo.write(90);
  delay(1000);
}

void loop() {
  myservo.write(0);    delay(500);
  myservo.write(90);   delay(500);
  myservo.write(180);  delay(500);
  myservo.write(90);   delay(500);
}
