#include <RedBot.h>

RedBotSensor left = RedBotSensor(A3);   // initialize a left sensor object on A3
RedBotSensor center = RedBotSensor(A6); // initialize a center sensor object on A4
RedBotSensor right = RedBotSensor(A7);  // initialize a right sensor object on A7
RedBotMotors motors;

byte data [4];
boolean manualMode = true;
int leftSpeed;
int rightSpeed;

int leftRead;
int centerRead;
int rightRead;

#define LINETHRESHOLD 800
#define SPEED 65  // sets the nominal speed. Set to any number from 0 - 255.


void setup() {
  Serial.begin(9600);

}
void loop() {
  if (Serial.available() > 0) {
    Serial.readBytes(data, 4);
    //Serial.println("I got epic value" + data[0])
    if (data[1] == 2 && data[3] == 2) {
      manualMode = !manualMode;
      //Serial.println("Actually going automatic rn fr fr");
    }
    if (manualMode) {
      leftSpeed = data[0] * pow(-1, data[1]);
      rightSpeed = data[2] * pow(-1, data[3]);

    }
  }
  //if (!manualMode) {
  leftRead = left.read();
  centerRead = center.read();
  rightRead = right.read();
  //Serial.print(leftRead);
  //Serial.print("\t");  // tab character
  //Serial.print(centerRead);
  //Serial.print("\t");  // tab character
  //Serial.print(rightRead);
  if (centerRead < LINETHRESHOLD)
  {
    leftSpeed = -leftSpeed;
    rightSpeed = rightSpeed;
  }

  // if the line is under the right sensor, adjust relative speeds to turn to the right
  else if (rightRead < LINETHRESHOLD)
  {
    leftSpeed = -(leftSpeed + 30);
    rightSpeed = rightSpeed - 30;
  }

  // if the line is under the left sensor, adjust relative speeds to turn to the left
  else if (leftRead < LINETHRESHOLD)
  {
    leftSpeed = -(leftSpeed - 30);
    rightSpeed = rightSpeed + 30;
  }
  //}
  motors.leftMotor(leftSpeed);
  motors.rightMotor(rightSpeed);
}
