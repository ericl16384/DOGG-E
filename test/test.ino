

#include <Servo.h>

#define servoShoulderPin 5
#define servoKneePin 6





float toDegrees(float angleInRadians) {
  float angleInDegrees = angleInRadians * RAD_TO_DEG;
  return angleInDegrees;
}

float toRadians(float angleInDegrees) {
  float angleInRadians = angleInDegrees * DEG_TO_RAD;
  return angleInRadians;
}



Servo servoShoulder;
Servo servoKnee;

float legLength = 100; // sum of both segments
float x = 200;
float y = -100;

float distance;
float angle1;
float angle2;

void updateServos() {
  servoShoulder.write(angle1 + 180);
  servoKnee.write(angle2); // todo
}

void updateAngles() {
  distance = sqrt(pow(x, 2) + pow(y, 2));
  
  angle1 = toDegrees(atan2(y, x));
  angle2 = acos(distance / legLength);

//  angle1 -= ang



  // debug
  angle2 = 180;
}


void setup() {
  Serial.begin(9600);
  
  // put your setup code here, to run once:

  servoShoulder.attach(servoShoulderPin);
  servoKnee.attach(servoKneePin);


  servoShoulder.write(0);
  servoKnee.write(0);
  delay(1000);
}

void loop() {
  // put your main code here, to run repeatedly:

  x -= 1;

  updateAngles();
  updateServos();

//  Serial.println(angle1);
//  Serial.println(distance);
  Serial.println(acos(distance / legLength));

  delay(10);
}
