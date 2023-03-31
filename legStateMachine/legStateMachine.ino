

#include <Servo.h>

#define servoShoulderPin 5
#define servoKneePin 6

//#define servoPowerPin 30





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
float x = -60;
float y = -60;

float distance;
float angle1;
float angle2;

void updateServos() {
  servoShoulder.write(angle1);
  servoKnee.write(angle2); // todo
}

bool updateAngles() {
  distance = sqrt(pow(x, 2) + pow(y, 2));

  angle1 = 180;
  angle2 = 0;

  // distance
  if(distance / legLength < 1) {
    angle1 += toDegrees(asin(distance / legLength)) * -1;
    angle2 += toDegrees(asin(distance / legLength)) * 2;
  } else if(distance / legLength == 1) {
    angle1 += -90;
    angle2 += 180;
  } else {
    return false;
  }

//  Serial.println(angle1);

  // angle
  if(distance > 0) {
    angle1 += toDegrees(atan2(y, x)) + 90;
  }
  Serial.println(toDegrees(atan2(y, x)) + 90);
  
  return true;
}



void setup() {
  Serial.begin(9600);

  servoShoulder.attach(servoShoulderPin);
  servoKnee.attach(servoKneePin);
  
//  pinMode(servoPowerPin, OUTPUT);


  servoShoulder.write(0);
  servoKnee.write(180);
  delay(1000);

//  digitalWrite(servoPowerPin, LOW);
  
  servoShoulder.write(180);
  servoKnee.write(0);
  delay(1000);

//   DEBUG
//  delay(1000000);
}


#define STATE_STOWED -1
#define STATE_LIFTING 0
#define STATE_NO_CONTACT 1
#define STATE_GROUND_CONTACT 2
#define STATE_TO_GROUND 3
#define STATE_FROM_GROUND 4

int state = STATE_STOWED;



void loop() {
//  x += 0.2;
////  y -= 0.5;

  if(state == STATE_STOWED) {
    state = STATE_LIFTING;
  }


  if(updateAngles()) { updateServos(); }











//  Serial.println(angle1);
//  Serial.println(distance);
//  Serial.println(angle2);

//  Serial.print("x:");
//  Serial.print(x);
//  Serial.print(",");
//
//  Serial.print("y:");
//  Serial.print(y);
//  Serial.print(",");
//
//  Serial.print("a1:");
//  Serial.print(angle1);
//  Serial.print(",");
//
//  Serial.print("a2:");
//  Serial.print(angle2);
//  Serial.print(",");
//
//  Serial.println("");

  delay(10);
}
