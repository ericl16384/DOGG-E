// TODO: invert lower leg





#include <Servo.h>

float toDegrees(float angleInRadians) {
  float angleInDegrees = angleInRadians * RAD_TO_DEG;
  return angleInDegrees;
}

float toRadians(float angleInDegrees) {
  float angleInRadians = angleInDegrees * DEG_TO_RAD;
  return angleInRadians;
}


class Leg {
  public:
    Servo servo1;
    Servo servo2;
    
//    float getAngle1() {return angle1;}
//    float getAngle2() {return angle2;}
//    float setAngle1(float a) {angle1 = a; controlServos();}
//    float setAngle2(float a) {angle2 = a; controlServos();}

    float getAngle1() {return servo1.read() + angleOffset1;}
    float getAngle2() {return servo2.read() + angleOffset2;}
    
    void setAngle1(float a) {servo1.write(a - angleOffset1);}
    void setAngle2(float a) {servo2.write(a - angleOffset2);}
    void setAngles(float* a) {setAngle1(a[0]); setAngle2(a[1]);}
  
    float mountX = 0;
    float mountY = 0;
    float mountZ = 0;

//    float footX;
//    float footY;
//    float footZ;

    float length1;
    float length2;

    float angleOffset1 = 0;
    float angleOffset2 = 0;

    bool kneeFacesPositive = true;

    Leg() {}
    Leg(int pin1, int pin2, float len1, float len2) {
//      pin1 = p1;
//      pin2 = p2;
      servo1.attach(pin1);
      servo2.attach(pin2);
      
      length1 = len1;
      length2 = len2;
    }

    bool calculateInverseKinematics(float targetZ, float targetY, float* anglesArr) {
      // (kneeFacesPositive = case 2; kneeFacesPositive = case 1;)
      
      // find knee position
      
      // https://stackoverflow.com/questions/3349125/circle-circle-intersection-points
      // http://paulbourke.net/geometry/circlesphere/
      // http://paulbourke.net/geometry/circlesphere/2circle1.gif

      // r0 = length1
      // r1 = length2
      
      // p0 = mount
      // p1 = target
      // p2 = midpoint between intersections
      // p3 = intersection (0, 1, 2 count)

      // distance between mount and target
      float dsq = sq(targetZ - mountX) + sq(targetY - mountY);
      float d = sqrt(dsq);

      if(d > length1 + length2) {
        Serial.println("leg attempted overextension");
        
//        // debug
//        angle1 = 0;
//        angle2 = 0;
        
        return false;
      } else if(d == length1 + length2) {
//        Serial.println("leg at maximum extension");
        
//        angle1 = abs(toDegrees(atan2(targetY, targetZ)));
//        angle2 = 180;
        anglesArr[0] = abs(toDegrees(atan2(targetY, targetZ)));
        anglesArr[1] = 180;
        
        return true;
      }
//      Serial.println("leg operating normally");

      // distance between mount and knee
      float a = (sq(length1) - sq(length2) + dsq) / (2 * d);

      // distance between knee and target
      float b = d - a;

      // point between the intersections
      float p2x = mountX + a * (targetZ - mountX) / d;
      float p2y = mountY + a * (targetY - mountY) / d;

      // distance between center to intersections
      float h = sqrt(sq(length1) - sq(a));

      // intersection 1 (case 1 for knee)
      float p3x1 = p2x + h * (targetY - mountY) / d;
      float p3y1 = p2y - h * (targetZ - mountX) / d;

      // intersection 2 (case 2 for knee)
      float p3x2 = p2x - h * (targetY - mountY) / d;
      float p3y2 = p2y + h * (targetZ - mountX) / d;


      // find angles

      // case 1
      float aUpper1 = abs(toDegrees(atan2(p3y1, p3x1)));
      float aLower1 = abs(toDegrees(atan(a / h) + atan(b / h)));

      // case 2
      float aUpper2 = abs(toDegrees(atan2(p3y2, p3x2)));
      float aLower2 = abs(toDegrees(atan(a / h) + atan(b / h)));

//      Serial.println(aUpper1);
//      Serial.println(aLower1);
//      Serial.println(aUpper2);
//      Serial.println(aLower2);


      // finish

      if(kneeFacesPositive) {
//        angle1 = aUpper2;
//        angle2 = aLower2;
        anglesArr[0] = aUpper2;
        anglesArr[1] = aLower2;
      } else {
//        angle1 = aUpper1;
//        angle2 = aLower1;
        anglesArr[0] = aUpper1;
        anglesArr[1] = aLower1;
      }

//      Serial.println(angle1);
//      Serial.println(angle2);
      
      return true;
    }

    bool toPosition(float targetZ, float targetY) {
      float angles[2];
      bool success = calculateInverseKinematics(targetZ, targetY, angles);
      if(success) {setAngles(angles);}
      return success;
    }
};

//class Robot {
//  int numLegs = 4;
//  int legsInitialized = 0;
////  Leg legs[numLegs];
//  Leg legs[4];
//
//  public:
//    Robot() {}
//  
////    bool newLeg(Leg leg) {
////      if(legsInitialized == numLegs) {
////        return false;
////      }
////      
////      legs[legsInitialized] = leg;
////      legsInitialized++;
////      return true;
////    }
//    Leg* newLeg(int pin1, int pin2, float len1, float len2) {
//      if(legsInitialized == numLegs) {
//        return 0;
//      }
//      
//      legs[legsInitialized] = Leg(pin1, pin2, len1, len2);
//      legsInitialized++;
//      return &legs[legsInitialized-1];
//    }
//
//    float originX = 0;
//    float originY = 0;
//    float originZ = 0;
//};

struct Robot {
  Leg LF;
  Leg LB;
  // mega
//  Leg RF;
//  Leg RB;

  float positionX = 0;
  float positionY = 0;
  float positionZ = 0;

//  bool calculateInverseKinematics(float targetX, float targetY, float* anglesArr) {
//    
//  }

  bool toPosition(float x, float y, float z) {
    // for now
    if(x != 0) {Serial.println("robot cannot move sideways"); return false;}
    
    positionX = x;
    positionY = y;
    positionZ = z;

    // assume mount is (0, 0)

    float anglesLF[2], anglesLB[2];
    int successes = 0;
    successes += LF.calculateInverseKinematics(-positionZ, -positionY, anglesLF);
    successes += LB.calculateInverseKinematics(-positionZ, -positionY, anglesLB);
    if(successes == 2) {
      LF.setAngles(anglesLF);
      LB.setAngles(anglesLB);
      return true;
    } else {
      Serial.println("robot attempted invalid position");
      return false;
    }
  }

  void printSerialPlotter() {
    Serial.print("LF1:"); Serial.print(LF.getAngle1()); Serial.print(",");
    Serial.print("LF2:"); Serial.print(LF.getAngle2()); Serial.print(",");
    
    Serial.print("LB1:"); Serial.print(LB.getAngle1()); Serial.print(",");
    Serial.print("LB2:"); Serial.print(LB.getAngle2()); Serial.print(",");
    
//    Serial.print("RF1:"); Serial.print(RF.getAngle1()); Serial.print(",");
//    Serial.print("RF2:"); Serial.print(RF.getAngle2()); Serial.print(",");
//    
//    Serial.print("RB1:"); Serial.print(RB.getAngle1()); Serial.print(",");
//    Serial.print("RB2:"); Serial.print(RB.getAngle2()); Serial.print(",");

    Serial.println("");
  }
};


void setup() {
  Serial.begin(9600);
//  Serial.println("setup start");

//  Leg legLF(9, 10, 50, 50); legLF.kneeFacesPositive = false;
//  Leg legLB(5, 6, 50, 50);
  Robot robot;
  robot.LF = Leg(9, 10, 50, 50); robot.LF.kneeFacesPositive = false;
  robot.LB = Leg(5, 6, 50, 50);
  
  
//  Leg* l;
//  // left back
//  l = robot.newLeg(5, 6, 50, 50); l->mountX = -50; l->mountZ = -64.5;

//  leg1.toPosition(40, -75);
//  delay(5000);

  // legs for MEGA
//  Leg legFR(2, 3, 50, 50);
//  Leg legFL(4, 5, 50, 50);
//  Leg legBL(6, 7, 50, 50);
//  Leg legBR(8, 9, 50, 50);

  for(int x=80; x>=-80; x-=1) {
//    bool success = leg1.toPosition(x, -60);
//    bool success = l->toPosition(x, -60);
//    bool success = legLB.toPosition(x, -60);
    bool success = robot.toPosition(0, 60, x);
    
    if(!success) {continue;}
    
    Serial.print("x:");
    Serial.print(x);
    Serial.print(",");
//    
//    Serial.print("angle1:");
////    Serial.print(leg1.getAngle1());
//    Serial.print(robot.LF.getAngle1());
//    Serial.print(",");
//    
//    Serial.print("angle2:");
////    Serial.print(leg1.getAngle2());
//    Serial.print(robot.LF.getAngle2());
//    Serial.print(",");
//    
//    Serial.println("");

    robot.printSerialPlotter();
    
    delay(2);
  }

  delay(2000);
//  leg1.setAngle1(0);
//  leg1.setAngle2(0);

  robot.LF.setAngle1(0);
  robot.LF.setAngle2(0);
  robot.LB.setAngle1(0);
  robot.LB.setAngle2(0);
  
//  Serial.println("setup end");
}

void loop() {
  


////  Serial.print("x:");
////  Serial.print(x);
////  Serial.print(", ");
//  
//  Serial.print("angle1:");
//  Serial.print(leg1.getAngle1());
//  Serial.print(", ");
//  
//  Serial.print("angle2:");
//  Serial.print(leg1.getAngle2());
//  Serial.println("");
}
