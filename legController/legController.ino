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


class LegController {
  Servo servo1;
  Servo servo2;
  
  public:
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

    float length1;
    float length2;

    float angleOffset1 = 0;
    float angleOffset2 = 0;

    bool jointFacesPositiveX = true;
    
    LegController(int pin1, int pin2, float len1, float len2) {
//      pin1 = p1;
//      pin2 = p2;
      servo1.attach(pin1);
      servo2.attach(pin2);
      
      length1 = len1;
      length2 = len2;
    }

    bool calculateInverseKinematics(float targetX, float targetY, float* anglesArr) {
      // (jointFacesPositiveX = case 2; jointFacesPositiveX = case 1;)
      
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
      float dsq = sq(targetX - mountX) + sq(targetY - mountY);
      float d = sqrt(dsq);

      if(d > length1 + length2) {
        Serial.println("leg attempted overextension");
        
//        // debug
//        angle1 = 0;
//        angle2 = 0;
        
        return false;
      } else if(d == length1 + length2) {
//        Serial.println("leg at maximum extension");
        
//        angle1 = abs(toDegrees(atan2(targetY, targetX)));
//        angle2 = 180;
        anglesArr[0] = abs(toDegrees(atan2(targetY, targetX)));
        anglesArr[1] = 180;
        
        return true;
      }
//      Serial.println("leg operating normally");

      // distance between mount and knee
      float a = (sq(length1) - sq(length2) + dsq) / (2 * d);

      // distance between knee and target
      float b = d - a;

      // point between the intersections
      float p2x = mountX + a * (targetX - mountX) / d;
      float p2y = mountY + a * (targetY - mountY) / d;

      // distance between center to intersections
      float h = sqrt(sq(length1) - sq(a));

      // intersection 1 (case 1 for knee)
      float p3x1 = p2x + h * (targetY - mountY) / d;
      float p3y1 = p2y - h * (targetX - mountX) / d;

      // intersection 2 (case 2 for knee)
      float p3x2 = p2x - h * (targetY - mountY) / d;
      float p3y2 = p2y + h * (targetX - mountX) / d;


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

      if(jointFacesPositiveX) {
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

    bool toPosition(float targetX, float targetY) {
      float angles[2];
      bool success = calculateInverseKinematics(targetX, targetY, angles);
      if(success) {setAngles(angles);}
      return success;
    }
};


void setup() {
  Serial.begin(9600);
//  Serial.println("setup start");

  LegController leg1(5, 6, 50, 50);

//  leg1.toPosition(40, -75);
//  delay(5000);

  // legs for MEGA
//  LegController legFR(2, 3, 50, 50);
//  LegController legFL(4, 5, 50, 50);
//  LegController legBL(6, 7, 50, 50);
//  LegController legBR(8, 9, 50, 50);

  for(int x=80; x>=-80; x-=1) {
    int success = leg1.toPosition(x, -60);
    if(!success) {continue;}
    
    Serial.print("x:");
    Serial.print(x);
    Serial.print(",");
    
    Serial.print("angle1:");
    Serial.print(leg1.getAngle1());
    Serial.print(",");
    
    Serial.print("angle2:");
    Serial.print(leg1.getAngle2());
    Serial.println("");
    
    delay(2);
  }

  delay(2000);
  leg1.setAngle1(0);
  leg1.setAngle2(0);
  
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
