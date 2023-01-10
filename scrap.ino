
    bool inverseKinematics(float targetX, float targetY) {
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
        
        angle1 = abs(toDegrees(atan2(targetY, targetX)));
        angle2 = 180;
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
        angle1 = aUpper2;
        angle2 = aLower2;
      } else {
        angle1 = aUpper1;
        angle2 = aLower1;
      }

//      Serial.println(angle1);
//      Serial.println(angle2);
      
      return true;
    }

    bool toPosition(float targetX, float targetY) {
      bool success = inverseKinematics(targetX, targetY);
      servo1.write(angle1);
      servo2.write(angle2);
      return success;
    }