#define PIN 5



#include <Servo.h>

Servo servo;
float angle = 0;

void setup() {
  Serial.begin(9600);
  servo.attach(PIN);
  delay(100);

//  servo.write(0);
//  delay(500);
//  servo.write(90);
//  delay(500);
//  servo.write(180);
//  delay(500);
//  servo.write(0);

  Serial.print("starting");

  servo.write(0);
  delay(100);

 // scan from 0 to 180 degrees
  for(angle = 0; angle < 180; angle++)  
  {                                  
    servo.write(angle);               
    delay(15);                   
  } 
  // now scan back from 180 to 0 degrees
  for(angle = 180; angle > 0; angle--)    
  {                                
    servo.write(angle);           
    delay(15);       
  }

  delay(100);
  servo.write(0);

  Serial.print("finished");
}

void loop() {
}
