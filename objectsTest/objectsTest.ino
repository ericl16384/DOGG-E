struct Test {
  float x = 0;
  float y = 0;
  float z = 0;

  void print() {
    Serial.print("Test(");
    Serial.print(x);
    Serial.print(", ");
    Serial.print(y);
    Serial.print(", ");
    Serial.print(z);
    Serial.println(",)");
  }
};


void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);

  Test a;
  a.x = 1;

  Test b = a;
  b.y = 2;
  a.z = 3;

  a.print();
  b.print();
}

void loop() {
  // put your main code here, to run repeatedly:

}
