// defines pins numbers
const int STEP[5] = {25, 14, 19, 5, 22};
const int DIR[5]  = {33, 27, 21, 18, 23};
#define enable 10

 
void setup() {
  // Sets the two pins as Outputs
  for (int i; i<5;i++){
    pinMode(STEP[i], OUTPUT);
    pinMode(DIR[i], OUTPUT);
  }
  pinMode(enable, OUTPUT);
  digitalWrite(enable, LOW); //activate stepper
}
void loop() {
  for(int i=0; i<5;i++) digitalWrite(DIR[i],HIGH); // Enables the motor to move in a particular direction
  
  // Makes 200 pulses for making one full cycle rotation
  for(int x = 0; x < 2000; x++) {
    for(int i=0; i<5;i++) digitalWrite(STEP[i],HIGH); // Enables the motor to move in a particular direction
    delayMicroseconds(400);
     
    for(int i=0; i<5;i++) digitalWrite(STEP[i],LOW); // Enables the motor to move in a particular direction
    delayMicroseconds(400); 
  }
  delay(1000); // One second delay
  
  for(int i=0; i<5;i++) digitalWrite(DIR[i],LOW); // change direction rotation

  // Makes 400 pulses for making two full cycle rotation
  for(int x = 0; x < 4000; x++) {
    for(int i=0; i<5;i++) digitalWrite(STEP[i],HIGH); // Enables the motor to move in a particular direction
    delayMicroseconds(400);
     
    for(int i=0; i<5;i++) digitalWrite(STEP[i],LOW); // Enables the motor to move in a particular direction
    delayMicroseconds(400); 
  }
  delay(1000);
}
