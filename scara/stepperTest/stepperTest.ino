// defines pins numbers
const int step1 = 23;
const int step11 = 22;
const int step2 = 21;
const int step22 = 19;
const int step3 = 18;
const int step33 = 5;
const int step4 = 27;
const int step44 = 14;
const int step5 = 33; 
const int step55 = 25;

const int dirPin = 34; 
 
void setup() {
  // Sets the two pins as Outputs
  pinMode(step1,OUTPUT);
  pinMode(step2,OUTPUT);
  pinMode(step3,OUTPUT);
  pinMode(step4,OUTPUT);
  pinMode(step5,OUTPUT); 
  pinMode(step11,OUTPUT);
  pinMode(step22,OUTPUT);
  pinMode(step33,OUTPUT);
  pinMode(step44,OUTPUT);
  pinMode(step55,OUTPUT); 
  pinMode(dirPin,OUTPUT);
}
void loop() {
  digitalWrite(dirPin,HIGH); // Enables the motor to move in a particular direction
  // Makes 200 pulses for making one full cycle rotation
  for(int x = 0; x < 2000; x++) {
    digitalWrite(step1,HIGH);
    digitalWrite(step2,HIGH);
    digitalWrite(step3,HIGH);
    digitalWrite(step4,HIGH);
    digitalWrite(step5,HIGH); 
    digitalWrite(step11,HIGH);
    digitalWrite(step22,HIGH);
    digitalWrite(step33,HIGH);
    digitalWrite(step44,HIGH);
    digitalWrite(step55,HIGH);
    delayMicroseconds(400); 
    digitalWrite(step1,LOW);
    digitalWrite(step2,LOW);
    digitalWrite(step3,LOW);
    digitalWrite(step4,LOW);
    digitalWrite(step5,LOW); 
    digitalWrite(step11,LOW);
    digitalWrite(step22,LOW);
    digitalWrite(step33,LOW);
    digitalWrite(step44,LOW);
    digitalWrite(step55,LOW); 
    delayMicroseconds(400); 
  }
  delay(1000); // One second delay
  
  digitalWrite(dirPin,LOW); //Changes the rotations direction
  // Makes 400 pulses for making two full cycle rotation
  for(int x = 0; x < 4000; x++) {
    digitalWrite(step1,HIGH);
    digitalWrite(step2,HIGH);
    digitalWrite(step3,HIGH);
    digitalWrite(step4,HIGH);
    digitalWrite(step5,HIGH); 
    digitalWrite(step11,HIGH);
    digitalWrite(step22,HIGH);
    digitalWrite(step33,HIGH);
    digitalWrite(step44,HIGH);
    digitalWrite(step55,HIGH);
    delayMicroseconds(400); 
    digitalWrite(step1,LOW);
    digitalWrite(step2,LOW);
    digitalWrite(step3,LOW);
    digitalWrite(step4,LOW);
    digitalWrite(step5,LOW); 
    digitalWrite(step11,LOW);
    digitalWrite(step22,LOW);
    digitalWrite(step33,LOW);
    digitalWrite(step44,LOW);
    digitalWrite(step55,LOW); 
    delayMicroseconds(400);
  }
  delay(1000);
}
