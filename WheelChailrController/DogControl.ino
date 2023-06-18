#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();


int incomingByte = 0;
int initFlag = 0;

void setup() {
  Serial.begin(115200);
  Serial.println("12 channel Servo test!");
  pwm.begin();
  pwm.setPWMFreq(50);
}

void loop() {
  if (Serial.available() > 0) {
    incomingByte = Serial.read();
    Serial.println(incomingByte, DEC);
    
    if (incomingByte == 115) {  // input = 's'
      if (initFlag == 0) {
        initServo();
        delay(1000);
        initFlag = 1;
      }
      
    }
    
    if (incomingByte == 102) {  // input = 'f'
      forward();
    }
    
    if (incomingByte == 108) {  // input = 'l'
      leftTurn();
    }
    
    if (incomingByte == 114) {  // input = 'r'
      rightTurn();
    }
    
    if (incomingByte == 101) {  // input = 'e'
      initFlag = 0;
    }
  }
  delay(50);
}
