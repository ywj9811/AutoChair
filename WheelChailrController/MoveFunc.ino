void forward(){
  Serial.println("forward");
  if (sitFlag == 1) {
    for (int i = 0; i < 4; i++) {
      if (fl_b_Flag==1) {
        pwm.setPWM(1,0,fl_b-25);
        delay(200);
        hr_b_Flag = 1;
        pwm.setPWM(1,0,fl_b);
        fl_b_Flag = 0;
      }
      if (fr_b_Flag == 1) {
        pwm.setPWM(5,0,fr_b+25);
        delay(200);
        hl_b_Flag = 1;
        pwm.setPWM(5,0,fr_b);
        fr_b_Flag = 0;
      }
      if (hr_b_Flag==1) { 
        pwm.setPWM(13,0,hr_b+25);
        delay(200);
        fr_b_Flag = 1;
        pwm.setPWM(13,0,hr_b);
        hr_b_Flag = 0;
      }
      if (hl_b_Flag == 1) {
        pwm.setPWM(9,0,hl_b-25); 
        delay(200);
        fl_b_Flag = 1;
        pwm.setPWM(9,0,hl_b);
        hl_b_Flag = 0;
      }
    }
  }
}

void leftTurn() {
  Serial.println("left turn");
  if (sitFlag == 1) {
    if (fr_a_Flag==1) {
      pwm.setPWM(4,0,fr_a-65);
      fl_b_Flag=1;
      delay(200);
      pwm.setPWM(4,0,fr_a);
      fr_a_Flag = 0;
    }
    if (fl_b_Flag == 1) {
      pwm.setPWM(1,0,fl_b+25);
      hr_b_Flag = 1;
      delay(200);
      pwm.setPWM(1,0,fl_b);
      fl_b_Flag = 0;
    }
    if (hr_b_Flag == 1) {
      pwm.setPWM(13,0,hr_b+25);
      fr_a_Flag = 1;
      delay(200);
      pwm.setPWM(13,0,hr_b);
      hr_b_Flag = 0;
    }
  }
  fl_b_Flag = 1;
}

void rightTurn() {
  Serial.println("right turn");
  if (sitFlag == 1) {
    if (fl_a_Flag==1) {
      pwm.setPWM(0,0,fl_a+65);
      fr_b_Flag=1;
      delay(200);
      pwm.setPWM(0,0,fl_a);
      fl_a_Flag = 0;
    }
    if (fr_b_Flag == 1) {
      pwm.setPWM(5,0,fr_b-25);
      hl_b_Flag = 1;
      delay(200);
      pwm.setPWM(5,0,fr_b);
      fr_b_Flag = 0;
    }
    if (hl_b_Flag == 1) {
      pwm.setPWM(9,0,hl_b-25);
      fl_a_Flag = 1;
      delay(200);
      pwm.setPWM(9,0,hl_b);
      hl_b_Flag = 0;
    }
  }
  fl_b_Flag = 1;
}
