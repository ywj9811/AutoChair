// 초기 값 0~180을 펄스길이150~600으로 매핑해주고, rad의 최소,최대를 150,600을 넘지 않게 해준다.
// Spread of leg -> alpha down
// Front Left up -> beta down, gamma up
int fl_a = constrain(map(0, 0, 180, 150, 600), 150, 600);
int fl_b = constrain(map(50, 0, 180, 150, 600), 150, 600);
// int fl_g = constrain(map(0, 0, 180, 150, 600), 150, 600);
//int fl_b = 150;
int fl_g = 150;

// Spread of leg -> alpha up
// Front right up -> beta up, gamma down
int fr_a = constrain(map(30, 0, 180, 150, 600), 150, 600);
//int fr_b = constrain(map(40, 0, 180, 150, 600), 150, 600);
// int fr_g = constrain(map(118, 0, 180, 150, 600), 150, 600);
int fr_b = 150;
int fr_g = 150;

// Spread of leg -> alpha up
// Hind Left up -> beta down, gamma up
int hl_a = constrain(map(105, 0, 180, 150, 600), 150, 600);
//int hl_b = constrain(map(100, 0, 180, 150, 600), 150, 600);
// int hl_g = constrain(map(0, 0, 180, 150, 600), 150, 600);
int hl_b = 150;
int hl_g = 150;

// Spread of leg -> alpha down
// Hind right up -> beta up, gamma down
int hr_a = constrain(map(130, 0, 180, 150, 600), 150, 600);
//int hr_b = constrain(map(30, 0, 180, 150, 600), 150, 600);
// int hr_g = constrain(map(105, 0, 180, 150, 600), 150, 600);
int hr_b = 170;
int hr_g = 150;

// Flag
int fl_a_Flag = 1;
int hr_a_Flag = 0;
int fr_a_Flag = 1;
int hl_a_Flag = 0;

int fl_b_Flag = 1;
int hr_b_Flag = 0;
int fr_b_Flag = 0;
int hl_b_Flag = 0;

int sitFlag = 1; // status: situp

void  initServo(){
  pwm.setPWM(0,0,fl_a); 
  pwm.setPWM(4,0,fr_a); 
  pwm.setPWM(8,0,hl_a); 
  pwm.setPWM(12,0,hr_a);
  pwm.setPWM(10,0,hl_g); 
  pwm.setPWM(14,0,hr_g);
  pwm.setPWM(9,0,hl_b); 
  pwm.setPWM(13,0,hr_b);
  pwm.setPWM(1,0,fl_b); 
  pwm.setPWM(5,0,fr_b);
  pwm.setPWM(2,0,fl_g); 
  pwm.setPWM(6,0,fr_g);
  
//  Serial.printf("fl_a = %d \n" , fl_a);
//  Serial.printf("fl_b = %d \n" , fl_b);
//  Serial.printf("fl_g = %d \n" , fl_g);
//  Serial.printf("fr_a = %d \n" , fr_a);
//  Serial.printf("fr_b = %d \n" , fr_b);
//  Serial.printf("fr_g = %d \n" , fr_g);
//  Serial.printf("hl_a = %d \n" , hl_a);
//  Serial.printf("hl_b = %d \n" , hl_b);
//  Serial.printf("hl_g = %d \n" , hl_g);
//  Serial.printf("hr_a = %d \n" , hr_a);
//  Serial.printf("hr_b = %d \n" , hr_b);
//  Serial.printf("hr_g = %d \n" , hr_g);
 
}
