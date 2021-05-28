#include <Servo.h>
int input[6];
const int servonum = 5; // The amount of servos
Servo servo[servonum]; // Create servo object
//변수 정리

int sleep_detection= -1;
int theta=0;

//각도 변수
int current[5]={90,120, 60, 90,135}; //현재의 모터 각도값들을 저장

int* realtime_input();
void faceseeking(int* current);
void hori_control(int* current);
void vert_control(int* current);

void setup() {
  Serial.begin(9600);
  servo[0].attach(7); //alpha
  servo[1].attach(8); //beta1
  servo[2].attach(9); //beta2
  servo[3].attach(11); //gamma
  servo[4].attach(12); //delta
  servo[0].write(90);
  delay(30);
  servo[1].write(120);
  delay(30);
  servo[2].write(60);
  delay(30);
  servo[3].write(90);
  delay(30);
  servo[4].write(135); //90도가 기준 수평
  delay(30);
  faceseeking(current);
}


void loop() {
  hori_control(current);
  vert_control(current);
  sleep();
}

void hori_control(int* current){ 
  realtime_input();
  if(input[3]<-60||input[2]>30){
      for(int i=current[0];i<130;i++){
        servo[0].write(i);
        delay(10);     
        servo[3].write(i);
        delay(10);
        realtime_input();
        if (input[3]>-20 && input[2]<10){
          current[0]=i;
          current[3]=i;
          break;
        }
        current[0]=i;
        current[3]=i;
  }
  }
  realtime_input();
  if(input[3]>60||input[2]<-30){ 
      for(int i=current[0];i>60;i--){
        servo[0].write(i);
        delay(10);     
        servo[3].write(i);
        delay(10);
        realtime_input();
        if (input[3]<20 && input[2]>-10){
          current[0]=i;
          current[3]=i;
          break;
        }
        current[0]=i;
        current[3]=i;
  }
  }
}

void vert_control(int* current){
  realtime_input();
  if (input[4]<-40){
    for(int i=current[1];i<180;i++){
        servo[1].write(i);
        theta=180-i;
        delay(10);
        servo[2].write(180-2*theta);
        delay(10);
        servo[4].write(i+15);
        delay(10);
        realtime_input();
        if (input[4]>-20){
          current[1]=i;
          current[2]=180-2*theta;
          current[4]=i+15;
          break;
        }
        current[1]=i;
        current[2]=180-2*theta;
        current[4]=i+15;
  }
  }
  realtime_input();
  if (input[4]>40){   
     for(int i=current[1];i>120;i--){
        servo[1].write(i);
        theta=180-i;
        delay(10);
        servo[2].write(180-2*theta);
        delay(10);
        servo[4].write(i+15);
        delay(10);
        realtime_input();
        if (input[4]<20){
          current[1]=i;
          current[2]=180-2*theta;
          current[4]=i+15;
          break;
        }
        current[1]=i;
        current[2]=180-2*theta;
        current[4]=i+15;
  }
  }
}


//돌다가 얼굴 발견하면 멈춤
void faceseeking(int* current){
  delay(5000); //파이썬 코드 연결 시간
  int i=90;
  int s=1;
  while(1){
    servo[0].write(i);
    delay(15);
    servo[3].write(i);
    delay(100);
    if(Serial.available()>0){
      current[0]=i;
      break;
      }
     i+=s;
     if (i>100){
      s=-1;
     }
     if(i<50){
      s=1;
     }
  }
  
}

void sleep(){ 
  int check=0;
  realtime_input();
  if(input[5]==2){
    while(check!=4){
      check=0;
      for(int i=0;i<5;i++){
        servo[i].write(current[i]-1);
        delay(15);
        current[i]-=1;
        if (current[i]<0){
          check+=1;
        }
      }
    }
    exit(0);
  }
}

int*int int* realtime_input(){
    while(Serial.available() <=0); //입력 받을때까지
    if (Serial.available() >= 6){
    for (int i = 0; i < 6; i++){
      if (i==5){
        input[i] = Serial.read();
      }
      else{
        input[i] = Serial.read()*10-500;
      }
    }
}
  return input;
}
