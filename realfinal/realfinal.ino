#include <LiquidCrystal_I2C.h>
#include <Wire.h>
#include <QueueArray.h>
#include <NewPing.h>

//=======================================
//Perception///
//=======================================
//Ultrasounds Sensor1 GPIO PIN
const uint8_t echo1PIN = 13;  //define Ultsenser1 echo
const uint8_t trig1PIN = 8;   //define Ultsenser1 triger

/*
//Ultrasounds Sensor2 GPIO PIN
const uint8_t echo2PIN = 11;  //define Ultsenser2 echo
const uint8_t trig2PIN = 10;  //define Ultsenser2 triger
*/
//Configuration Settings 
const uint8_t Sensor_NUM = 1;
const uint8_t Max_distance = 150;
static long dis_l = 0;
static long dis_r = 0;
static uint8_t delay_ = 75;
//using ping libary(pin -> array)
NewPing sonar[Sensor_NUM]={
  NewPing(trig1PIN,echo1PIN, Max_distance), //13: echoPIN 12: trigPIN
  //NewPing(trig2PIN,echo2PIN, Max_distance)
};

//======================================
//decision//
//======================================
static QueueArray <long int> left_distances;
static QueueArray <long int> right_distances;
static QueueArray <long int> object_detect;
//const int Movement = 0;

//=======================================
//Control///
//=======================================
// Define Motor1 GPIO PIN
const int motor1IN1 = 7;        //define Motor1 DIR
const int motor1IN2 = 6;        //define Motor1 PWM

// Define Motor2 GPIO PIN
const int motor2IN1 = 5;        //define Motor2 DIR
const int motor2IN2 = 4;        //define Motor2 PWM

// Define PWM GPIO pin
const int PWM_ENA = 9;             //define Motor1 PWM
const int PWM_ENB = 3;             //define Motor2 PWM

// Define Motor list
#define Left 0
#define Right 1

// Define Situation
static int Movement;
static int i = 0;

const int Forward = 1;
const int Backward = 2;
const int TurnRight = 3;
const int TurnLeft = 4;
const int Stop = 5;
const int Break = 6;

LiquidCrystal_I2C display(0x27,16,2);

// Define Motor Speed (PWM is Analog signal 0~255)
unsigned int PWM = 255;
const int Time = 10;
static int Duty_left = 0;
static int Duty_right = 0;
boolean ObjectDetect = false;
boolean rightmoving = false;

//스마트카트 제어(직진,후진,우회전,일시정지,정지)
void forward()
{
  digitalWrite(motor1IN1,HIGH);
  digitalWrite(motor1IN2,LOW);
  
  digitalWrite(motor2IN1,HIGH);
  digitalWrite(motor2IN2,LOW);
}
void backward()
{
  digitalWrite(motor1IN1,LOW);
  digitalWrite(motor1IN2,HIGH);
  
  digitalWrite(motor2IN1,LOW);
  digitalWrite(motor2IN2,HIGH);
}
void turnRight()
{
  digitalWrite(motor1IN1,HIGH);
  digitalWrite(motor1IN2,LOW);
  
  digitalWrite(motor2IN1,LOW);
  digitalWrite(motor2IN2,HIGH);
}
void turnLeft()
{
  digitalWrite(motor1IN1,HIGH);
  digitalWrite(motor1IN2,LOW);
  
  digitalWrite(motor2IN1,LOW);
  digitalWrite(motor2IN2,HIGH);
}
void stop_()
{
  digitalWrite(motor1IN1,LOW);
  digitalWrite(motor1IN2,LOW);
  
  digitalWrite(motor2IN1,LOW);
  digitalWrite(motor2IN2,LOW);
}
void break_()
{
  digitalWrite(motor1IN1,HIGH);
  digitalWrite(motor1IN2,HIGH);
  
  digitalWrite(motor2IN1,HIGH);
  digitalWrite(motor2IN2,HIGH);
}

//모터제어문 캡슐화
void setMotorControl(unsigned int situation, unsigned int PWM_Value1, unsigned int PWM_Value2)
{
  //Motor Speed control = PWM
  analogWrite(PWM_ENA,PWM_Value1);
  analogWrite(PWM_ENB,PWM_Value2);
  
  if (situation == Forward){forward();
  }else if (situation == Backward){backward();
  }else if (situation == TurnRight){turnRight();
  }else if (situation == TurnLeft){turnLeft();
  }else if (situation == Stop){stop_();
  }else if (situation == Break){break_();}      
}
//스마트카트 초기세팅(init)
void setup() {
  //define Seiral function : connecting to Host Moniter
  Serial.begin(9600);
  // set DisplayConfig
  display.init();
  display.backlight();

  // set MotorConifg
  pinMode(motor1IN1 , OUTPUT);      
  pinMode(motor1IN2 , OUTPUT);
  pinMode(motor2IN1 , OUTPUT);
  pinMode(motor2IN2 , OUTPUT);

  pinMode(PWM_ENA , OUTPUT);
  pinMode(PWM_ENB , OUTPUT);
  unsigned int currentPWM = 0;

  //setting PWM init
  analogWrite(PWM_ENA,currentPWM);
  analogWrite(PWM_ENB,currentPWM);
}

boolean isfirstdata(QueueArray<long> dis){
    /*
    Serial.println("check");
    Serial.println(dis.count());
    */
    if (dis.count() == 0) {
      Serial.println("okay");
      return true;
    }
    else return false;
}
void addinstances(long dis1){
  if (left_distances.isEmpty()){
    if (ObjectDetect != false){
      left_distances.dequeue();
    }
    left_distances.enqueue(dis1);
  }
  else{
    left_distances.dequeue();
    left_distances.enqueue(dis1);   
  }
}
void TrunRight(long disl){
  if(disl ==0 ){
    rightmoving = true;
    if(left_distances.peek() >= 30){
      display.println("setting trun right");
      Movement = TurnRight;
      //tuning prameter
      Duty_left = 0;
      Duty_right = 255;
      delay_ = 50;
      dis_l = 50;
      dis_r = 50;
    }
    else if ((left_distances.peek() < 30) or (ObjectDetect = true)){
      display.println("it is failable");
   //tuning prameter
    }
    //else if (disl >50 disr >50) rightmoving = false;
  }
  addinstances(disl);
}
//스마트카트 사랑추종 기능(Cruise Control)
void followingPeople(long l_dis){
    if ((0< l_dis && l_dis< 30)){
      if (isfirstdata(left_distances) ){ 
        Movement = Stop;
        //display.println("it is first data");
      }
      else if(left_distances.peek() >= 30 ){
        Movement = Forward;
      }
      else if(left_distances.peek() < 30){
        Movement = Stop;
        //display.println("object detect");
        ObjectDetect = true;
      }
    }
    else if ((30 <=l_dis && l_dis <50 )){
        Movement = Forward;
        //display.println("set forward small moving");
        Duty_left = 198;
        Duty_right = 240;
        delay_ = 70;
    }
    else if ((50 <=l_dis)){
        //display.println("set forward full moving");
        Movement = Forward;
        Duty_left = 212;
        Duty_right = 255;
    }
}

//스마트카트 메인문
void loop(){
  //display.setCursor(0,1);
  // 초음파센서 데이터 전처리
  dis_l= sonar[Left].ping_cm();
  /* display, serial monitor 출력문
  Serial.print("distance: ");
  Serial.println(dis_l);
  //delay(1000);
  //display.print("distance: ");
  //display.println(dis_l);*/
  //사람 추종 기능- cruise control
  followingPeople(dis_l);
  //사람 추종 기능- 우회전
  TrunRight(dis_l);
  //Queue Array 객체 생성 : 거리에 따른 차량의 지시사항
  addinstances(dis_l);
  //스마트카트 제어문
  setMotorControl(Movement, Duty_left,Duty_right);
  delay(80);
  ); 
  //display.clear();
}

void exit(void){
  while(1){
  }
}
