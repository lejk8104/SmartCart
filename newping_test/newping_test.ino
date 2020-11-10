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

//Ultrasounds Sensor2 GPIO PIN
const uint8_t echo2PIN = 11;  //define Ultsenser2 echo
const uint8_t trig2PIN = 10;  //define Ultsenser2 triger

//Configuration Settings 
const uint8_t Sensor_NUM = 2;
const uint8_t Max_distance = 200;

static long dis_l = 0;
static long dis_r = 0;

//using ping libary(pin -> array)
NewPing sonar[Sensor_NUM]={
  NewPing(trig1PIN,echo1PIN, Max_distance), //13: echoPIN 12: trigPIN
  NewPing(trig2PIN,echo2PIN, Max_distance)
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

void setMotorControl(unsigned int situation, unsigned int PWM_Value1, unsigned int PWM_Value2)
{
  //Motor Speed control = PWM
  analogWrite(PWM_ENA,PWM_Value1);
  analogWrite(PWM_ENB,PWM_Value2);
  
  if (situation == Forward){
    forward();
  }
  else if (situation == Backward){
    backward();
  }
  else if (situation == TurnRight){
    turnRight();
  }
  else if (situation == TurnLeft){
    turnLeft();
  }
  else if (situation == Stop){
    stop_();
  }
  else if (situation == Break){
    break_();
  }      
}
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
void addinstances(long dis1, long dis2){
  if (left_distances.isEmpty() && right_distances.isEmpty()){
    if (ObjectDetect != false){
      left_distances.dequeue();
      right_distances.dequeue();
    }
    left_distances.enqueue(dis1);
    right_distances.enqueue(dis2);
  }
  else{
    left_distances.dequeue();
    right_distances.dequeue();
    left_distances.enqueue(dis1);
    right_distances.enqueue(dis2);    
  }
}
  /*
  if (left_distances.isEmpty() && right_distances.isEmpty()) ObjectDetect = false;
  
  if(ObjectDetect != false){
    left_distances.enqueue(dis1);
    right_distances.enqueue(dis2);
  }
  else{
    left_distances.dequeue();
    right_distances.dequeue(); 
   }
  Serial.println(left_distances.count());
}
*/
void TrunRight(long disl,long disr){
  
  if(disl ==0 && disr ==0 ){
    rightmoving = true;
    Serial.println(left_distances.peek());
    Serial.println(right_distances.peek());
    if((left_distances.peek() >= 30 && right_distances.peek() >= 30)){
      display.println("setting trun right");
      Movement = TurnRight;
      //tuning prameter
      Duty_left = 255;
      Duty_right = 0;

      dis_l = 50;
      dis_r = 50;
      Serial.println(Duty_left);
      Serial.println(Duty_right);
      Serial.println(disl);
      Serial.println(disr);
    }
    else if ((left_distances.peek() < 30 && right_distances.peek() < 30 ) or (ObjectDetect = true)){
      display.println("it is failable");
   //tuning prameter
    }
    //else if (disl >50 disr >50) rightmoving = false;
  }
  addinstances(disl,disr);
}


void followingPeople(long l_dis, long r_dis){

    Serial.println("input data");
    Serial.println(l_dis);
    Serial.println(r_dis);

    if ((0< l_dis && l_dis< 30) && (0< r_dis && l_dis< 30)){

      if (isfirstdata(left_distances) && isfirstdata(right_distances) ){ 
        Movement = Stop;
        //display.println("it is first data");
      }
      else if(left_distances.peek() >= 30 && right_distances.peek() >= 30 ){
        Movement = Forward;
        Serial.println("pre_left");
        Serial.println(left_distances.peek());
        Serial.println("pre_right");
        Serial.println(right_distances.peek());
        Serial.println("Duty_left");
        Serial.println(Duty_left);
        Serial.println("Duty_right");
        Serial.println(Duty_right);
      }
      else if(left_distances.peek() < 30 && right_distances.peek() < 30 ){
        Movement = Stop;
        //display.println("object detect");
        ObjectDetect = true;
      }
    }
    else if ((30 <=l_dis && l_dis <50 )&&( 30 <= r_dis && r_dis <50)){
        Movement = Forward;
        //display.println("set forward small moving");
        Duty_left = 205;
        Duty_right = 200;
        Serial.println(Duty_left);
        Serial.println(Duty_right);
    }
    else if ((50 <=l_dis)&&( 50 <= r_dis)){
        //display.println("set forward full moving");
        Movement = Forward;
        Duty_left = 234;
        Duty_right = 255;
        Serial.println(Duty_left);
        Serial.println(Duty_right);
    }
}
  
  
void exit(void){
  while(1){
  }
}
/*
static long dis1[] = {28,50,0,28,10,10,10,40,40,40};
static long dis2[] = {28,50,0,28,10,10,10,40,40,40};

static long dis1[] = {70,60,45,40,50,0,0,0,0,70};
static long dis2[] = {70,60,45,40,50,0,0,0,0,70};
*/
void loop(){

  //display.setCursor(0,1);
  
  dis_l= sonar[Left].ping_cm();
  dis_r = sonar[Right].ping_cm();
  
  display.print("L: ");
  display.print(dis_r);
  display.print("R: ");
  display.println(dis_l);
  
  /*
  if (i>sizeof(dis1)/sizeof(int)){
    exit();  
  }

  //Serial.println(sizeof(dis1)/sizeof(long));
  dis_l = dis1[i];
  dis_r = dis2[i];
  */
  //Serial.println("data?");
  //Serial.println(dis_l);
  //Serial.println(dis_r);
  delay(1000);
  
  followingPeople(dis_l,dis_r);
                                     // SmartCart forward operation full Speed
  //Motor 2 Forward
/*
  //SmartCart Forward function2
  setMotorControl(Forward, 51,255);
  delay(300);                                   //SmartCart forward operation 1/5 Speed                        .
  
  //SMart Cart TurnRight325
  setMotorControl(TurnRight, 255,255);
  delay(300);                                   //SmartCart TurnRight operation 0.3s
                                   
  //SMart Cart TurnLeft
  setMotorControl(TurnLeft, 255,255);
  delay(300);                                    //SmartCart TrunLeft operation 0.3s
    
  //Motor 1,2 PWM control
  for (int PWM_test = 100; PWM_test<255; PWM_test=PWM_test+50){
    setMotorControl(Backward, PWM_test,PWM_test);
    delay(300);                                  //Smart Backward operation with PWM added
  }

  //Motor1,2 stop
  setMotorControl(Stop, 255,255);
  
  delay(3000);
  */
  TrunRight(dis_l,dis_r);
  Serial.println("is it?");
  addinstances(dis_l,dis_r);
  Serial.println(left_distances.count());
  setMotorControl(Movement, Duty_left,Duty_right);
  delay(100); 
  display.clear();
  //i = i+1;
}
