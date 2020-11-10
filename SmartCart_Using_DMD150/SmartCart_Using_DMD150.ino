//=======================================
//Perception///
//=======================================
//Ultrasounds Sensor
const int echoPIN = 8;         //define Ultsenser echo
const int trigPIN = 10;         //define Ultsenser triger
const double Ultsenser_perCenter = 2941.17647;  //100000/340 [cm/us]

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
#define Motor1 0
#define Motor2 1

// Define Situation
const int Forward = 0;
const int Backward = 1;
const int TurnRight = 2;
const int TurnLeft = 3;
const int Stop = 4;
const int Break = 5;

//unsigned char MotorSituation[] = {Forward,Backward,TurnRight,TurnLeft,Stop,Break};

// Define Motor Speed (PWM is Analog signal 0~255)
unsigned int PWM = 255;
const int TIme = 10;
/*
  double measureDistance()
  {
  double distance = 0;
  double cycletime = 0;

  digitalWrite(trigPIN,HIGH);
  delayMicroseconds(TIme);

  //define measured distance
  cycleTime = pulseIn(echoPIN,HIGH);
  distance = cycleTime/(2*Ultsenser_perCenter)

  //print distance
  Serial.print("Distance:");
  Serial.print(distance);
  Serial.println("m");
  return distance
  }
*/

void forward()
{
  digitalWrite(motor1IN1, HIGH);
  digitalWrite(motor1IN2, LOW);
  digitalWrite(motor2IN1, HIGH);
  digitalWrite(motor2IN2, LOW);
  



}

void backward()
{
  digitalWrite(motor1IN1, LOW);
  digitalWrite(motor1IN2, HIGH);

  digitalWrite(motor2IN1, LOW);
  digitalWrite(motor2IN2, HIGH);
}

void turnRight()
{
  digitalWrite(motor1IN1, HIGH);
  digitalWrite(motor1IN2, LOW);

  digitalWrite(motor2IN1, LOW);
  digitalWrite(motor2IN2, HIGH);
}

void turnLeft()
{
  digitalWrite(motor1IN1, HIGH);
  digitalWrite(motor1IN2, LOW);

  digitalWrite(motor2IN1, LOW);
  digitalWrite(motor2IN2, HIGH);
}

void stop_()
{
  digitalWrite(motor1IN1, LOW);
  digitalWrite(motor1IN2, LOW);

  digitalWrite(motor2IN1, LOW);
  digitalWrite(motor2IN2, LOW);
}

void break_()
{
  digitalWrite(motor1IN1, HIGH);
  digitalWrite(motor1IN2, HIGH);

  digitalWrite(motor2IN1, HIGH);
  digitalWrite(motor2IN2, HIGH);
}

void setMotorControl(unsigned int situation, unsigned int PWM_Value1,unsigned int PWM_Value2)
{
  //Motor Speed control = PWM
  analogWrite(PWM_ENA, PWM_Value1);
  analogWrite(PWM_ENB, PWM_Value2);

  if (situation == Forward) {
    forward();
  }
  else if (situation == Backward) {
    backward();
  }
  else if (situation == TurnRight) {
    turnRight();
  }
  else if (situation == TurnLeft) {
    turnLeft();
  }
  else if (situation == Stop) {
    stop_();
  }
  else if (situation == Break) {
    break_();
  }
}

void setup() {
  //define Seiral function : connecting to Host Moniter
  Serial.begin(9600);

  //set Ultrasounds SensorConfig
  pinMode(trigPIN , OUTPUT);
  pinMode(echoPIN , INPUT);

  // set MotorConifg
  pinMode(motor1IN1 , OUTPUT);
  pinMode(motor1IN2 , OUTPUT);
  pinMode(motor2IN1 , OUTPUT);
  pinMode(motor2IN2 , OUTPUT);

  pinMode(PWM_ENA , OUTPUT);
  pinMode(PWM_ENB , OUTPUT);
  unsigned int currentPWM = 0;

  //setting PWM init
  analogWrite(PWM_ENA, currentPWM);
  analogWrite(PWM_ENB, currentPWM);
}

void loop() {
  //SmartCart Forward function1
  //setMotorControl(break_, 255,255);
  //delay(100);
  
  setMotorControl(Forward, 212  ,255);
  delay(300);                                    // SmartCart forward operation full Speed

  /*
    //Motor1,2 stop
    setMotorControl(Stop, 255);
    delay(3000);

    //SmartCart Forward function2
    setMotorControl(Forward, 200);
    delay(300);                                   //SmartCart forward operation 1/5 Speed

     //SMart Cart TurnLeft
    setMotorControl(TurnLeft, 255);
    delay(300);
  */
}
