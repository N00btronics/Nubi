#include <Servo.h> //Servo Library
#include <SoftwareSerial.h> //Serial library

//LED Setup
const int eye_TR = 4; //Top right eye pin
const int eye_BR = 6; //Bottom right eye pin
const int eye_TL = 7; //Top left eye pin
const int eye_BL = 12; //Bottom left eye pin

//Motor Setup
int speed_val_1 = 200; //Set Speed for motor 1
int speed_val_2 = 255; //Set Speed for motor 2
const int En_M1 = 3; //motor 1 EN pin
const int En_M2 = 5; //motor 2 EN pin
const int M1_A = A0; //motor 1 V+ pin
const int M1_B = A1; //motor 1 V- pin
const int M2_A = A2; //motor 2 V+ pin
const int M2_B = A3; //motor 2 V- pin

//Ultrasonic Sensor
const int UltraSonic_Trig = 10; //Ultrasonic sensor Trig pin
const int UltraSonic_Echo = 2; //Ultrasonic sensor Echo pin
bool trigger_state = LOW; //State of sensor trigger
int distance = 0; //Distance measured

//Servo
const int servo_pin = 11; //Servo signal pin
Servo servo; //Servo object
int servo_correction = 0; //angle correction of servo from center
int servo_pos; //position of servo

//Timer
unsigned long previousMillis = 0;  // will store time recorded in previous loop
const long interval = 100;  // interval at which to perform function

//Robot
int bump_distance = 20;
int lookRight_angle = 15;
int lookLeft_angle = 165;
int lookStraight_angle = 90;

//Autonomous
int servoDelay = 5;



void setup() {
  //Setup eyes
  pinMode(eye_TR, OUTPUT);
  pinMode(eye_BR, OUTPUT);
  pinMode(eye_TL, OUTPUT);
  pinMode(eye_BL, OUTPUT);

  // Setup motors
  pinMode(En_M1, OUTPUT);
  pinMode(En_M2, OUTPUT);
  pinMode(M1_A, OUTPUT);
  pinMode(M1_B, OUTPUT);
  pinMode(M2_A, OUTPUT);
  pinMode(M2_B, OUTPUT);

  //Setup ultrasonic sensor
  pinMode(UltraSonic_Trig, OUTPUT);
  pinMode(UltraSonic_Echo, INPUT);
  digitalWrite(UltraSonic_Trig, LOW);

  //Setup servo
  servo.attach(servo_pin);

  //Setup Serial  
  Serial.begin(9600);
  mySerial.begin(9600); //BLE Serial

  //On Startup
  servo.write(lookStraight_angle + servo_correction); //Set head to face forward
  look_happy(); //Set eyes to look happy
  delay(3000);
}

int get_distance(){
  digitalWrite(UltraSonic_Trig, LOW);
  delay(2);
  digitalWrite(UltraSonic_Trig, HIGH); //Send out a trigger
  delay(10);
  digitalWrite(UltraSonic_Trig, LOW);
  int timing = pulseIn(UltraSonic_Echo, HIGH); //Count the time it took receive the trigger
  int distance = (timing * 0.034) / 2; //Calculate the distance based on speed of sound
  //Serial.print("Distance: ");
  //Serial.println(get_distance());
  return distance;
}

void move_forward(){
  analogWrite(En_M1, speed_val_1);
  analogWrite(En_M2, speed_val_2);

  digitalWrite(M1_A, HIGH);
  digitalWrite(M1_B, LOW);

  digitalWrite(M2_A, HIGH);
  digitalWrite(M2_B, LOW);
}

void move_backward(){
  analogWrite(En_M1, speed_val_1/2);
  analogWrite(En_M2, speed_val_2/2);

  digitalWrite(M1_A, LOW);
  digitalWrite(M1_B, HIGH);

  digitalWrite(M2_A, LOW);
  digitalWrite(M2_B, HIGH);
}

void move_stop(){
  analogWrite(En_M1, speed_val_1);
  analogWrite(En_M2, speed_val_2);

  digitalWrite(M1_A, LOW);
  digitalWrite(M1_B, LOW);

  digitalWrite(M2_A, LOW);
  digitalWrite(M2_B, LOW);
}

void move_left(){
  analogWrite(En_M1, speed_val_1/2);
  analogWrite(En_M2, speed_val_2/2);

  digitalWrite(M1_A, HIGH);
  digitalWrite(M1_B, LOW);

  digitalWrite(M2_A, LOW);
  digitalWrite(M2_B, HIGH);
}

void move_right(){
  analogWrite(En_M1, speed_val_1/2);
  analogWrite(En_M2, speed_val_2/2);

  digitalWrite(M1_A, LOW);
  digitalWrite(M1_B, HIGH);

  digitalWrite(M2_A, HIGH);
  digitalWrite(M2_B, LOW);
}

void look_normal(){
  digitalWrite(eye_TR, HIGH);
  digitalWrite(eye_BR, HIGH);
  digitalWrite(eye_TL, HIGH);
  digitalWrite(eye_BL, HIGH);
}

void look_happy(){
  digitalWrite(eye_TR, HIGH);
  digitalWrite(eye_BR, LOW);
  digitalWrite(eye_TL, HIGH);
  digitalWrite(eye_BL, LOW);
}

void look_sad(){
  digitalWrite(eye_TR, LOW);
  digitalWrite(eye_BR, HIGH);
  digitalWrite(eye_TL, LOW);
  digitalWrite(eye_BL, HIGH);
}

int move_servo(int current_position, int desired_position, int servo_delay){
  if(current_position > desired_position){
    for (int pos = current_position; pos >= desired_position + servo_correction; pos--){
      servo.write(pos);
      delay(servo_delay);
      return pos;
    }
  }else if(current_position < desired_position){
    for (int pos = current_position; pos >= desired_position + servo_correction; pos++){
      servo.write(pos);
      delay(servo_delay);
      return pos;
    }
  }else{
    return current_position;
  }
}

void AutoControl(){
  int direction1 = 0; //Direction when facing clockwise
  int direction2 = 0; //Direction when facing counter-clockwise

  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {
    distance = get_distance(); //Grab the distance each interval
    previousMillis = currentMillis; //save lasttime loop was ran
  }

  /* 
  When robot senses an obstacle that is less than 20 CM, 
  begin the animated sequence to devide on a direction to move to
  */
  if(distance > 0 and distance <= bump_distance){
    move_stop();
    look_sad();
    delay(2000);

    look_normal();
    move_backward();
    delay(500);

    move_stop();
    delay(1000);

    //Look right, change the speed by changing the delay
    servo_pos = move_servo(servo_pos, lookRight_angle, servoDelay);
    delay(500); //Pause before taking measurement from the sensor
    direction1 = distance;
    delay(250);

    //Look Left, change the speed by changing the delay
    servo_pos = move_servo(servo_pos, lookLeft_angle, servoDelay);
    delay(500); //Pause before taking measurement from the sensor
    direction2 = distance;
    delay(250);

    //Look Straight, change the speed by changing the delay
    servo_pos = move_servo(servo_pos, lookStraight_angle, servoDelay);

    delay(100);
    look_happy();
    delay(1000);

    if(direction1 <= bump_distance && direction2 <= bump_distance){
      move_right();
      delay(600);
    }
    else if(direction1 >= direction2){
      move_right();
      delay(400); //delay for making a turn, modify if robot turns too fast or not fast enough
    }else{
      move_left();
      delay(400); //delay for making a turn, modify if robot turns too fast or not fast enough
    }

    move_stop();
    delay(500);
    look_normal();
    delay(500);

  }else{
    move_forward();
    look_normal();
  }
}
  }else{

    //When a command is done being received, excute the action
    servo.write(servo_pos + servo_correction);

    if(forward && distance >= bump_distance){
      move_forward();
    }else if(right){
      move_right();
    }else if(left){
      move_left();
    }else if(backward){
      move_backward();
    }else{
      move_stop();
    }
  }
}

void loop() {
  AutoControl();
}
