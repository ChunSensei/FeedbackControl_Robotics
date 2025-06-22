#include <Servo.h>

Servo sv1;
Servo sv2;

#define PWM1 6
#define INA1 11
#define INB1 10

#define PWM2 5
#define INA2 9
#define INB2 8

int sensorPin[4] = {5 , 4 , 3 , 2};
int sensorValue[4] = {0, 0, 0, 0};
int minRead[4] = {0, 0, 0, 0};
int midRead[4] = {0, 0, 0, 0};
int maxRead[4] = {0, 0, 0, 0};
int lastError = 0;
int last_value = 0;

void setup() {
  intitial();
  //motorControl(130 , -130);delay(2000);
  //readSensor();
  cross_check(110 , 0.07 , 1.5);
  cross_check(110 , 0.07 , 1.5);
  cross_check(110 , 0.07 , 1.5);
  cross_check(110 , 0.07 , 2.1);
  L();
  cross_check(110 , 0.07 , 1.5);
  cross_check(110 , 0.07 , 1.5);
  L();
  STOPPID(110 , 0.05 , 1.5);
}

void loop() {
  STOP();
}

void intitial() {
  Serial.begin(9600);

  pinMode(INA1 , OUTPUT);
  pinMode(INB1 , OUTPUT);
  pinMode(PWM1 , OUTPUT);
  pinMode(INA2 , OUTPUT);
  pinMode(INB2 , OUTPUT);
  pinMode(PWM2 , OUTPUT);

  analogWrite(PWM1 , 255);
  digitalWrite(INA1 , 1);
  digitalWrite(INB1 , 1);
  analogWrite(PWM2 , 255);
  digitalWrite(INA2 , 1);
  digitalWrite(INB2 , 1);

  sv1.attach(1);
  sv2.attach(7);
  
  sv2.write(0); delay(1000);
  sv2.write(90);
  displayValue(2000);
  sv2.write(0); delay(1000);
  sv2.write(90);
  delay(1000);
}

void motorControl(int Speed1 , int Speed2) {
  if (Speed1 >= 0) {
    analogWrite(PWM1 , abs(Speed1));
    digitalWrite(INA1 , 1);
    digitalWrite(INB1 , 0);
  }
  else {
    analogWrite(PWM1 , abs(Speed1));
    digitalWrite(INA1 , 0);
    digitalWrite(INB1 , 1);
  }
  if (Speed2 >= 0) {
    analogWrite(PWM2 , abs(Speed2));
    digitalWrite(INA2 , 1);
    digitalWrite(INB2 , 0);
  }
  else {
    analogWrite(PWM2 , abs(Speed2));
    digitalWrite(INA2 , 0);
    digitalWrite(INB2 , 1);
  }
}

void STOP(){
  analogWrite(PWM1 , 255);
  digitalWrite(INA1 , 1);
  digitalWrite(INB1 , 1);
  analogWrite(PWM2 , 255);
  digitalWrite(INA2 , 1);
  digitalWrite(INB2 , 1);
}

void readSensor() {
  while (1) {
    for (int i = 0 ; i < 4 ; i ++) {
      Serial.print("sensor ");
      Serial.print(i);
      Serial.print(" value = ");
      Serial.println(analogRead(sensorPin[i]));
    }
    Serial.println("////////////////////////////");
    delay(350);
  }
}

void Read() {
  for (int i = 0; i < 4; i++) {
    sensorValue[i] = analogRead(sensorPin[i]);
  }
}

void readCarlib() {
  Read();
  for (int j = 0; j < 4; j++) {
    sensorValue[j] = map(sensorValue[j], minRead[j], maxRead[j], 0, 1000);
    sensorValue[j] = (sensorValue[j] < 0 ? 0 : sensorValue[j]);
    sensorValue[j] = (sensorValue[j] > 1000 ? 1000 : sensorValue[j]);
  }
}

int ReadLine() {
  readCarlib();
  int on_line[4] , idx = 0;
  unsigned long avg;
  unsigned int sum = 0;
  for (int k = 0 ; k < 4 ; k++) {
    if (sensorValue[k] < midRead[k]){
      on_line[k] = 1;
      sum += on_line[k];
      avg += (long)k * 1000;
      idx = k;
    }
  }
  if (sum == 0) {
    if (last_value < 1500) last_value = 0;
    else last_value = 3000;
  }
  else {
    if (sum >= 2) {
      last_value = avg / sum;
    }
    else {
      last_value = idx * 1000;
    }
  }
  return last_value;
}

void displayValue(int calibrateTime) {

  for (int sens = 0; sens <= 3 ; sens++) {
    minRead[sens] = 1000;
    maxRead[sens] = 0;
  }
  for (int j = 0; j < calibrateTime; j++) {
    Read();
    for (int i = 0; i < 4; i++) {

      if (j == 0 || maxRead[i] < sensorValue[i])
        maxRead[i] = sensorValue[i];

      if (j == 0 || minRead[i] > sensorValue[i])
        minRead[i] = sensorValue[i];
    }
    delay(2);
  }
  for(int k = 0 ; k < 4 ; k++){
    midRead[k] = (maxRead[k] + minRead[k]) / 2;
  }
}

void stdpid(int BaseSpeed, float Kp, float Kd) {
  int Position  = ReadLine();
  int error = Position - 1500;
  int PowerMotor = (Kp * error) + (Kd * (error - lastError));
  lastError = error;

  if (PowerMotor > BaseSpeed) {
    PowerMotor = BaseSpeed;
  }
  if (PowerMotor < 0 - BaseSpeed) {
    PowerMotor = 0 - BaseSpeed;
  }
  int LeftMotor = BaseSpeed + PowerMotor;
  int RightMotor = BaseSpeed - PowerMotor;
  if (LeftMotor >= 255) LeftMotor = 255;
  if (LeftMotor <= 0) LeftMotor = -130;
  if (RightMotor >= 255) RightMotor = 255;
  if (RightMotor <= 0) RightMotor = -130;
  motorControl(LeftMotor, RightMotor);
}

void STOPPID(int BaseSpeed, float Kp, float Kd) {
  while(1){
    readCarlib();
    if(sensorValue[0] < midRead[0] && sensorValue[3] < midRead[3]){
      break;
    }
    else stdpid(BaseSpeed, Kp, Kd);
  }
  STOP(); delay(70);
}

void cross_check(int BaseSpeed, float Kp, float Kd){
  STOPPID(BaseSpeed , Kp , Kd);
  motorControl(130 , 130); delay(190);
  STOP(); delay(70);
}

void L(){
  motorControl(-130 , 130);
  delay(170);
  while(1){
    readCarlib();
    if(sensorValue[0] < midRead[0]){
      break;
    }
  }
  STOP(); delay(70);
}

void R(){
  motorControl(130 , -130);
  delay(170);
  while(1){
    readCarlib();
    if(sensorValue[3] < midRead[3]){
      break;
    }
  }
  STOP(); delay(70);
}
