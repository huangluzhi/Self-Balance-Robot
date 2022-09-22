#include <Wire.h>
#include <MsTimer2.h>
#include <math.h>
#include <ArduinoJson.h>
#include "DataScope_DP.h"
#include "control.h"
// #include <string>
#include <Arduino.h>

BalanceCtr balanceCtr;

void setup() {
  // BalanceCtr balanceCtr;
  // put your setup code here, to run once:
  pinMode(IN1,OUTPUT);  //电机及PWM引脚为输出模式
  pinMode(IN2,OUTPUT);
  pinMode(IN3,OUTPUT);
  pinMode(IN4,OUTPUT);
  pinMode(PWM_L,OUTPUT);
  pinMode(PWM_R,OUTPUT);
  pinMode(ENCODER_L,INPUT); //编码器引脚为输入模式
  pinMode(ENCODER_R,INPUT);
  pinMode(DIRECTION_L,INPUT);
  pinMode(DIRECTION_R,INPUT);
  Serial.begin(9600); //开启串口
  Wire.begin(); //开启I2C通信
  delay(1500);  //延时以等待通信系统全部打开
  balanceCtr.Write_MPUData(0x6B,0);  //激活MPU6050
  delay(20);
  MsTimer2::set(2,balanceCtr.Kernel);  //设置5ms的定时器中断，中断函数为Kernel
  MsTimer2::start();  //打开定时器中断
  attachInterrupt(1,balanceCtr.READ_ENCODER_L,CHANGE); //打开外部中断，用于编码器计数，下同
  attachInterrupt(0,balanceCtr.READ_ENCODER_R,CHANGE);
}

unsigned char readSerial() {
    while (!Serial.available())
        ;
    return Serial.read();
}

string readSerialStr() {
  unsigned char c=readSerial();
  int dataLen = 0;
  string data_serial
  if (c == 233)
  {
    for (int i = 0; i < 4; i++){
        unsigned char t = readSerial();
        dataLen = dataLen * 255 + t;
    }
    for (int i = 0; i < dataLen; i++)
    {
        data_serial += ((char)(readSerial()));
    }
  }
  return data_serial;
}

void loop() {
  if (Serial.available()) {
    string dataStr = readSerialStr();
    if (!dataStr.empty()){
      DynamicJsonDocument dataJson(1024);
      deserializeJson(dataJson, dataStr);
      balanceCtr.speed = dataJson["speed"].as<int>();
      balanceCtr.turn = dataJson["turn"].as<int>();
      Serial.println("speed: " + string(balanceCtr.speed));
      Serial.println("turn: " + string(balanceCtr.turn));
      if (dataJson["bal_Kp"].as<int>() != 0) 
        balanceCtr.Balance_Kp = dataJson["bal_Kp"].as<int>();
      if (dataJson["bal_Kd"].as<int>() != 0) 
        balanceCtr.Balance_Kd = dataJson["bal_Kd"].as<int>();
      if (dataJson["spd_Kp"].as<int>() != 0) 
        balanceCtr.Velocity_Kp = dataJson["spd_Kp"].as<int>();
      if (dataJson["spd_Kd"].as<int>() != 0) 
        balanceCtr.Velocity_Ki = dataJson["spd_Kd"].as<int>();
    }
  }

  // put your main code here, to run repeatedly:
  // Serial.print("L_Pwm: ");
  // Serial.print(L_Pwm);
  // Serial.print("\t");
  // Serial.print("R_Pwm: ");
  // Serial.print(R_Pwm);
  // Serial.print("\t");
  // Serial.print("Balance_Pwm: ");
  // Serial.print(Balance_Pwm);
  // Serial.print("\t");
  // Serial.print("Velocity: ");
  // Serial.print(Velocity);
  // Serial.print("\t");
  // Serial.print("Velocity_Integral: ");
  // Serial.print(Velocity_Integral);
  // Serial.print("\t");
  // Serial.print("\n");
}
