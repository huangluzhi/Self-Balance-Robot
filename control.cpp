// #include <string>
#include <iostream>
#include <Wire.h>
#include <MsTimer2.h>
#include <math.h>
#include <ArduinoJson.h>
#include "DataScope_DP.h"
#include "control.h"

using namespace std;

//函数说明：将单精度浮点数据转成4字节数据并存入指定地址
//附加说明：用户无需直接操作此函数
//target:目标单精度数据
//buf:待写入数组
//beg:指定从数组第几个元素开始写入
//函数无返回
void BalanceCtr::Float2Byte(float *target,unsigned char *buf,unsigned char beg)
{
    unsigned char *point;
    point = (unsigned char*)target;    //得到float的地址
    buf[beg]   = point[0];
    buf[beg+1] = point[1];
    buf[beg+2] = point[2];
    buf[beg+3] = point[3];
}

//函数说明：将待发送通道的单精度浮点数据写入发送缓冲区
//Data：通道数据
//Channel：选择通道（1-10）
//函数无返回
void BalanceCtr::DataScope_Get_Channel_Data(float Data,unsigned char Channel)
{
  if ( (Channel > 10) || (Channel == 0) ) return;  //通道个数大于10或等于0，直接跳出，不执行函数
  else
  {
    switch (Channel)
    {
      case 1:  Float2Byte(&Data,DataScope_OutPut_Buffer,1); break;
      case 2:  Float2Byte(&Data,DataScope_OutPut_Buffer,5); break;
      case 3:  Float2Byte(&Data,DataScope_OutPut_Buffer,9); break;
      case 4:  Float2Byte(&Data,DataScope_OutPut_Buffer,13); break;
      case 5:  Float2Byte(&Data,DataScope_OutPut_Buffer,17); break;
      case 6:  Float2Byte(&Data,DataScope_OutPut_Buffer,21); break;
      case 7:  Float2Byte(&Data,DataScope_OutPut_Buffer,25); break;
      case 8:  Float2Byte(&Data,DataScope_OutPut_Buffer,29); break;
      case 9:  Float2Byte(&Data,DataScope_OutPut_Buffer,33); break;
      case 10: Float2Byte(&Data,DataScope_OutPut_Buffer,37); break;
    }
  }
}

//函数说明：生成 DataScopeV1.0 能正确识别的帧格式
//Channel_Number，需要发送的通道个数
//返回发送缓冲区数据个数
//返回0表示帧格式生成失败
unsigned char BalanceCtr::DataScope_Data_Generate(unsigned char Channel_Number)
{
  if ( (Channel_Number > 10) || (Channel_Number == 0) ) { return 0; }  //通道个数大于10或等于0，直接跳出，不执行函数
  else
  {
  DataScope_OutPut_Buffer[0] = '$';  //帧头

  switch(Channel_Number)
  {
    case 1:   DataScope_OutPut_Buffer[5]  =  5; return  6; break;
    case 2:   DataScope_OutPut_Buffer[9]  =  9; return 10; break;
    case 3:   DataScope_OutPut_Buffer[13] = 13; return 14; break;
    case 4:   DataScope_OutPut_Buffer[17] = 17; return 18; break;
    case 5:   DataScope_OutPut_Buffer[21] = 21; return 22; break;
    case 6:   DataScope_OutPut_Buffer[25] = 25; return 26; break;
    case 7:   DataScope_OutPut_Buffer[29] = 29; return 30; break;
    case 8:   DataScope_OutPut_Buffer[33] = 33; return 34; break;
    case 9:   DataScope_OutPut_Buffer[37] = 37; return 38; break;
    case 10:  DataScope_OutPut_Buffer[41] = 41; return 42; break;
  }
  }
  return 0;
}

//读取左轮编码器数据
//正转+1，反转-1
void BalanceCtr::READ_ENCODER_L()
{
  if(digitalRead(ENCODER_L)==LOW)
  {
    if(digitalRead(DIRECTION_L)==HIGH)
      Velocity_L++;
    else
      Velocity_L--;
  }
  else
  {
    if(digitalRead(DIRECTION_L)==LOW)
      Velocity_L++;
    else
      Velocity_L--;
  }
}

//读取左轮编码器数据
//正转+1，反转-1
void BalanceCtr::READ_ENCODER_R() //读取右轮编码器数据
{
  if(digitalRead(ENCODER_R)==LOW)
  {
    if(digitalRead(DIRECTION_R)==LOW)
      Velocity_R++;
    else
      Velocity_R--;
  }
  else
  {
    if(digitalRead(DIRECTION_R)==HIGH)
      Velocity_R++;
    else
      Velocity_R--;
  }
}

//向MPU中输入一个字节的数据
void BalanceCtr::Write_MPUData(int addre,unsigned char Data)
{
  Wire.beginTransmission(MPU);
  Wire.write(addre);
  Wire.write(Data);
  Wire.endTransmission(true);
}

//从MPU6050中读出全部数据，储存到Measure_Data[Read_Num]中
void BalanceCtr::Read_AccGyr_Data()
{
  Wire.beginTransmission(MPU);
  Wire.write(0x3B);
  Wire.requestFrom(MPU,Data_Num*2,true);
  Wire.endTransmission(true);
  for(int i=0;i<Data_Num;i++)
    Measure_Data[i]=Wire.read()<<8|Wire.read();
}

//计算出角度值
void BalanceCtr::GetPitch()
{
  Angle_Last=Angle_New; //存储上一次的角度值
  Angle_Acc=atan(Acc_New[0]/Acc_New[2]);  //通过加速度得到当前角度值（弧度制）
  Angle_Acc*=57.3;  //转换为角度制
  Angle_Gyr=Angle_Last+(-Gyr_New[1])*dt;  //通过角速度得到当前角度值
  Angle_New=F_AccGyr*Angle_Acc+(1-F_AccGyr)*Angle_Gyr;  //加速度和角速度融合
}

//对读数进行换算，并转化成浮点型物理量
void BalanceCtr::Converse()
{
  for(int i=0;i<3;i++)
    Acc_Last[i]=Acc_New[i]; //对数据进行存储
  for(int i=0;i<3;i++)
    Gyr_New[i]=Gyr_Last[i]; //对数据进行存储
  Acc_New[0]=(float)((Measure_Data[0])/16384.0+0.15); //加速度以g为单位,z轴正加速度方向向下
  Acc_New[1]=(float)((Measure_Data[1])/16384.0);
  Acc_New[2]=(float)((Measure_Data[2])/16384.0);
  Gyr_New[0]=(float)((Measure_Data[4])/131.07+1.74); //角加速度以度每秒为单位
  Gyr_New[1]=(float)((Measure_Data[5])/131.07-0.55);
  Gyr_New[2]=(float)((Measure_Data[6])/131.07+0.26);
  for(int i=0;i<3;i++)
    Acc_New[i]=L_Acc*Acc_New[i]+(1-L_Acc)*Acc_Last[i];  //加速度一阶滤波
  for(int i=0;i<3;i++)
    Gyr_New[i]=L_Gyr*Gyr_New[i]+(1-L_Gyr)*Gyr_Last[i];  //角速度一阶滤波
}

//计算直立的PWM值，5ms一次
void BalanceCtr::Set_Balance_Pwm()
{
  Balance_Pwm=Balance_Kp*Angle_New+Balance_Kd*Gyr_New[1]; //计算出直立的PWM值
  // Balance_Pwm+=1.0*(Balance_Pwm-(Velocity/10));
  Balance_Vel=Balance_Pwm*10;
}

//计算速度的PWM值，40ms一次
void BalanceCtr::Set_Velocity_Pwm()
{
  Velocity_Bias=Velocity_L*9.6+Velocity_R*9.6-0.0;  //计算此时的速度偏差值，9.6为一任意系数，增大编码器读数
  Velocity*=0.7;  //速度值一阶低通滤波
  Velocity+=Velocity_Bias*0.3;
  Velocity_Integral+=(Balance_Vel-Velocity);  //计算速度值的积分
  if(Velocity_Integral>10000) //设置积分上限
    Velocity_Integral=10000;
  if(Velocity_Integral<-10000)  //设置积分下限
    Velocity_Integral=-10000;
  // Velocity_Integral=0;
  Velocity_Pwm=(Balance_Vel-Velocity)*Velocity_Kp+Velocity_Integral*Velocity_Ki;  //计算出速度的PWM值
  Velocity_L=0; //左轮编码器读数清零
  Velocity_R=0; //右轮编码器读数清零
}

//为左右轮赋PWM值
void BalanceCtr::Coor_Motor()
{
  L_Pwm=Balance_Pwm; //为左轮赋值
  R_Pwm=Balance_Pwm; //为右轮赋值
  // L_Pwm=Balance_Pwm+Velocity_Pwm; //为左轮赋值
  // R_Pwm=Balance_Pwm+Velocity_Pwm; //为右轮赋值
  Limit_Pwm();  //PWM限幅
}

//PWM限幅
void BalanceCtr::Limit_Pwm()
{
  const int L_Pwm_DZ = 90;
  const int R_Pwm_DZ = 100;
  if(L_Pwm>0)
  {
    if (L_Pwm < L_Pwm_DZ && abs(Velocity_L) <= 40) {
      L_Pwm*=3;
      L_Pwm=min(L_Pwm, L_Pwm_DZ);
    }
      // L_Pwm+=1.0*(40-Velocity_L);  //90为左轮正转的死区PWM，具体数值会由于硬件有所改变
      // L_Pwm+=10;  //90为左轮正转的死区PWM，具体数值会由于硬件有所改变
    if(L_Pwm>255)
      L_Pwm=255;
  }
  if(L_Pwm<0)
  {
    if (L_Pwm > 0-L_Pwm_DZ && abs(Velocity_L) <= 40) {
      L_Pwm*=3;
      L_Pwm=max(L_Pwm, -L_Pwm_DZ);
    }
      // L_Pwm-=1.0*(40-Velocity_L);  //90为左轮正转的死区PWM，具体数值会由于硬件有所改变
    // L_Pwm-=50;  //90为左轮反转的死区PWM，下同
    if(L_Pwm<-255)
      L_Pwm=-255;
  }
  if(R_Pwm>0)
  {
    if (R_Pwm < R_Pwm_DZ && abs(Velocity_R) <= 40) {
      R_Pwm*=3;
      R_Pwm=min(R_Pwm, R_Pwm_DZ);
    }
      // R_Pwm+=1.0*(40-Velocity_R);
    if(R_Pwm>255)
      R_Pwm=255;
  }
  if(R_Pwm<0)
  {
    if (R_Pwm > 0-R_Pwm_DZ && abs(Velocity_R) <= 40) {
      R_Pwm*=3;
      R_Pwm=max(R_Pwm, -R_Pwm_DZ);
    }
      // R_Pwm-=1.0*(40-Velocity_R);
    if(R_Pwm<-255)
      R_Pwm=-255;
  }
  analogWrite(PWM_L,abs(L_Pwm));
  analogWrite(PWM_R,abs(R_Pwm));
}

//通过计算出来的PWM设置正反转，视接线方法而定
void BalanceCtr::Set_Direction()
{
  if(L_Pwm<0)
    digitalWrite(IN3,LOW),digitalWrite(IN4,HIGH);
  else
    digitalWrite(IN3,HIGH),digitalWrite(IN4,LOW);
  if(R_Pwm<0)
    digitalWrite(IN1,LOW),digitalWrite(IN2,HIGH);
  else
    digitalWrite(IN1,HIGH),digitalWrite(IN2,LOW);
}

//定时器中断函数，5ms一次
void BalanceCtr::Kernel()
{
  sei();  //开启全局中断
  Read_AccGyr_Data(); //获取加速度及角速度的值
  Converse(); //对获取的数据进行换算
  GetPitch(); //计算出Pitch角
  Set_Balance_Pwm();  //计算直立的PWM值
  if(++Velocity_Count>=1) //每到40ms时进入此函数
  {
    Set_Velocity_Pwm(); //计算速度的PWM值
    Velocity_Count=0; //速度调控计数位清零
  }
  Set_Direction();  //设置方向
  Coor_Motor(); //为电机赋PWM
  DataScope_Get_Channel_Data(Acc_New[0],1); //下为虚拟示波器上位机程序，培训时会提到
  DataScope_Get_Channel_Data(Acc_New[1],2);
  DataScope_Get_Channel_Data(Acc_New[2],3);
  DataScope_Get_Channel_Data(Gyr_New[0],4);
  DataScope_Get_Channel_Data(Gyr_New[1],5);
  DataScope_Get_Channel_Data(Gyr_New[2],6);
  DataScope_Get_Channel_Data(Angle_New,7);
  DataScope_Get_Channel_Data(Balance_Pwm,8);
  DataScope_Get_Channel_Data(Angle_Acc,9);
  DataScope_Get_Channel_Data(Angle_Gyr,10);
//  DataScope_Get_Channel_Data(R_Pwm,6);
  Send_Count=DataScope_Data_Generate(10);
  // for(int i=0;i<Send_Count;i++)
  //   Serial.write(DataScope_OutPut_Buffer[i]);
  // delay(50);
}


unsigned char BalanceCtr::readSerial() {
    while (!Serial.available())
        ;
    return Serial.read();
}

string BalanceCtr::readSerialStr() {
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