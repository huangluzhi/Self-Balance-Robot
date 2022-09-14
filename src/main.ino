#include <Wire.h>
#include <Math.h>
#include <MsTimer2.h>
#include <math.h>
#include "DataScope_DP.h"

#define MPU 0x68  //MPUI2C总线地址

//MPU6050读取及数据处理
const int Data_Num=7; //数据数量
const int Acc_Num=3;  //加速度相关变量数量
const int Gyr_Num=3;  //角速度相关变量数量
const float dt=0.005; //中断时间间隔5ms
static volatile int Measure_Data[Data_Num]; //测量出的数据
static float Acc_New[Acc_Num];  //换算后的加速度值
static float Acc_Last[Acc_Num]; //上一次得到的加速度值
static float Gyr_New[Gyr_Num];  //换算后的角速度值
static float Gyr_Last[Gyr_Num]; //上一次得到的角速度值

//电机（IN1 LOW,IN2 HIGH 为前，IN3 LOW,IN4 HIGH 为后）
#define IN1 7 //右轮
#define IN2 8 //右轮
#define IN3 10 //左轮
#define IN4 11  //左轮
#define PWM_L 9 //左轮PWM引脚
#define PWM_R 6 //右轮PWM引脚
static int L_Pwm,R_Pwm; //左右电机PWM值

//编码器
#define ENCODER_L 3
#define ENCODER_R 2
#define DIRECTION_L 5
#define DIRECTION_R 4
static int Velocity_L=0,Velocity_R=0; //左右编码区数据

//直立PD闭环
static float Balance_Pwm=0; //直立PD得出的PWM值
static float Balance_Kp=15,Balance_Kd=-2.1;//10 -2.1 直立PD的PD系数
static float Balance_Vel=0; //直立PD得出的期望速度值

//速度PI闭环
static float Velocity_Pwm=0;  //速度PI得出的PWM值
static float Velocity_Bias=0; //速度的偏差值
static float Velocity=0;  //当前速度值
static float Velocity_Kp=0.4,Velocity_Ki=Velocity_Kp/200; //速度PI的PI系数
static float Velocity_Integral=0; //速度偏差的积分
static int Velocity_Count=0;  //速度计数位，使速度PI每40ms进行调控

//虚拟示波器
unsigned char DataScope_OutPut_Buffer[42] = {0};     //串口发送缓冲区
unsigned char Send_Count; //发送数据的个数

//加速度及角速度一阶滤波
const float L_Acc=0.3;  //速度一阶滤波系数
const float L_Gyr=0.3;  //角速度一阶滤波系数

//加速度与角速度互补融合
const float F_AccGyr=0.9; //加速度角速度融合置信度

//计算倾角
static float Angle_New=0; //最新得出的角度值
static float Angle_Last=0;  //上一次的角度值
static float Angle_Acc=0; //通过加速度得到的角度值
static float Angle_Gyr=0; //通过角速度得到的角度值

//函数说明：将单精度浮点数据转成4字节数据并存入指定地址 
//附加说明：用户无需直接操作此函数 
//target:目标单精度数据
//buf:待写入数组
//beg:指定从数组第几个元素开始写入
//函数无返回
void Float2Byte(float *target,unsigned char *buf,unsigned char beg)
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
void DataScope_Get_Channel_Data(float Data,unsigned char Channel)
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
unsigned char DataScope_Data_Generate(unsigned char Channel_Number)
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
void READ_ENCODER_L()
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
void READ_ENCODER_R() //读取右轮编码器数据
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
void Write_MPUData(int addre,unsigned char Data)
{
  Wire.beginTransmission(MPU);
  Wire.write(addre);
  Wire.write(Data);
  Wire.endTransmission(true);
}

//从MPU6050中读出全部数据，储存到Measure_Data[Read_Num]中
void Read_AccGyr_Data()
{
  Wire.beginTransmission(MPU);
  Wire.write(0x3B);
  Wire.requestFrom(MPU,Data_Num*2,true);
  Wire.endTransmission(true);
  for(int i=0;i<Data_Num;i++)
    Measure_Data[i]=Wire.read()<<8|Wire.read();
}

//计算出角度值
void GetPitch()
{
  Angle_Last=Angle_New; //存储上一次的角度值
  Angle_Acc=atan(Acc_New[0]/Acc_New[2]);  //通过加速度得到当前角度值（弧度制）
  Angle_Acc*=57.3;  //转换为角度制
  Angle_Gyr=Angle_Last+(-Gyr_New[1])*dt;  //通过角速度得到当前角度值
  Angle_New=F_AccGyr*Angle_Acc+(1-F_AccGyr)*Angle_Gyr;  //加速度和角速度融合
}

//对读数进行换算，并转化成浮点型物理量
void Converse()
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
void Set_Balance_Pwm()
{
  Balance_Pwm=Balance_Kp*Angle_New+Balance_Kd*Gyr_New[1]; //计算出直立的PWM值
  // Balance_Pwm+=1.0*(Balance_Pwm-(Velocity/10));
  Balance_Vel=Balance_Pwm*10;
}

//计算速度的PWM值，40ms一次
void Set_Velocity_Pwm()
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
void Coor_Motor()
{
  L_Pwm=Balance_Pwm; //为左轮赋值
  R_Pwm=Balance_Pwm; //为右轮赋值
  // L_Pwm=Balance_Pwm+Velocity_Pwm; //为左轮赋值
  // R_Pwm=Balance_Pwm+Velocity_Pwm; //为右轮赋值
  Limit_Pwm();  //PWM限幅
}

//PWM限幅
void Limit_Pwm()
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
void Set_Direction()
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
void Kernel()
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

void setup() {
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
  Write_MPUData(0x6B,0);  //激活MPU6050
  delay(20);
  MsTimer2::set(2,Kernel);  //设置5ms的定时器中断，中断函数为Kernel
  MsTimer2::start();  //打开定时器中断
  attachInterrupt(1,READ_ENCODER_L,CHANGE); //打开外部中断，用于编码器计数，下同
  attachInterrupt(0,READ_ENCODER_R,CHANGE);
}

void loop() {
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
