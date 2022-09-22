#include <Arduino.h>
#include <Wire.h>
// #include <string>

// #define HIGH 1 
// #define LOW 0 

using namespace std;

  #define MPU 0x68  //MPUI2C总线地址

  //MPU6050读取及数据处理
  const int Data_Num=7; //数据数量
  const int Acc_Num=3;  //加速度相关变量数量
  const int Gyr_Num=3;  //角速度相关变量数量
  const float dt=0.005; //中断时间间隔5ms
  volatile int Measure_Data[Data_Num]; //测量出的数据
  float Acc_New[Acc_Num];  //换算后的加速度值
  float Acc_Last[Acc_Num]; //上一次得到的加速度值
  float Gyr_New[Gyr_Num];  //换算后的角速度值
  float Gyr_Last[Gyr_Num]; //上一次得到的角速度值

  //电机（IN1 LOW,IN2 HIGH 为前，IN3 LOW,IN4 HIGH 为后）
  #define IN1 7 //右轮
  #define IN2 8 //右轮
  #define IN3 10 //左轮
  #define IN4 11  //左轮
  #define PWM_L 9 //左轮PWM引脚
  #define PWM_R 6 //右轮PWM引脚
  int L_Pwm,R_Pwm; //左右电机PWM值

  //编码器
  #define ENCODER_L 3
  #define ENCODER_R 2
  #define DIRECTION_L 5
  #define DIRECTION_R 4
  int Velocity_L=0,Velocity_R=0; //左右编码区数据

class BalanceCtr {
  public:
  //小车运动控制信息
  int speed=0; //前进速度
  int turn=0; //转弯角度

  //直立PD闭环
  float Balance_Pwm=0; //直立PD得出的PWM值
  float Balance_Kp=15,Balance_Kd=-2.1;//10 -2.1 直立PD的PD系数
  float Balance_Vel=0; //直立PD得出的期望速度值

  //速度PI闭环
  float Velocity_Pwm=0;  //速度PI得出的PWM值
  float Velocity_Bias=0; //速度的偏差值
  float Velocity=0;  //当前速度值
  float Velocity_Kp=0.4,Velocity_Ki=Velocity_Kp/200; //速度PI的PI系数
  float Velocity_Integral=0; //速度偏差的积分
  int Velocity_Count=0;  //速度计数位，使速度PI每40ms进行调控

  //虚拟示波器
  unsigned char DataScope_OutPut_Buffer[42] = {0};     //串口发送缓冲区
  unsigned char Send_Count; //发送数据的个数

  //加速度及角速度一阶滤波
  const float L_Acc=0.3;  //速度一阶滤波系数
  const float L_Gyr=0.3;  //角速度一阶滤波系数

  //加速度与角速度互补融合
  const float F_AccGyr=0.9; //加速度角速度融合置信度

  //计算倾角
  float Angle_New=0; //最新得出的角度值
  float Angle_Last=0;  //上一次的角度值
  float Angle_Acc=0; //通过加速度得到的角度值
  float Angle_Gyr=0; //通过角速度得到的角度值

  //函数说明：将单精度浮点数据转成4字节数据并存入指定地址
  //附加说明：用户无需直接操作此函数
  //target:目标单精度数据
  //buf:待写入数组
  //beg:指定从数组第几个元素开始写入
  //函数无返回
  void Float2Byte(float *target,unsigned char *buf,unsigned char beg);

  //函数说明：将待发送通道的单精度浮点数据写入发送缓冲区
  //Data：通道数据
  //Channel：选择通道（1-10）
  //函数无返回
  void DataScope_Get_Channel_Data(float Data,unsigned char Channel);

  //函数说明：生成 DataScopeV1.0 能正确识别的帧格式
  //Channel_Number，需要发送的通道个数
  //返回发送缓冲区数据个数
  //返回0表示帧格式生成失败
  unsigned char DataScope_Data_Generate(unsigned char Channel_Number);

  //读取左轮编码器数据
  //正转+1，反转-1
  void READ_ENCODER_L();

  //读取左轮编码器数据
  //正转+1，反转-1
  void READ_ENCODER_R(); //读取右轮编码器数据

  //向MPU中输入一个字节的数据
  void Write_MPUData(int addre,unsigned char Data);

  //从MPU6050中读出全部数据，储存到Measure_Data[Read_Num]中
  void Read_AccGyr_Data();

  //计算出角度值
  void GetPitch();

  //对读数进行换算，并转化成浮点型物理量
  void Converse();

  //计算直立的PWM值，5ms一次
  void Set_Balance_Pwm();

  //计算速度的PWM值，40ms一次
  void Set_Velocity_Pwm();

  //为左右轮赋PWM值
  void Coor_Motor();

  //PWM限幅
  void Limit_Pwm();

  //通过计算出来的PWM设置正反转，视接线方法而定
  void Set_Direction();

  //定时器中断函数，5ms一次
  void Kernel();


  unsigned char readSerial();

  std::string readSerialStr();
};