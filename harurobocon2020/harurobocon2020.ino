//春ロボコン足回り
//最終更新　2020/03/16

#include <Arduino.h>
#include <MsTimer2.h>

#include "RoboClaw.h"
#include "phaseCounterPeach.h"
#include "ManualControl.h"
#include "lpms_me1Peach.h"
#include "define.h"

phaseCounter enc1(1);
phaseCounter enc2(2);
RoboClaw roboclaw(&SERIAL_ROBOCLAW,1);
ManualControl Controller;
lpms_me1 lpms(&SERIAL_LPMSME1);

// コントローラデータ格納用
unsigned int ButtonState = 0, LJoyX = 127, LJoyY = 127, RJoyX = 127, RJoyY = 127;

/****自己位置推定用の変数****/
double encX_rad , encX  ,pre_encX = 0.0;
double encY_rad , encY , pre_encY = 0.0;
double x_axis = 0.0 , x_axis_prime , y_axis = 0.0 , y_axis_prime;

/****角度PID制御用の変数****/
double angle_rad;
double rad_error , pre_rad_error = 0.0;
double Kp_rad = 0.045 , Kd_rad = 0.005;
double refVz;

/****位置PID制御用の変数****/
double x_error , pre_x_error = 0.0;
double y_error , pre_y_error = 0.0;
double x_target = 2.0 , y_target = 1.0;
double Kp_posi = 0.0 , Kd_posi = 0.0;

/****フラグの変数****/
bool flag_10ms  = false;
bool flag_100ms = false;
bool flag_400ms = false;
bool flag_800ms = false;

// LEDをチカチカさせるための関数
void LEDblink(byte pin, int times, int interval){
  analogWrite(pin, 0);
  for(int i = 0; i < times; i++){
    delay(interval);
    analogWrite(pin, 255);
    delay(interval);
    analogWrite(pin, 0);
  }
}

void LED_iv(int high_or_low){
  digitalWrite(PIN_LED_1,high_or_low);
  digitalWrite(PIN_LED_2,high_or_low);
  digitalWrite(PIN_LED_3,high_or_low);
  digitalWrite(PIN_LED_4,high_or_low);
}

// 最大最小範囲に収まるようにする関数
double min_max(double value, double minmax){
  if(value > minmax) value = minmax;
  else if(value < -minmax) value = -minmax;
  return value;
}

// コントローラデータを取得する部分
void controller_receive(){
  static int recv_num = 0;
  //static int checksum = 0;
  static char recv_msgs[9];
  char c;
  while(SERIAL_LEONARDO.available()){
    c = SERIAL_LEONARDO.read();
    if(c == '\n'){
      if(recv_num == 9){// && (checksum & 0x3F == recv_msgs[recv_num-1] - 0x20)){
        ButtonState = 0, LJoyX = 0, LJoyY = 0, RJoyX = 0, RJoyY = 0;
        ButtonState |= recv_msgs[0] - 0x20;
        ButtonState |= (recv_msgs[1] - 0x20) << 6;
        ButtonState |= (recv_msgs[2] - 0x20) << 12;

        LJoyX |= (recv_msgs[3] - 0x20);
        LJoyX |= ((recv_msgs[4] - 0x20) & 0x03) << 6;

        LJoyY |= ((recv_msgs[4] - 0x20) & 0x3C) >> 2;
        LJoyY |= ((recv_msgs[5] - 0x20) & 0x0F) << 4;

        RJoyX |= ((recv_msgs[5] - 0x20) & 0x30) >> 4;
        RJoyX |= ((recv_msgs[6] - 0x20) & 0x3F) << 2;

        RJoyY |= (recv_msgs[7] - 0x20);
        RJoyY |= ((recv_msgs[8] - 0x20) & 0x03) << 6;
      }
      recv_num = 0;
      //checksum = 0;
    }
    else{
      recv_msgs[recv_num] = c; 
      //checksum += recv_msgs[recv_num];
      recv_num++;
    }
  }
}

/****位置PID制御の関数****/
double rad_pid(double rad_target, double rad){
    rad_error = rad_target - rad; //誤差を計算
    double rad_next = rad; //目標位置を初期化
    rad_next += Kp_rad*rad_error + Kd_rad*(rad_error - pre_rad_error); //目標位置を決定
    pre_rad_error = rad_error; //誤差を更新
    return (rad_next - rad) / 0.01; //速度指令値を返す
}

double position_x_pid(double x){
  x_error = x_target - x; //誤差を計算
  double x_next = x; //目標位置を初期化
  x_next += Kp_posi*x_error + Kd_posi*(x_error - pre_x_error); //目標位置を決定
  pre_x_error = x_error; //誤差を更新
  return (x_next - x) / 0.01; //速度指令値を返す
}

double position_y_pid(double y){
  y_error = y_target - y; //誤差を計算
  double y_next = y; //目標位置を初期化 
  y_next += Kp_posi*y_error + Kd_posi*(y_error - pre_y_error); //目標位置を決定
  pre_y_error = y_error; //誤差を更新
  return (y_next - y) / 0.01; //速度指令を返す
}

/****自己位置推定の関数****/
void axis_func(){
  
  encX_rad = (double)enc2.getCount() * _2PI_RES4;
  encY_rad = (double)enc1.getCount() * _2PI_RES4;
  angle_rad = (double)lpms.get_z_angle();

  encX = RADIUS_X * encX_rad;
  encY = RADIUS_Y * encY_rad;
  x_axis_prime = encX - pre_encX;
  y_axis_prime = encY - pre_encY;

  x_axis += x_axis_prime*cos(angle_rad) - y_axis_prime*sin(angle_rad);
  y_axis += x_axis_prime*sin(angle_rad) + y_axis_prime*cos(angle_rad);

  pre_encX = encX;
  pre_encY = encY;
}

/****割込みの関数****/
void timer_warikomi(){
  // RGB LED を良い感じに光らせるための処理
  static int count = 0;
  static int flag_100ms_count = 0;
  static int flag_400ms_count = 0;
  static int flag_800ms_count = 0;
  count += 2; // ここで光る周期を変えられる(はず)

  if(count < 255){
    analogWrite(PIN_LED_RED, count);
    analogWrite(PIN_LED_BLUE, 255 - count);
  }
  else if(count < 255 * 2){
    analogWrite(PIN_LED_GREEN, count - 255);
    analogWrite(PIN_LED_RED, 255 * 2 - count);
  }
  else if(count < 255 * 3){
    analogWrite(PIN_LED_BLUE, count - 255 * 2);
    analogWrite(PIN_LED_GREEN, 255 * 3 - count);
  }
  else{
    count = 0;
  }

  axis_func(); //自己位置
  
  flag_100ms_count++;
  flag_400ms_count++;
  flag_800ms_count++;

  if(flag_100ms_count >= 10){
    flag_100ms = !flag_100ms;
    flag_100ms_count = 0;
  }
  if(flag_400ms_count >= 40){
    flag_400ms = !flag_400ms;
    flag_400ms_count = 0;
  }
  if(flag_800ms_count >= 80){
    flag_800ms = !flag_800ms;
    flag_800ms_count = 0;
  }

  flag_10ms = true;
}

void setup(){
  bool ready_to_start = false;

  Serial.begin(115200);
  SERIAL_LPC1768.begin(115200);
  SERIAL_LEONARDO.begin(115200);
  roboclaw.begin(115200);
 // SERIAL_LCD.begin(115200);
  //SERIAL_XBEE.begin(115200);
  
  pinMode(PIN_SW, INPUT); // オンボードのスイッチ

  pinMode(PIN_LED_1, OUTPUT);
  pinMode(PIN_LED_2, OUTPUT);
  pinMode(PIN_LED_3, OUTPUT);
  pinMode(PIN_LED_4, OUTPUT);
  pinMode(PIN_LED_ENC, OUTPUT);

  pinMode(PIN_DIP1,INPUT);
  pinMode(PIN_DIP2,INPUT);
  pinMode(PIN_DIP3,INPUT);
  pinMode(PIN_DIP4,INPUT);
  
  pinMode(PIN_ENC_A, INPUT);
  pinMode(PIN_ENC_B, INPUT);

  LEDblink(PIN_LED_GREEN, 2, 100);
  
  // LPMS-ME1の初期化
  if(lpms.init() != 1)
  LEDblink(PIN_LED_BLUE, 2, 100);  // 初期が終わった証拠にブリンク
  Serial.println("LPMS-ME1 init done!");
  Serial.flush();
  
  // コントローラの"START"ボタンが押されるまで待機
  while(!ready_to_start){
    controller_receive();
    if(ButtonState & BUTTON_START){
      digitalWrite(PIN_LED_ENC,HIGH);
      ready_to_start = true;
      Serial.println("BUTTON_START");
    }
  }

  enc1.init();
  enc2.init();
  //Serial0.println(ButtonState,HEX);

  MsTimer2::set(10, timer_warikomi); // 10ms period
  MsTimer2::start();
}

void loop(){  
  controller_receive(); // コントローラ(Leonardo)からの受信

  if( flag_10ms ){

    coords rawV = Controller.getVel(LJoyX, LJoyY, RJoyY);

    coords rawV2;
    static double refAngle = 0.0;
    refAngle += rawV.z*0.01;
    rawV2.x = +rawV.x*cos(angle_rad) + rawV.y*sin(angle_rad);
    rawV2.y = -rawV.x*sin(angle_rad) + rawV.y*cos(angle_rad);
    rawV2.z = min_max(rad_pid(refAngle,angle_rad),MAX_OMEGA);

    coords refV;
    double vel;
    vel = sqrt(rawV2.x*rawV2.x + rawV2.y*rawV2.y);
    if(vel <= 0.0){
      refV.x = rawV2.x;
      refV.y = rawV2.y;
    }else if(fabs(rawV2.x) > fabs(rawV2.y)){
      refV.x = rawV2.x*fabs(rawV2.x) / vel;
      refV.y = rawV2.y*fabs(rawV2.x) / vel;
    }else if(fabs(rawV2.x) <= rawV2.y){
      refV.x = rawV2.x*fabs(rawV2.y) / vel;
      refV.y = rawV2.y*fabs(rawV2.y) / vel;
    }
    
    if(fabs(refV.x) <= 0.01) refV.x = 0.0;
    if(fabs(refV.y) <= 0.01) refV.y = 0.0;
    if(fabs(refV.z) <= 0.15) refV.z = 0.0;

    static double a = MAX_VEL;
    if(ButtonState & BUTTON_X){
      if(a < HIGH_VELOCITY) a += 0.05;
      if(flag_100ms) LED_iv(HIGH);
      else LED_iv(LOW);
    }
    else if(MAX_VEL < a){
      a -= 0.05;
      if(flag_400ms) LED_iv(HIGH);
      else LED_iv(LOW);
    }
    else if (ButtonState & BUTTON_B){
      if( LOW_VELOCITY < a) a -= 0.05;
      if(flag_800ms) LED_iv(HIGH);
      else LED_iv(LOW);
    }else if(a < MAX_VEL) {
      a += 0.05;
      if(flag_400ms) LED_iv(HIGH);
      else LED_iv(LOW);
    }
    
    refV.x *= a;
    refV.y *= a;
    refV.z = rawV2.z;

    double refOmega1, refOmega2, refOmega3, refOmega4;
    refOmega1 = (+refV.x*SINCOS_PI_4 + refV.y*SINCOS_PI_4 + refV.z * DIST2WHEEL) / WHEEL_R * GEARRATIO;
    refOmega2 = (-refV.x*SINCOS_PI_4 + refV.y*SINCOS_PI_4 + refV.z * DIST2WHEEL) / WHEEL_R * GEARRATIO;
    refOmega3 = (-refV.x*SINCOS_PI_4 - refV.y*SINCOS_PI_4 + refV.z * DIST2WHEEL) / WHEEL_R * GEARRATIO;
    refOmega4 = (+refV.x*SINCOS_PI_4 - refV.y*SINCOS_PI_4 + refV.z * DIST2WHEEL) / WHEEL_R * GEARRATIO;

    /**PID制御**/
    // double pid_x, pid_y, pid_z;
    // pid_x = min_max(position_x_pid(x_axis),2.0);
    // pid_y = min_max(position_y_pid(y_axis),2.0);
    // pid_z = min_max(rad_pid(angle_rad),2.5);

    // refOmega1 = (+pid_x*SINCOS_PI_4 + pid_y*SINCOS_PI_4 + pid_z * DIST2WHEEL) / WHEEL_R * GEARRATIO;
    // refOmega2 = (-pid_x*SINCOS_PI_4 + pid_y*SINCOS_PI_4 + pid_z * DIST2WHEEL) / WHEEL_R * GEARRATIO;
    // refOmega3 = (-pid_x*SINCOS_PI_4 - pid_y*SINCOS_PI_4 + pid_z * DIST2WHEEL) / WHEEL_R * GEARRATIO;
    // refOmega4 = (+pid_x*SINCOS_PI_4 - pid_y*SINCOS_PI_4 + pid_z * DIST2WHEEL) / WHEEL_R * GEARRATIO;

    double mdCmd1, mdCmd2, mdCmd3, mdCmd4;
    mdCmd1 = refOmega1 * _2RES_PI;
    mdCmd2 = refOmega2 * _2RES_PI;
    mdCmd3 = refOmega3 * _2RES_PI;
    mdCmd4 = refOmega4 * _2RES_PI;

    roboclaw.SpeedM1(ADR_MD1,(int)mdCmd1); //wheel1
    roboclaw.SpeedM2(ADR_MD1,(int)mdCmd2); //wheel2
    roboclaw.SpeedM1(ADR_MD2,(int)mdCmd3); //wheel3
    roboclaw.SpeedM2(ADR_MD2,(int)mdCmd4); //wheel4
    Serial0.println(ButtonState,HEX);
    // Serial.print(x_axis);
    // Serial.print("\t");
    // Serial.println(y_axis);

    flag_10ms = false;
  }
}



/* adress : https://www.instagram.com/kikuchi8810/ */