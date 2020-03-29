#ifndef DEFINE_h
#define DEFINE_h

#include "Arduino.h"

struct coords{
    double x;
    double y;
    double z;
};

// スイッチやLEDのピン設定
#define PIN_DIP1 25
#define PIN_DIP2 24
#define PIN_DIP3 69
#define PIN_DIP4 70

#define PIN_SW_UP    32
#define PIN_SW_LEFT  33
#define PIN_SW_RIGHT 31
#define PIN_SW_DOWN  30

#define PIN_SW_BRACK    29
#define PIN_SW_RED      28

#define PIN_ENC_A  26
#define PIN_ENC_B  27

#define PIN_LED_1   20
#define PIN_LED_2   36
#define PIN_LED_3   37
#define PIN_LED_4   38
#define PIN_LED_ENC 40

#define SERIAL_LPC1768  Serial0
#define SERIAL_LPMSME1  Serial1
#define SERIAL_ROBOCLAW Serial4
#define SERIAL_LEONARDO Serial5

// Lernardo からのコントローラ用データのマスクデータ
#define BUTTON_X  0x0001
#define BUTTON_Y  0x0002
#define BUTTON_A  0x0004
#define BUTTON_B  0x0008

#define BUTTON_L1     0x0010
#define BUTTON_R1     0x0020
#define BUTTON_L2     0x0040
#define BUTTON_R2     0x0080

#define BUTTON_JOY_L   0x0100
#define BUTTON_JOY_R   0x0200
#define BUTTON_BACK    0x0400
#define BUTTON_START   0x0800

#define BUTTON_UP     0x1000
#define BUTTON_RIGHT  0x2000
#define BUTTON_DOWN   0x4000
#define BUTTON_LEFT   0x8000

//#define PIN_CTRL    ( A1 )
//#define PIN_XY      (  )

// 制御周期
#define INT_TIME			( 0.01 )//( 0.001 )

// フェーズ管理
//#define STATE1      ( 10 )// スタートからゲルゲ受け渡しまで(0から数えて)
#define STATE1_1    ( 7 )// ベジエTANGENTモード
#define STATE1_2    ( 8 )// ベジエCOMMANDモード
#define STATE1_3    ( 9 )// ベジエCOMMANDモード(フェーズの変更は収束判定ではなくリミットスイッチで)
#define STATE1_4    ( 10 )// 使うか分からないけど
#define STATE2      ( 14 )// ゲルゲ受け渡し後からシャガイ取得まで
#define STATE3      ( 15 )// シャガイ取得後からスローイングゾーン待機まで
#define STATE4      ( 17 )// 投擲位置まで移動
#define STATE5      ( 20 )//( 19 )// 2個目のシャガイまで
#define STATE6      ( 22 )//( 21 )// シャガイ取得後からスローイングゾーン待機まで
#define STATE7      ( 25 )//( 23 )// 3個目のシャガイまで
#define STATE8      ( 27 )//( 25 )// シャガイ取得後からスローイングゾーン待機まで

#define STATE_ALL   ( STATE1 + STATE2 + STATE3 + STATE4 )

// 上半身との通信
// #define BIT_START   ( 0b10010000 ) // 0x90:最初に送る
#define BIT_RED     ( 0b11010000 )// 赤の初期化
#define BIT_BLUE    ( 0b10010000 )// 青の初期化
#define BIT_INIT    ( 0b10001000 )// 
#define BIT_DOWN    ( 0b10001001 )// シャガイを取るモード
#define BIT_ROT     ( 0b10001101 )// ローラ回転
#define BIT_DEP     ( 0b10100000 )// ゲルゲ展開
#define BIT_STOR    ( 0b10000001 )// ゲルゲ格納
#define BIT_EXT     ( 0b10001110 )// シャガイを投げる
#define BIT_0       ( 0b00000000 )

#define MASK_SHAGAIARM  ( 0b00010000 )// シャガイハンド上下のマスク

// VL53L0X
#define SENSOR_NUM  4 // 使用するセンサーの数
#define ADDRESS_DEFALUT 0b0101001 // 0x29
#define ADDRESS_00 (ADDRESS_DEFALUT + 2)

// 自己位置推定用エンコーダ関連
#define _2PI_RES4   ( 2 * 3.141592 / 400 ) // res = 100
#define RADIUS_X    ( 0.0188 )
#define RADIUS_Y    ( 0.0188 )

#define DRIVE_MECHANUM      ( 0 )
#define DRIVE_OMNI4WHEEL    ( 1 )
#define DRIVE_OMNI3WHEEL    ( 2 )
#define DRIVE_DUALWHEEL     ( 3 )

#define DRIVE_MODE  ( DRIVE_OMNI4WHEEL )

#if DRIVE_MODE == DRIVE_DUALWHEEL
    // 双輪キャスター関連
    #define PIN_CSB     ( 10 )    // turntableのPIN(CSB)
    #define RADIUS_R    ( 0.04 )    // wheel radius
    #define RADIUS_L    ( 0.04 )    // wheel radius
    #define W           ( 0.265 )    // tread
    #define GEARRATIO   ( 5.5 )
    #define TT_RES4     ( 4096 )    // turntableの分解能
    #define _2RES_PI    ( 2 * 2048 / 3.141592 ) // 駆動輪の角速度[rad/s]からRoboClawの指令値[pulses/s]に変換するための定数  
    #define _2RES_PI_T  ( 2 * 500 / 3.141592 ) //  ターンテーブルの角速度[rad/s]からRoboClawの指令値[pulses/s]に変換するための定数
#elif DRIVE_MODE == DRIVE_MECHANUM
    // メカナム関連
    #define MECANUM_RES			( 500 )
    #define MECANUM_HANKEI		( 0.05 )
    #define MECANUM_HANKEI_D	( 0.15561 )
    #define MECANUM_HANKEI_L	( 0.26023 )
#elif DRIVE_MODE == DRIVE_OMNI3WHEEL
    #define _2RES_PI    ( 2 * 3 / 3.141592 ) // 駆動輪の角速度[rad/s]からRoboClawの指令値[pulses/s]に変換するための定数(3ppr)
    #define WHEEL_R		( 0.019 )  
    #define DIST2WHEEL  ( 0.120 )   
    #define GEARRATIO   ( 51.45)   
    #define COS_PI_6    ( 0.86602540378 )
    #define SIN_PI_6    ( 0.5 )
#elif DRIVE_MODE == DRIVE_OMNI4WHEEL
    #define _2RES_PI    ( 2 * 1000 / 3.141592 ) // 駆動輪の角速度[rad/s]からRoboClawの指令値[pulses/s]に変換するための定数 (1000ppr)
    #define WHEEL_R		( 0.05 )  
    #define DIST2WHEEL  ( 0.338 )   
    #define GEARRATIO   ( 0.6 )   
    #define SINCOS_PI_4 ( 0.70710678118 )
    #define WHEELXY_R   (0.02) // meter
    #define MAX_VEL     (1.0)
    #define MAX_OMEGA   (2.094395102393)
    #define HIGH_VELOCITY (2.0)
    #define LOW_VELOCITY  (0.3)

#endif

// RoboClaw関連
#define ADR_MD1             ( 129 )
#define ADR_MD2             ( 130 )

//const double _2PI_MEASRMX = 2.0 * PI / MEASURE_RES_MUL_X;
//const double _2PI_MEASRMY = 2.0 * PI / MEASURE_RES_MUL_Y;
//const double _0P5_MEASHD = 0.5 / MEASURE_HANKEI_D;
//const double _MECAHD_ADD_MECAHL = MECANUM_HANKEI_D + MECANUM_HANKEI_L;
//const double _2MECAR_PI = 2.0 * MECANUM_RES / PI;

#endif
