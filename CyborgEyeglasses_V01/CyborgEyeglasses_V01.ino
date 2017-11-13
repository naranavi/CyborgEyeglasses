#include <Servo.h>
#include <EEPROM.h>
extern "C" {
#include "user_interface.h"
}
#include "common.h"
#include "MovingAverage.h"
#include "UdpComm.h"
#include <Adafruit_NeoPixel.h>
/*
 * サイボーグメガネ　2017.11.9
 * ESP-2866 NODE MCU
 * GPduinoで光り方を制御
 * 上下色合い　左右バランス
 * 上下
 * 上下急に離すとその色を保持
 * 　MAXはサイクルモード
 */

/*
 * 
シルク、ArduinoNo
D0:
D1:16:SCL
D2: 5:SDA
D3: 0:
D4: 2:SrialLED
3V3
GND
D5:14:
D6:12:S_RX
D7:13:S_TX
D8:15:
RX: 3:
TX: 1:
A0:ADC0
RSV
RSV
SD3:10:
SD2: 9:
SD1
CMD
SDO
CLK
GND
3V3
EN
RST
GND
Vin

 */

// UDP通信クラス
UdpComm udpComm;
// UDP受信コールバック
void udpComm_callback(char* buff);

// ピン番号 (デジタル)
#define MOTOR_R_PWM    14   //D5 HSCLK
#define MOTOR_R_IN2    12   //-D6 HMISO
#define MOTOR_R_IN1    13   //D7 RXD2 HMOSI
#define MOTOR_L_PWM    15   //D8 TXD2 HCS
#define MOTOR_L_IN2    2    //-D4  TXD1
#define MOTOR_L_IN1    0    //-D3  FLASH
#define SERVO0_PWM     16   //D0  WAKE
#define SERVO1_PWM     5    //-D1
#define SERVO2_PWM     4    //-D2

//nodeMCU
#define MOTOR_R_pwm    4
#define MOTOR_R_dir    2
#define MOTOR_L_pwm    5
#define MOTOR_L_dir    0
#define MOTOR_FAN     15

#define TS 20   //20ms

//表示ファンクション
enum dispF {Nomal,Rainbow,Flash};

enum dispF func;

// サーボ
Servo servo[3];

// バッテリー電圧チェック
MovingAverage Vbat_MovingAve;

// モード
static int drive_mode;
#define MODE_TANK   1   // 戦車モード
#define MODE_CAR    0   // 自動車モード

// 戦車モードのプロポ状態保持用
static int g_fb;  // 前後方向
static int g_lr;  // 左右方向

// 暴走チェックカウンタ
static int cnt_runaway;

// FAN動作時間
static int cnt_fanRun;

// 3.5V未満でローバッテリーとする
// (3.5V / 11) / 1.0V * 1024 =  325
#define LOW_BATTERY    325

// 四輪操舵のモード
static int g_steer_pol_f;
static int g_steer_pol_b;

// サーボの調整
static int g_servo_pol[3]; // 極性
static int g_servo_ofs[3]; // オフセット
static int g_servo_amp[3]; // 振幅

// 送信バッファ
static char txbuff[256];

#define PIN    2    // Digital IO pin connected to the Neostrip.
#define NUMPIXELS      2

Adafruit_NeoPixel strip = Adafruit_NeoPixel(NUMPIXELS, PIN, NEO_GRB + NEO_KHZ800);

/*
 * 通常表示
 */
void nomal(void)
{
    static int fb=0;
    static int lr=0;
    float d1,d2;

    Serial.println("func nomal");

      
   while(func == Nomal){
    // UDP通信
    delayUDP(TS);
Serial.println("func nomal in");
    if (g_fb!=0 ){
      fb = g_fb;
    }

    if(g_fb != 0 || g_lr != lr){
      //表示更新
      uint8_t d = fb+127;   //0-255)
      float k = (float)(127-g_lr)/255.0;
      strip.setPixelColor(0,WheelK(d,k)); 
      
      k = (float)(127+g_lr)/255.0;
      strip.setPixelColor(1,WheelK(d,k)); 
      
      strip.show();
    }
    

  }
}

  //2）レインボー１
void rainbow(void)
{
    Serial.println("func rainbow");
   while(func == Rainbow){
      // UDP通信
      //udpComm.loop();
    Serial.println("func rainbow in");
    rainbowFade2White(30,5,2);
    }
}

  //3）赤目ちらつき
void flash(void)
{  
    Serial.println("func flash");
  while(func == Flash){
      colorWipe(strip.Color(128, 0, 0), 0); // Red
      delayUDP(30);
      colorWipe(strip.Color(60, 0, 0), 0); // Red
      delayUDP(15);
  }
}

  //3）赤目ちらつき
void flash2(void)
{  
    Serial.println("func flash");
  while(func == Flash){
    // UDP通信
    //udpComm.loop();
    colorWipe(strip.Color(random(20,150) , 0, 0), random(20,150)); // Red
    delayUDP(TS);
  }
}

void udpLoop(void)
{
  static unsigned long t;

  if(millis()-t>=20){
    t = millis();
    udpComm.loop();
  }
}

void delayUDP(unsigned long t)
{

  for(long i=0;i<t;i++){
    udpLoop();
    if (i % 20 ==0){
      battery_check();// バッテリー電圧チェック
    }
    delay(1);
  }
}

void rainbowFade2White(uint8_t wait, int rainbowLoops, int whiteLoops) {
  float fadeMax = 100.0;
  int fadeVal = 0;
  uint32_t wheelVal;
  int redVal, greenVal, blueVal;

  for(int k = 0 ; k < rainbowLoops ; k ++){
    
    for(int j=0; j<256; j++) { // 5 cycles of all colors on wheel

      for(int i=0; i< strip.numPixels(); i++) {

        wheelVal = Wheel(((i * 256 / strip.numPixels()) + j) & 255);

        redVal = red(wheelVal) * float(fadeVal/fadeMax);
        greenVal = green(wheelVal) * float(fadeVal/fadeMax);
        blueVal = blue(wheelVal) * float(fadeVal/fadeMax);

        strip.setPixelColor( i, strip.Color( redVal, greenVal, blueVal ) );

      }

      //First loop, fade in!
      if(k == 0 && fadeVal < fadeMax-1) {
          fadeVal++;
       }

      //Last loop, fade out!
      else if(k == rainbowLoops - 1 && j > 255 - fadeMax ){
          fadeVal--;
      }

        strip.show();
        delayUDP(wait);
        udpComm.loop(); // UDP通信
        if (func != Rainbow)return ;
    }
  
  }

  for(int k = 0 ; k < whiteLoops ; k ++){

    for(int j = 0; j < 256 ; j++){

            strip.setPixelColor(0, strip.Color(j,j,j) );
            strip.setPixelColor(1, strip.Color(j,j,j) );
         strip.show();
          delayUDP(wait*2);
          udpComm.loop(); // UDP通信
          if (func != Rainbow)return ;
        }

        delayUDP(1000);
    for(int j = 255; j >= 0 ; j--){

             strip.setPixelColor(0, strip.Color(j,j,j) );
            strip.setPixelColor(1, strip.Color(j,j,j) );
         strip.show();
          delayUDP(wait*2);
          udpComm.loop(); // UDP通信
          if (func != Rainbow)return ;
        }
  }

  delayUDP(500);


}



// Input a value 0 to 255 to get a color value.
// The colours are a transition r - g - b - back to r.
// k:0-1.0
uint32_t WheelK(byte WheelPos,float k) {
  WheelPos = 255 - WheelPos;
  if(WheelPos < 85) {
    return strip.Color((uint8_t)((float)(255 - WheelPos * 3)*k), 0, (uint8_t)((float)(WheelPos * 3)*k));
  }
  if(WheelPos < 170) {
    WheelPos -= 85;
    return strip.Color(0, (uint8_t)((float)(WheelPos * 3)*k), (uint8_t)((float)(255 - WheelPos * 3)*k));
  }
  WheelPos -= 170;
  return strip.Color((uint8_t)((float)(WheelPos * 3)*k), (uint8_t)((float)(255 - WheelPos * 3)*k), 0);
}

// Input a value 0 to 255 to get a color value.
// The colours are a transition r - g - b - back to r.
uint32_t Wheel(byte WheelPos) {
  WheelPos = 255 - WheelPos;
  if(WheelPos < 85) {
    return strip.Color(255 - WheelPos * 3, 0, WheelPos * 3,0);
  }
  if(WheelPos < 170) {
    WheelPos -= 85;
    return strip.Color(0, WheelPos * 3, 255 - WheelPos * 3,0);
  }
  WheelPos -= 170;
  return strip.Color(WheelPos * 3, 255 - WheelPos * 3, 0,0);
}

uint8_t red(uint32_t c) {
  return (c >> 8);
}
uint8_t green(uint32_t c) {
  return (c >> 16);
}
uint8_t blue(uint32_t c) {
  return (c);
}

// Fill the dots one after the other with a color
void colorWipe(uint32_t c, uint8_t wait) {
  for(uint16_t i=0; i<strip.numPixels(); i++) {
    strip.setPixelColor(i, c);
    strip.show();
    delayUDP(wait);
  }
}





/*
 * FAN 運転開始
 */
 void fan_run(void)
 {
    cnt_fanRun = 2000/TS;
     digitalWrite(MOTOR_FAN,HIGH);
 }

 void fan_stop(void)
 {
     digitalWrite(MOTOR_FAN,LOW);
 }
 
/**
 * バッテリー電圧チェック
 */
void battery_check()
{
    if(!udpComm.isReady()) return;
    
    static int cnt1 = 0;
    static int cnt2 = 0;

    //Fan停止チェック
    if(cnt_fanRun>0){
      if(--cnt_fanRun ==0){
        //STOP
        fan_stop();
      }
       
    }
    // 100msecごとに電圧値測定
    cnt1++;
    if(cnt1 < 100/TS) return;
    cnt1 = 0;

    unsigned short Vbat = system_adc_read();
    unsigned short Vbat_ave = Vbat_MovingAve.pop(Vbat);
    //Serial.print("Vbat_ave");Serial.println(Vbat_ave);

    // 1秒ごとに電圧値送信
    cnt2++;
    if(cnt2 >= 10)
    {
        cnt2=0;
        
        txbuff[0]='#';
        txbuff[1]='B';
        Uint16ToHex(&txbuff[2], Vbat_ave, 3);
        txbuff[5]='$';
        txbuff[6]='\0';
        udpComm.send(txbuff);
    }
    
    // 電圧が低下したら停止
        return;  //@@
    if(Vbat_ave < LOW_BATTERY){
        // モータ停止
       // analogWrite(MOTOR_L_pwm, 0);
      //  analogWrite(MOTOR_R_pwm, 0);
        fan_stop();
         // 復帰しない
        Serial.println("BATT LOW!!");
    //    while(true){
    //        ;
    //    }
    }
}



/*
 * サーボの初期化, モードの設定
 */
void servo_init()
{
    int i;
    if(EEPROM.read(0) == 0xA5){
        for(i=0;i<3;i++){
            g_servo_pol[i] = (int)((signed char)EEPROM.read(i*3 + 1));
            g_servo_ofs[i] = (int)((signed char)EEPROM.read(i*3 + 2));
            g_servo_amp[i] =                    EEPROM.read(i*3 + 3);
        }
        drive_mode = EEPROM.read(10);
        if(drive_mode == MODE_TANK){
            Serial.println("TANK MODE");
        }else{
            Serial.println("CAR MODE");
        }
        
    }else{
        EEPROM.write(0, 0xA5);
        for(i=0;i<3;i++){
            g_servo_pol[i] = 1;
            g_servo_ofs[i] = 0;
            g_servo_amp[i] = 90;
            EEPROM.write(i*3 + 1, g_servo_pol[i]);
            EEPROM.write(i*3 + 2, g_servo_ofs[i]);
            EEPROM.write(i*3 + 3, g_servo_amp[i]);
        }
        drive_mode = MODE_CAR;
        EEPROM.write(10, drive_mode);
        EEPROM.commit();
    }
    //servo[0].attach(SERVO0_PWM);
    //servo[1].attach(SERVO1_PWM);
    //servo[2].attach(SERVO2_PWM);
    //servo_ctrl(0,0);
    //servo_ctrl(1,0);
    //servo_ctrl(2,0);
}

/*
 * サーボ制御
 */
void servo_ctrl(int ch, int val)
{
  /*
    int deg = 90 + ((val + g_servo_ofs[ch]) *  g_servo_amp[ch]) / 127 * g_servo_pol[ch];
    
    // sprintf(txbuff, "%4d %4d / %4d %4d %4d", val, deg, g_servo_ofs[ch], g_servo_amp[ch], g_servo_pol[ch]);
    // Serial.println(txbuff);
    
    servo[ch].write(deg);
    */
}

// 初期設定
void setup() {
    pinMode(MOTOR_FAN, OUTPUT);
    digitalWrite(MOTOR_FAN,LOW);
    Serial.begin(115200);
    delay(100);
    strip.begin(); // This initializes the NeoPixel library.
    strip.show();

  Serial.println("LED begin");
  colorWipe(strip.Color(0, 0, 128), 5);

  if(wifi_set_sleep_type(LIGHT_SLEEP_T)){
    Serial.println("sleep true");
  }else{
    Serial.println("sleep false");
  }
    
    // EEPROMの初期化
    EEPROM.begin(128); // 128バイト確保
    
    // UDP通信の設定
    udpComm.beginAP(NULL, "12345678");
    //udpComm.beginSTA("SSID", "password", "gpduino");
    udpComm.onReceive = udpComm_callback;

    digitalWrite(MOTOR_FAN,LOW);
    
    // モード判定
    //int mode_check = analogRead( MODE_CHECK );
    //drive_mode = (mode_check > 512) ? MODE_CAR : MODE_TANK;
    
    //if(drive_mode == MODE_CAR){
        // サーボの初期化, モード設定
        //servo_init();
    //}
    
    // 変数初期化
    drive_mode = MODE_TANK;
    g_fb = 0;
    g_lr = 0;
    cnt_runaway = 0;
    Vbat_MovingAve.init();
    g_steer_pol_f = 1;
    g_steer_pol_b = 0;
    func=Nomal;
}

// メインループ
void loop() {
     
      udpComm.loop(); // UDP通信
  if(func==Nomal){
    nomal();
  }else if(func==Rainbow){
    rainbow();
  }else if(func==Flash){
      flash();
  }
   
    battery_check();// バッテリー電圧チェック
    // 暴走チェック
  //  runaway_check();
      delay(TS);
}






/**
 * 戦車のモータ制御
    g_fb:上下
    g_lr:左右
 */
void ctrl_tank()
{
  
  if (g_fb == 0){
  }else if(g_fb == 127){
    func = Rainbow;
  }else if(g_fb == -127){
    func = Flash;
  }else{
    func = Nomal; 
  }
}

/**
 * 受信したコマンドの実行
 *
 * @param buff 受信したコマンドへのポインタ
 */
void udpComm_callback(char* buff)
{
    unsigned short val;
    int sval, sval2;
    int deg;
    int ch;
    int i;
    
    cnt_runaway = 0; // 暴走チェックカウンタのクリア
    
    Serial.print("udpComm_callback:");Serial.println(buff);
    
    if(buff[0] != '#') return;
    buff++;
    
    switch(buff[0])
    {
    /* Dコマンド(前進/後退)
       書式: #Dxx$
       xx:7f--81(-127...+127)
       xx: 0のとき停止、正のとき前進、負のとき後退。
     */
    case 'D':
        // 値の解釈
        if( HexToUint16(&buff[1], &val, 2) != 0 ) break;
        sval = (int)((signed char)val);
        

       // 自動車モードの場合
        if(drive_mode == MODE_CAR)
        {
            
            
        }else{
          // 戦車モードの場合
            g_fb = sval;
            ctrl_tank();
        }
        break;
        
    /* Tコマンド(旋回)
       書式: #Txxn$
       n: 4WSモード。0のとき後輪固定、1のとき同相、2のとき逆相
       xx: 0のとき中立、正のとき右旋回、負のとき左旋回
     */
    case 'T':
        // 4WSモード
        switch(buff[3]){
        case '0':// FRONT
            g_steer_pol_f = 1;
            g_steer_pol_b = 0;
            break;
        case '1':// COMMON
            g_steer_pol_f = 1;
            g_steer_pol_b = 1;
            break;
        case '2':// REVERSE
            g_steer_pol_f = 1;
            g_steer_pol_b = -1;
            break;
        case '3':// REAR
            g_steer_pol_f = 0;
            g_steer_pol_b = 1;
            break;
        default:
            g_steer_pol_f = 1;
            g_steer_pol_b = 0;
            break;
        }
        // 値の解釈
        if( HexToUint16(&buff[1], &val, 2) != 0 ) break;
        sval = (int)((signed char)val);
        
        // 自動車モードの場合
        if(drive_mode == MODE_CAR)
        {
          //  servo_ctrl(0, sval);
          //  servo_ctrl(1, g_steer_pol_f * sval);
          //  servo_ctrl(2, g_steer_pol_b * sval);
        }
        // 戦車モードの場合
        else
        {
            g_lr = sval;
            ctrl_tank();
        }
        break;
        
    /* Mコマンド(モータ制御)
       書式1: #Mnxx$
       n: '1'はモータ1(左)、'2'はモータ2(右)
       xx: 0のとき停止、正のとき正転、負のとき反転
       
       書式2: #MAxxyy$
       xx: モータ1(左)  0のとき停止、正のとき正転、負のとき反転
       yy: モータ2(右)  0のとき中立、正のとき正転、負のとき反転
     */
    case 'M':
        switch(buff[1]){
        case '1':
            // 値の解釈
            if( HexToUint16(&buff[2], &val, 2) != 0 ) break;
            sval = (int)((signed char)val);
            
            break;
        case '2':
            // 値の解釈
            if( HexToUint16(&buff[2], &val, 2) != 0 ) break;
            sval = (int)((signed char)val);
            
            break;
        case 'A':
            // 値の解釈
            if( HexToUint16(&buff[2], &val, 2) != 0 ) break;
            sval = (int)((signed char)val);
            if( HexToUint16(&buff[4], &val, 2) != 0 ) break;
            sval2 = (int)((signed char)val);
            
            
            break;
        }
        break;
        
    /* Sコマンド(サーボ制御)
       書式: #Sxxn$
       xx: 0のとき中立、正のとき正転、負のとき反転
       n: サーボチャンネル 0～2
     */
    case 'S':
        // 値の解釈
        if( HexToUint16(&buff[1], &val, 2) != 0 ) break;
        sval = (int)((signed char)val);
        // チャンネル
        ch = (buff[3] == '1') ? 1 : ((buff[3] == '2') ? 2 : 0);
        // サーボの制御
        servo_ctrl(ch, sval);
        break;
        
    /* Aコマンド(サーボの調整)
        (1)サーボ極性設定
           書式: #APxn$
           n: サーボチャンネル 0～2
           xは'+'または'-'。'+'は正転、'-'は反転。初期値は正転。
        (2)サーボオフセット調整
           書式: #AOxxn$
           n: サーボチャンネル 0～2
           xx:中央位置のオフセット
        (3)サーボ振幅調整
           書式: #AAnxx$
           n: サーボチャンネル 0～2
           xx:振れ幅
        (4)サーボ設定値保存
           書式: #AS$
           サーボの設定値をEEPROMに保存する
        (5)サーボ設定値読み出し
            書式: #AL$
           サーボの設定値をEEPROMから読み出し、送信する
     */
    case 'A':
        switch(buff[1]){
        case 'P':
            ch = (buff[3] == '1') ? 1 : ((buff[3] == '2') ? 2 : 0);
            if(buff[2] == '+'){
                g_servo_pol[ch] = 1;
            }else if(buff[2] == '-'){
                g_servo_pol[ch] = -1;
            }
            break;
        case 'O':
            ch = (buff[4] == '1') ? 1 : ((buff[4] == '2') ? 2 : 0);
            // 値の解釈
            if( HexToUint16(&buff[2], &val, 2) != 0 ) break;
            g_servo_ofs[ch] = (int)((signed char)val);
            break;
        case 'A':
            ch = (buff[4] == '1') ? 1 : ((buff[4] == '2') ? 2 : 0);
            // 値の解釈
            if( HexToUint16(&buff[2], &val, 2) != 0 ) break;
            g_servo_amp[ch] = (int)((unsigned char)val);
            break;
        case 'S':
            for(i=0;i<3;i++){
                EEPROM.write(i*3 + 1, g_servo_pol[i]);
                EEPROM.write(i*3 + 2, g_servo_ofs[i]);
                EEPROM.write(i*3 + 3, g_servo_amp[i]);
            }
            EEPROM.write(10, drive_mode);
            EEPROM.commit();
            break;
        case 'L':
            txbuff[0]='#';
            txbuff[1]='A';
            txbuff[2]='L';
            for(i=0;i<3;i++){
                g_servo_pol[i] = (int)((signed char)EEPROM.read(i*3 + 1));
                g_servo_ofs[i] = (int)((signed char)EEPROM.read(i*3 + 2));
                g_servo_amp[i] =                    EEPROM.read(i*3 + 3);
                txbuff[3+i*5] = (g_servo_pol[i]==1) ? '+' : '-';
                Uint16ToHex(&txbuff[4+i*5], (unsigned short)((unsigned char)g_servo_ofs[i]), 2);
                Uint16ToHex(&txbuff[6+i*5], (unsigned short)(               g_servo_amp[i]), 2);
            }
            txbuff[18]='$';
            txbuff[19]='\0';
            udpComm.send(txbuff);
            break;
        }
        break;
        
    /* Vコマンド(車両モードの設定)
        (1)自動車モードに設定
           書式: #VC$
        (2)戦車モードに設定
           書式: #VT$
        (3)車両モードの読み出し
           書式: #VL$
     */
    case 'V':
        switch(buff[1]){
        case 'C':
            drive_mode = MODE_CAR;
            //EEPROM.write(10, drive_mode);
            //EEPROM.commit();
            break;
        case 'T':
            drive_mode = MODE_TANK;
            //EEPROM.write(10, drive_mode);
            //EEPROM.commit();
            break;
        case 'L':
            txbuff[0]='#';
            txbuff[1]='V';
            txbuff[2]= (drive_mode == MODE_TANK) ? 'T' : 'C';
            txbuff[18]='$';
            txbuff[19]='\0';
            udpComm.send(txbuff);
            break;
        }
    }
 
}

