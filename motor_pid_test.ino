#include <Bounce.h>
#include "Ticker.h"
#include "MotorDriver.h"
#include "PulseIn.h"
#include "Encoder.h"
#include "PID.h"
#include "Ring.h"
#include "Logging.h"

//ピン割り当て
const int m0_pwm = 6;
const int m0_in1 = 5;
const int m0_in2 = 10;
const int enc0_A = 1;
const int enc0_B = 0;

const int m1_pwm = 11;
const int m1_in1 = 20;  // <-21
const int m1_in2 = 21;  // <-20 反対側のモータは逆方向に回転させる
const int enc1_A = 13;
const int enc1_B = 12;

const int stby = 9;
const int button = 19;  // A5

MotorDriver m0(m0_in1, m0_in2, m0_pwm);
MotorDriver m1(m1_in1, m1_in2, m1_pwm);

PulseIn plsin0(enc0_A, enc0_B);
PulseIn plsin1(enc1_A, enc1_B);

Encoder enc0(enc0_A, enc0_B);
Encoder enc1(enc1_A, enc1_B);

PID m0_pid;
PID m1_pid;

Bounce btn = Bounce(button, 10);

Ring<Log, 128> plog;

uint32_t polling_test = 0;
void encoder_polling(void)
{
  enc0.poll();
  enc1.poll();
  polling_test++;
}

volatile int32_t target0 = 0; // パルス/秒
volatile int32_t target1 = 0; // パルス/秒
int tuning = 0;

uint32_t pid_test = 0;
void pid_process(void)
{
  float p0, p1, u0 = 0, u1 = 0, mv0, mv1;
  int w0, w1, d0, d1, c0, c1, n0, n1;
  pid_test++;

  // パルス幅と1秒辺りのパルス数を取得
  noInterrupts();
  w0 = plsin0.pulseWidth();
  p0 = plsin0.pulsePerSecond();
  w1 = plsin1.pulseWidth();
  p1 = plsin1.pulsePerSecond();
  c0 = enc0.count();
  d0 = enc0.delta();
  n0 = enc0.direction();
  c1 = enc1.count();
  d1 = enc1.delta();
  n1 = enc1.direction();
  interrupts();

  p0 *= n0;
  p1 *= n1;

  if (tuning) {
    m0.go(target0);
    m1.go(target0);
  } else {
    // フィードバック制御
    if (target0) {
      u0 = m0_pid.process(((float)target0) - p0);
      u1 = m1_pid.process(((float)target0) - p1);
      mv0 = m0_pid._previous_output;
      mv1 = m1_pid._previous_output;
      m0.go(u0);
      m1.go(u1);
    }
  }

  if (d0 || d1 || target0) {
    Log stat = {
      micros(),
      {
        { target0, p0, mv0, u0},
        { target0, p1, mv1, u1},
      },
    };
    plog.put(stat);
  }
}

#define TIME(s,n) do {\
    uint32_t t = micros();\
    for(int i=0; i<n; i++){\
      (s);\
    }\
    t = micros() - t;\
    Serial.print(#s);\
    Serial.print(':');\
    Serial.println(t);\
  }while(0);

void setup() {
  // シリアルポート初期化
  Serial.begin(250000);
  while (!Serial);
  // ボタン入力
  pinMode(button, INPUT_PULLUP);
  // モータドライバ初期化
  m0.begin();
  m1.begin();
  // モータドライバスタンバイ解除
  pinMode(stby, OUTPUT);
  digitalWrite(stby, HIGH);
  // エンコーダ初期化
  enc0.begin();
  enc1.begin();
  TIME(enc0.poll(), 1000);
  // パルス入力初期化
  plsin0.begin();
  plsin1.begin();
  TIME(plsin1.poll(), 1000);
  // PID制御初期化
  m0_pid.setInterval(0.001F);
  m1_pid.setInterval(0.001F);
  TIME(m0_pid.process(0), 1000);
  // ZN P
  //  #define KP 3.74491F
  //  #define KI 0.0F
  //  #define KD 0.0F

  // ZN PI
  //  #define KP 3.37042F
  //  #define KI 127.66741F
  //  #define KD 0.0F

  // ZN PID
  //  #define KP 4.49389F
  //  #define KI 280.86830F
  //  #define KD 0.01798F

  //CHR 0%（空転）
  //#define KP 2.24695F
  //#define KI 54.80357F
  //#define KD 0.00899F

  //CHR 0%（自走）
#define KP 5.71824F
#define KI 62.83784F
#define KD 0.02287F

  //CHR 20%
  //  #define KP 3.55767F
  //  #define KI 64.27579F
  //  #define KD 0.01338F

  m0_pid.setGain(KP, KI, KD);
  m1_pid.setGain(KP, KI, KD);
  m0_pid.setOutputLimits(-1023, 1023);
  m1_pid.setOutputLimits(-1023, 1023);
  // 割り込み処理開始
  attachTickerInterrupt(0, encoder_polling, 25000);
  attachTickerInterrupt(2, pid_process, 1000);
}

uint32_t start_time = 0;
uint32_t shift_change = 0;

void loop()
{
  //  static uint32_t last;
  //  uint32_t t = millis();
  //  if (t - last > 1000) {
  //    Serial.println(pid_test);
  //    pid_test = 0;
  //    Serial.println(polling_test);
  //    polling_test = 0;
  //    last = t;
  //  }

  if (plog.count())
    plog.get().printTo(Serial);

  btn.update();
  if ( btn.fallingEdge() ) {
    if (tuning) {
      if (target0 == 0) {
        plsin0.reset();
        plsin1.reset();
        target0 = 1023;
        target1 = 0;
        start_time = millis();
        shift_change = 2000;
      } else {
        m0.stop();
        m1.stop();
        target0 = target1 = 0;
      }
    } else {
      if (target0 == 0) {
        m0_pid.reset();
        m1_pid.reset();
        plsin0.reset();
        plsin1.reset();
        target0 = 1000;
        target1 = 10;
        start_time = millis();
        shift_change = 2000;
      } else {
        m0.stop();
        m1.stop();
        target0 = target1 = 0;
      }
    }
  }

  if (shift_change && (millis() - start_time > shift_change)) {
    target0 = target1;
    shift_change = 0;
  }
}

