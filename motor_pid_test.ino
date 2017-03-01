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

PID pos0_tracking_p;
PID pos1_tracking_p;

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
  float p0, p1, u0 = 0, u1 = 0, mv0, mv1, s0, s1;
  int32_t w0, w1, d0, d1, c0, c1, n0, n1, i0, i1;
  pid_test++;

  // パルス幅と1秒辺りのパルス数を取得
  noInterrupts();
  w0 = plsin0.pulseWidth();
  p0 = plsin0.pulsePerSecond();
  w1 = plsin1.pulseWidth();
  p1 = plsin1.pulsePerSecond();
  c0 = enc0.count();
  d0 = enc0.delta();
  i0 = enc0.direction();
  c1 = enc1.count();
  d1 = enc1.delta();
  i1 = enc1.direction();
  interrupts();

  // 回転方向を加える
  w0 *= i0;
  p0 *= i0;
  w1 *= i1;
  p1 *= i1;

  if (tuning) {
    // チューニング中は何もしない
  } else {
    if (target0) {
      // 位置フィードバックループ（片方のタイヤはもう一方のタイヤに追従させる）
      s0 = pos0_tracking_p.process(target0 - c0);
      s1 = pos0_tracking_p.process(c0 - c1);
      // 速度フィードバックループ
      u0 = m0_pid.process(s0 - p0);
      u1 = m1_pid.process(s1 - p1);
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
        { target0, c0, s0, p0, mv0, u0},
        {      c0, c1, s1, p1, mv1, u1},
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
  // 位置制御PID初期化
  pos0_tracking_p.setInterval(0.001);
  pos1_tracking_p.setInterval(0.001);
  pos0_tracking_p.setGain(10, 0, 0);
  pos1_tracking_p.setGain(10, 0, 0);
  pos0_tracking_p.setOutputLimits(-1400, 1400);
  pos1_tracking_p.setOutputLimits(-1400, 1400);
  // PID制御初期化
  m0_pid.setInterval(0.001);
  m1_pid.setInterval(0.001);
  TIME(m0_pid.process(0), 1000);
  // ZN P
  //  #define KP 3.74491
  //  #define KI 0
  //  #define KD 0

  // ZN PI
  //  #define KP 3.37042
  //  #define KI 127.66741
  //  #define KD 0

  // ZN PID
  //  #define KP 4.49389
  //  #define KI 280.86830
  //  #define KD 0.01798

  //CHR 0%
#define KP 2.24695
#define KI 54.80357
#define KD 0.00899

  //CHR 20%
  //  #define KP 3.55767
  //  #define KI 64.27579
  //  #define KD 0.01338

  m0_pid.setGain(KP, KI, KD);
  m1_pid.setGain(KP, KI, KD);
  m0_pid.setOutputLimits(-1023, 1023);
  m1_pid.setOutputLimits(-1023, 1023);
  // 割り込み処理開始
  attachTickerInterrupt(0, encoder_polling, 25000);
  attachTickerInterrupt(2, pid_process, 1000);
}

uint32_t start_time = 0;

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
        m0_pid.reset();
        m1_pid.reset();
        plsin0.reset();
        plsin1.reset();
        m0.go(1023);
        m1.go(1023);
        target0 = 1023;
      } else {
        m0.stop();
        m1.stop();
        target0 = 0;
      }
    } else {
      if (target0 == 0) {
        m0_pid.reset();
        m1_pid.reset();
        plsin0.reset();
        plsin1.reset();
        target0 = 980;
        target1 = 0;
        start_time = millis();
      } else {
        m0.stop();
        m1.stop();
        target0 = target1 = 0;
      }
    }
  }

  if (target1 && (millis() - start_time > 1000)) {
    target0 = target1;
    target1 = 0;
  }
}

