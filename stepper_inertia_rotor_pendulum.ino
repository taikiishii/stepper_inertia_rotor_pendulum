#include <Wire.h>
#include <MsTimer2.h>
#include <TimerOne.h>

// k1:傾き　k2:倒れる速度　k3:ロータの回転角　k4:ロータの回転速度
volatile float k1 = 939, k2 = 39, k3 = 22, k4 = 6;  //SPG20-1332 マイクロステップなし

#define DIR   4
#define STEP  5
#define EN    6

#define PULSE_PERIOD   10// パルス生成のためのTimer1のタイマ割り込み周期 マイクロ秒単位
#define CONTROL_PERIOD 4 // 制御ループの周期 ミリ秒単位
#define LIMIT 200        // ステッピングモータが脱調しない最大のスピード SPG20-1332=150

volatile long lastUptime, starttime, currentTime;
volatile float dt;
volatile int16_t rawGyroY;
volatile float caribGyroY;
volatile float gyroY, degY = 0, dpsY = 0, rotY = 0;
volatile bool out = false;
volatile int16_t pulseSpeed = 0, count = 0;
volatile float lpfY, lpfA = 0.999;

// 加速度・ジャイロセンサーの制御定数
#define MPU6050_ADDR         0x68 // MPU-6050 device address
#define MPU6050_SMPLRT_DIV   0x19 // MPU-6050 register address
#define MPU6050_CONFIG       0x1a
#define MPU6050_GYRO_CONFIG  0x1b
#define MPU6050_ACCEL_CONFIG 0x1c
#define MPU6050_ACCEL_XOUT_H 0x3b
#define MPU6050_ACCEL_XOUT_L 0x3c
#define MPU6050_ACCEL_YOUT_H 0x3d
#define MPU6050_ACCEL_YOUT_L 0x3e
#define MPU6050_ACCEL_ZOUT_H 0x3f
#define MPU6050_ACCEL_ZOUT_L 0x40
#define MPU6050_GYRO_XOUT_H  0x43
#define MPU6050_GYRO_XOUT_L  0x44
#define MPU6050_GYRO_YOUT_H  0x45
#define MPU6050_GYRO_YOUT_L  0x46
#define MPU6050_GYRO_ZOUT_H  0x47
#define MPU6050_GYRO_ZOUT_L  0x48
#define MPU6050_PWR_MGMT_1   0x6b
#define MPU6050_WHO_AM_I     0x75

// センサーへのコマンド送信
void writeMPU6050(byte reg, byte data) {
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(reg);
  Wire.write(data);
  Wire.endTransmission();
}

// センサーからのデータ読み込み
byte readMPU6050(byte reg) {
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU6050_ADDR, 1, false);
  //readで読み取れるバイト数がなければLED13を消灯
  while (! Wire.available()) {
    digitalWrite(13, LOW);
  }
  byte data =  Wire.read();
  Wire.endTransmission(true);
  return data;
}

void setup() {
  Wire.setClock(400000);
  Serial.begin(115200);
  Serial.println("*******************RESTARTED********************");
  Serial.println("*******stepper_inertia_rotor_pendulum***********");

  // モータードライバ制御用ピンの初期化
  pinMode(DIR, OUTPUT);
  pinMode(STEP, OUTPUT);
  pinMode(EN, OUTPUT);

  //状態をLED 13にて表示
  pinMode(13, OUTPUT);

  // センサーの初期化
  Wire.begin();
  if (readMPU6050(MPU6050_WHO_AM_I) != 0x68) {
    Serial.println("WHO_AM_I error.");
    while (true) ;
  }
  else {
    Serial.println("WHO_AM_I OK.");
  }
  // see https://www.invensense.com/wp-content/uploads/2015/02/MPU-6000-Register-Map1.pdf
  writeMPU6050(MPU6050_SMPLRT_DIV, 0x07);   // sample rate: 8kHz/(7+1) = 1kHz
  writeMPU6050(MPU6050_CONFIG, 0x00);       // disable DLPF, gyro output rate = 8kHz
  writeMPU6050(MPU6050_GYRO_CONFIG, 0x00);  // gyro range: 0x00⇒±250dps 131LSB、0x08⇒±500dps 65.5LSB、0x10⇒±1000dps 32.8LSB、0x18⇒±2000dps 16.4LSB
  writeMPU6050(MPU6050_ACCEL_CONFIG, 0x00); // accel range: 0x00⇒±2g 16384LSB/g、0x01⇒±4g 8192LSB/g、0x02⇒±8g 4096LSB/g、0x03⇒±16g 2048LSB/g、
  writeMPU6050(MPU6050_PWR_MGMT_1, 0x01);   // disable sleep mode, PLL with X gyro
  Serial.println("MPU6050 Setup OK."); delay(2000);

  //ジャイロのゼロ点調整のために静止時の出力を1000回計測して平均を算出
  caribGyroY = 0;
  for (int i = 0; i < 1000  ; i++)  {
    rawGyroY = (readMPU6050(MPU6050_GYRO_YOUT_H) << 8) | readMPU6050(MPU6050_GYRO_YOUT_L);
    caribGyroY += (float) rawGyroY;
  }
  caribGyroY /= 1000;
  Serial.println("Carib OK.");

  // dt計測用
  lastUptime = micros();

  //　倒立時間計測用
  starttime = micros();

  digitalWrite(EN, LOW);

  Timer1.initialize(PULSE_PERIOD); //パルス生成のためのタイマ割込み 引数はマイクロ秒単位
  Timer1.attachInterrupt(pulse);
  MsTimer2::set(CONTROL_PERIOD, controlloop); //制御のためのタイマ割込み 引数はミリ秒単位
  MsTimer2::start();

  //準備が出来たらLDE 13を点灯
  digitalWrite(13, HIGH);
  Serial.println("******************** GO !! *********************");
  //Serial.println("degY,dpsY,pulseSpeed");
}

void pulse() {
  count += pulseSpeed;
  if (count > 10000) {
    //digitalWrite(STEP, out);
    //out = !out;
    PORTD ^= B00100000;
    count -= 10000;
  }
  else if (count < -10000) {
    //digitalWrite(STEP, out);
    //out = !out;
    PORTD ^= B00100000;
    count += 10000;
  }
}

void controlloop () {
  // 割り込みハンドラの中でI2C通信(割り込み処理を使用）を許可する
  interrupts();

  // dt計測
  currentTime = micros();
  dt = (currentTime - lastUptime) * 0.000001;
  lastUptime = currentTime;

  // 角速度を取得
  rawGyroY = (readMPU6050(MPU6050_GYRO_YOUT_H) << 8) | readMPU6050(MPU6050_GYRO_YOUT_L);
  gyroY =  (float) rawGyroY - caribGyroY;
  dpsY = gyroY / 131.0;

  // 角速度を積算して角度を求める
  degY +=  dpsY * dt;

  //ローパスフィルタでドリフトを補正
  lpfY *=  lpfA;
  lpfY +=  (1 - lpfA) * degY;

  // ロータの速度を積算して回転角を求める
  rotY += pulseSpeed * dt;

  // 制御量の計算
  pulseSpeed += (k1 * (degY - lpfY) + k2 * dpsY + k3 * rotY + k4 * pulseSpeed) * dt;

  // ステッピングモータの最大速度を制限
  pulseSpeed = constrain(pulseSpeed, 0 - LIMIT, LIMIT);

  if (pulseSpeed > 0) {
    // digitalWrite(DIR, LOW);
    PORTD &= ~B00010000;
  }
  else {
    //digitalWrite(DIR, HIGH);
    PORTD |= B00010000;
  }

  // 倒れたらモーター停止
  if (20 < abs(degY - lpfY)) {
    pulseSpeed = 0;
    digitalWrite(EN, HIGH);
    Serial.println("*********************STOP***********************");

    while (1) {
      // LED13を点滅
      digitalWrite(13, LOW);
      delay(500);
      digitalWrite(13, HIGH);
      delay(500);
    }
  }
}

void loop() {
}
