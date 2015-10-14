#include <TinyGPS++.h> //GPSデータを受信し終わるまで待機GPS++を使用するためにヘッダファイルをインクルード
#include <SoftwareSerial.h> //SoftwareSerialを使用するためにヘッダファイルをインクルード

/////////////ソフトウェアシリアルで使うピン/////////////////////////////////////
#define SSRX 10    //ソフトウェアシリアルでrxとして使用するピンをSSRXとして設定
#define SSTX  9    //ソフトウェアシリアルでtxとして使用するピンをSSTXとして設定
///////////////////////////////////////////////////////////////////////////////////

////////////モータ制御ピン/////////////
#define LFPin 6  //電圧をかけると左のモータが正転するマイコンのピン番号をLFPinとして設定
#define LBPin 7  //電圧をかけると左のモータが後転するマイコンのピン番号をLBPinとして設定
#define RFPin 5  //電圧をかけると右のモータが正転するマイコンのピン番号をRFPinとして設定
#define RBPin 4  //電圧をかけると右のモータが後転するマイコンのピン番号をRBPinとして設定
////////////////////////////////////////

/////////////ゴール関係//////////////////////
#define GOAL_LAT 35.515873   //ゴール地点の緯度をGOAL_LATとして設定
#define GOAL_LON 134.171707   //ゴール地点の経度をGOAL_LONとして設定
#define GOAL_RANGE 1  //ゴール地点の範囲[m]
////////////////////////////////////////////////

TinyGPSPlus gps;                //座標関係の各種演算をするTinyGPSPlusのオブジェクトを生成
SoftwareSerial ss(SSRX, SSTX);  //ソフトウェアシリアルのオブジェクトssをrxピンをSSRX(ピン10),txピンをSSTX(ピン9)で設定し、生成する

//////////座標関係///////////////
float originFlat_deg;    //原点座標：緯度[°]
float originFlon_deg;    //原点座標：経度[°]
float currentFlat_deg;   //現在の座標：緯度[°]
float currentFlon_deg;   //現在の座標：経度[°]
float goal_angle;       //北の方位を基準にしたゴール地点の角度[°]
/////////////////////////////////

////////////////制御定数////////////////////////////
const float p_gain = 0.3; //比例定数。紙が定めた奇跡の値を調整していってください
const int max_str = 125;  //モータに入力する値の上限値
const int min_str = 0;   //モータに入力する値の下限値
const int nstr = 125;     //ニュートラル（曲がらず直進する）で進むときの左右のモータに与える入力値
/////////////////////////////////////////////////////

void setup() {
  // put your setup code here, to run once:
  //モータ制御用のピンを出力で使用できるように設定
  pinMode(LFPin, OUTPUT);
  pinMode(LBPin, OUTPUT);
  pinMode(RFPin, OUTPUT);
  pinMode(RBPin, OUTPUT);

  //ハードウェアシリアル（備え付けのシリアル通信窓口)を9600[bps]で開始する
  Serial.begin(9600);
  //ソフトウェアシリアル通信をボーレート9600[bps]で開始する
  ss.begin(9600);

  gelay(1000); //GPSデータを受信し終わるまで待機。だいたい１秒かかる

  originFlat_deg = gps.location.lat(); //受信した緯度を原点座標の緯度に設定
  originFlon_deg = gps.location.lng(); //受信した経度を原点座標の経度に設定

  goal_angle = TinyGPSPlus::courseTo(originFlat_deg, originFlon_deg, GOAL_LAT, GOAL_LON); //北の方位から見たゴール座標の角度を求める
  
  //北の方位から見た機体の角度を求めるため、原点座標から10秒間前進して離れる（原点座標にとどまると機体の角度が求められない)
  motor_control(255, 255);
  delay(30000);
  motor_control(0, 0);
}

void loop() {
  // put your main code here, to run repeatedly:
  unsigned long int distance_m;  //機体の現在地からゴールまでの距離
  float current_angle;           //機体から見たゴールの座標(-180～180)[°]
  int control_value;

  gelay(1000); //GPSデータを受信し終わるまで待機
  currentFlat_deg = gps.location.lat();  //受信した座標の緯度を現在地の座標の緯度として設定
  currentFlon_deg = gps.location.lng();  //受信した座標の経度を現在地の座標の経度として設定

  //現在地からゴールまでの距離を求める
  distance_m = (unsigned long)TinyGPSPlus::distanceBetween(currentFlat_deg, currentFlon_deg,
               GOAL_LAT, GOAL_LON);
  Serial.print("dis ="); Serial.println(distance_m);
  //求めた距離が0だったらその地点がゴールなので終了
  if (distance_m < GOAL_RANGE) {
    Serial.println("goal");
    motor_control(0, 0);
    return;
  }

  //北の方位から見たときの現在の機体の角度を求める
  current_angle = TinyGPSPlus::courseTo(originFlat_deg, originFlon_deg, currentFlat_deg, currentFlon_deg);
  
  current_angle = goal_angle - current_angle;  //機体からみたときのゴール地点への角度を計算する

 //ゴールから機体までの角度を範囲(180～-180)の範囲に変換
  if(current_angle > 180) current_angle = current_angle - 360;
  else if(current_angle < -180) current_angle = current_angle + 360;
  
  Serial.print("cur_angle ="); Serial.println(current_angle);
  control_value = p_gain * current_angle;   //求めた角度をもとに制御値を計算する

  //制御値を設定した上限値と下限値の間に収まるように設定する
  if (control_value < 0)
    control_value = -constrain(abs(control_value), min_str, max_str);
  else
    control_value = constrain(abs(control_value), min_str, max_str);
  Serial.print("cont = "); Serial.println(control_value);
  motor_control(nstr + control_value, nstr - control_value);   //求めた制御値をモータに入力
}

/*GPSデータ受信用関数*/
/*     引数：なし
 *   返り値：GPSデータを受信できたとき 0
 *           GPSデータを受信できなかったとき -1
 */
int recvGPS(void)
{
  Serial.print("ava = "); Serial.println(ss.available());
  if (ss.available() == 0) return (-1);
  //データを受信したら
  while (ss.available())  //バッファ(俺がポストっていったやつ)に入っているデータを全部取りだす。
    gps.encode(ss.read());     //バッファからデータを1byteずつ取り出す

  return (0); //ちゃんと受信したときは0を返して終了
}

/*GPSデータ受信用関数。少なくともmsミリ秒待つ。（これをしないとモータ動かしたときに変な感じになった）
 * 基本的にrecvGPSは使わずこっちを使うように
 * 引数：unsigned long ms:待つ時間[ms]
 * 返り値：なし
 */
void gelay(unsigned long ms)
{
  unsigned long start = millis();
  do
  {
    while (ss.available())
      gps.encode(ss.read());
  } while (millis() - start < ms);
}

/*左右のモータを制御する関数*/
/*      引数：int motorL  左のモータに入力するPWMのデューティ比(0 ～　255)
 *            int motorR  右のモータに入力するPWMのデューティ比(0 ～　255)
 *    返り値：問題なく終了したとき 0
 */
int motor_control(int motorL, int motorR)
{

  if (motorL >= 0 && motorR <= 0) {
    analogWrite(LFPin, motorL);
    digitalWrite(LBPin, LOW);
    digitalWrite(RFPin, LOW);
    digitalWrite(RBPin, -motorR);
  }

  else if (motorL <= 0 && motorR >= 0) {
    digitalWrite(LFPin, LOW);
    analogWrite(LBPin, -motorL);
    analogWrite(RFPin, motorR);
    digitalWrite(RBPin, LOW);
  }

  else if (motorR <= 0 && motorL <= 0) {
    digitalWrite(LFPin, LOW);
    analogWrite(LBPin, -motorL);
    digitalWrite(RFPin, LOW);
    analogWrite(RBPin, -motorR);
  }

  else {
    analogWrite(LFPin, motorL);
    digitalWrite(LBPin, LOW);
    analogWrite(RFPin, motorR);
    digitalWrite(RBPin, LOW);
  }

  return (0);
}

