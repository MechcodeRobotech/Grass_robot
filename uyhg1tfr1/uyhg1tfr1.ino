#include <Arduino.h>

#define BLYNK_TEMPLATE_ID           "TMPL69phYUjlj"
#define BLYNK_TEMPLATE_NAME         "Grass Robot"
#define BLYNK_AUTH_TOKEN            "A9ZOvMmKuQKdiXCeyR3jJlfNuluHmolw"
#define BLYNK_PRINT Serial

#include <Arduino.h>
#include <PID_v1.h>
#include <WiFi.h>
#include <BlynkSimpleEsp32.h>

char ssid[] = "HOCO HI40-8123";
char pass[] = "12345678";

BlynkTimer timer;

//// --- Wheel motor control pins ---
#define R_LeftWheelRPM 21
#define L_LeftWheelRPM 18
#define R_RightWheelRPM 22
#define L_RightWheelRPM 19

//// --- Water pump pin ---
#define trig_Waterpump 23

//// --- Line follower sensor setup ---
#define NUM_SENSORS 4
int sensorPins[NUM_SENSORS] = {33, 32, 35, 34};
int sensorValues[NUM_SENSORS];
float position = 0;  // continuous value

bool state = false;
bool runState = false; // 🟢 ใช้ควบคุมการทำงานหุ่นยนต์
int Walkround = 0;

//// --- Base motor speed and gain ---
float Kp = 2;       // gain แบบ proportional control
int baseSpeed = 50;  // ยังไม่ได้ใช้ PWM ตอนนี้

bool allBlack = false;

unsigned long lastDetectedTime = 0;  // เวลาเมื่อเจอเส้นล่าสุด (ms)
int numAllBlack = 0;

//// --- Function prototypes ---
void readLineSensor();
void forward();
void turnLeft();
void turnRight();
void stopMotors();
void detectStartLine();
void stopAll();

BLYNK_WRITE(V1)
{
  int value1 = param.asInt();
  if (value1 == 1) {
    numAllBlack = 0;
    runState = true;   // เริ่มทำงาน
    Serial.println("Robot started");
    Serial.println(Walkround);
    state = true;
  } else if(value1 == 0) {
    runState = false;  // หยุดทำงาน
    state = false;
    Serial.println("Robot stopped");
    stopAll();         // 🔴 หยุดมอเตอร์และปั๊มน้ำทันที
  }
}

BLYNK_WRITE(V2)
{
  int pumpState = param.asInt();
  Walkround = pumpState;
  // Serial.println(Walkround);
}

//// ======================================================
void setup() {
  Serial.begin(115200);
  Blynk.begin(BLYNK_AUTH_TOKEN, ssid, pass);

  // Set wheel pins
  pinMode(R_LeftWheelRPM, OUTPUT); 
  pinMode(L_LeftWheelRPM, OUTPUT);
  pinMode(R_RightWheelRPM, OUTPUT); 
  pinMode(L_RightWheelRPM, OUTPUT);

  digitalWrite(R_LeftWheelRPM, LOW);
  digitalWrite(L_LeftWheelRPM, LOW);
  digitalWrite(R_RightWheelRPM, LOW);
  digitalWrite(L_RightWheelRPM, LOW);

  pinMode(trig_Waterpump, OUTPUT);

  // Set line follower pins
  for (int i = 0; i < NUM_SENSORS; i++)
    pinMode(sensorPins[i], INPUT);

  Serial.println("Line Follower Robot Initialized!");
}

//// ======================================================
void loop() {
  Blynk.run();
  timer.run();
  readLineSensor();
  detectStartLine();

    // --- ใช้ position ในการตัดสินใจเคลื่อนที่ ---
    if (runState){
      digitalWrite(trig_Waterpump, HIGH);

      if (position > 5) { 
        turnRight();   // เส้นอยู่ทางขวา
      } 
      else if (position < -5) { 
        turnLeft();    // เส้นอยู่ทางซ้าย
      } else if (position >= -5 && position <= 5) {
        forward();     // อยู่ตรงกลาง
      }
    
      if(numAllBlack == Walkround) {
        runState = false;
      }
    }
    delay(100);
  }
//// ======================================================
// อ่านค่า line sensors และคำนวณตำแหน่งเส้น
void readLineSensor() {
  float weightedSum = 0;
  float sumValues = 0;

  for (int i = 0; i < NUM_SENSORS; i++) {
    sensorValues[i] = analogRead(sensorPins[i]);
  }

  for (int i = 0; i < NUM_SENSORS; i++) {
    int value = 4095 - sensorValues[i];  // invert ถ้าเส้นดำค่าแรง
    weightedSum += value * i;
    sumValues += value;
  }

  if (sumValues != 0)
    position = weightedSum / sumValues;
  else
    position = (NUM_SENSORS - 1) / 2.0;

  position = (position * 10) - 15.0; // scale ให้ position ประมาณ -15 ถึง +15

  //  Serial.print("Sensor: ");
  //  for (int i = 0; i < NUM_SENSORS; i++) {
  //    Serial.print(sensorValues[i]);
  //    Serial.print("\t");
  //  }
  //  Serial.print(" Pos: ");
  //  Serial.println(position, 2);
}

//// ======================================================
// ฟังก์ชันควบคุมทิศทาง
void forward() {
  analogWrite(R_LeftWheelRPM, 70);
  analogWrite(L_LeftWheelRPM, 0);
  analogWrite(R_RightWheelRPM, 0);
  analogWrite(L_RightWheelRPM, 70);
  Serial.println("Forward");
}

void turnLeft() {
  analogWrite(R_LeftWheelRPM, 0);
  analogWrite(L_LeftWheelRPM, 0);
  analogWrite(R_RightWheelRPM, 0);
  analogWrite(L_RightWheelRPM, 70);
  Serial.println("Turn Left");
}

void turnRight() {
  analogWrite(R_LeftWheelRPM, 70);
  analogWrite(L_LeftWheelRPM, 0);
  analogWrite(R_RightWheelRPM, 0);
  analogWrite(L_RightWheelRPM, 0);
  Serial.println("Turn Right");
}

void stopMotors() {
  analogWrite(R_LeftWheelRPM, 0);
  analogWrite(L_LeftWheelRPM, 0);
  analogWrite(R_RightWheelRPM, 0);
  analogWrite(L_RightWheelRPM, 0);
  Serial.println("Stop");
}

void detectStartLine() {
  // อ่านค่าสีจาก sensor แต่ละตัว (คุณอาจจะมีฟังก์ชันอ่านแยกอยู่แล้ว)
  int sensorValues[4];
  sensorValues[0] = analogRead(32);
  sensorValues[1] = analogRead(33);
  sensorValues[2] = analogRead(35);
  sensorValues[3] = analogRead(34);

  // ✅ ถ้าเจอเส้นดำทุกตัว และเวลาผ่านไปมากกว่า 5 วินาทีจากครั้งล่าสุด
  if ((sensorValues[0] < 2000) && (sensorValues[1] < 2000) &&
      (sensorValues[2] < 2000) && (sensorValues[3] < 2000)) {

    unsigned long currentTime = millis();

    if ((currentTime - lastDetectedTime >= 5000) && (state == true)) {  // 5000 ms = 5 วินาที
      numAllBlack += 1;
      lastDetectedTime = currentTime;  // อัปเดตเวลาเจอล่าสุด
      Serial.print("✅ พบเส้นขวางครั้งที่: ");
      Serial.println(numAllBlack);
      if(Walkround == numAllBlack){
        stopAll();
        runState = false;
        state = false;
      }
    }
  }
}

void stopAll() {
  stopMotors();
  digitalWrite(trig_Waterpump, LOW);
}