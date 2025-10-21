#include <EEPROM.h>
#include <SoftwareSerial.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

// ----------------- Pin Definitions -----------------
#define FLEX_PIN   A0
#define BUTTON_PIN 2
#define EEPROM_ADDR 0
#define EEPROM_MID_ADDR (EEPROM_ADDR + sizeof(int))
#define EEPROM_MAX_ADDR (EEPROM_MID_ADDR + sizeof(int))

// ----------------- Bluetooth -----------------
SoftwareSerial BTSerial(10, 11); // RX, TX

// ----------------- Flex Variables -----------------
int baselineFlex = 0;
int midFlex = 0;
int flexMax = 0;
int calibState = 0;

float a=0, b=0, c=0;

// ----------------- BNO055 Variables -----------------
Adafruit_BNO055 bno = Adafruit_BNO055(55);
const int BA = 200;
const int DETECT_SAMPLES = 150;
const int SMOOTH_N = 6;

float baseline_euler[3] = {0,0,0};
int abduction_channel = -1;
int rotation_channel = -1;
float smoothA_buf[SMOOTH_N];
float smoothR_buf[SMOOTH_N];
int smooth_idx = 0;

// ----------------- Flags -----------------
bool flexCalibrated = false;
bool bnoBaselineSet = false;
bool bnoChannelDetected = false;
bool isCalibrating = false;

// ----------------- Setup -----------------
void setup() {
  Serial.begin(115200);
  BTSerial.begin(9600);
  pinMode(FLEX_PIN, INPUT);
  pinMode(BUTTON_PIN, INPUT_PULLUP);

  if (!bno.begin()) {
    Serial.println("BNO055 not detected. Check wiring.");
    while (1) delay(10);
  }
  bno.setExtCrystalUse(true);

  EEPROM.get(EEPROM_ADDR, baselineFlex);
  EEPROM.get(EEPROM_MID_ADDR, midFlex);
  EEPROM.get(EEPROM_MAX_ADDR, flexMax);
  if (baselineFlex && midFlex && flexMax) {
    computeQuadratic();
    flexCalibrated = true;
    Serial.println("Flex calibration loaded from EEPROM.");
  }

  Serial.println("Elbow Goniometer + Thigh BNO System Ready");
  Serial.println("Button: Flex calib | 'z': baseline | 'd': detect abduction | 's': stream");
}

// ----------------- Loop -----------------
void loop() {
  int flexRaw = analogRead(FLEX_PIN);

  if (digitalRead(BUTTON_PIN) == LOW) {
    delay(50);
    while (digitalRead(BUTTON_PIN) == LOW);
    isCalibrating = true;
    handleFlexCalibration(flexRaw);
  }

  if (Serial.available()) {
    char c = Serial.read();
    if (c == 'z') {
      isCalibrating = true;
      captureBaseline();
      bnoBaselineSet = true;
      isCalibrating = false;
    } else if (c == 'd') {
      isCalibrating = true;
      detectAbductionChannel();
      bnoChannelDetected = true;
      isCalibrating = false;
    } else if (c == 's') {
      if (!bnoBaselineSet || !bnoChannelDetected) {
        Serial.println("Run 'z' and 'd' first!");
      } else {
        Serial.println("Streaming flexion + abduction + rotation...");
      }
    }
  }

  // Main angle computation
  if (!isCalibrating && flexCalibrated && bnoBaselineSet && bnoChannelDetected) {
    int correctedFlex = flexRaw - baselineFlex;
    float elbowAngle = a*correctedFlex*correctedFlex + b*correctedFlex + c;
    elbowAngle = constrain(elbowAngle, 0, 130);

    imu::Vector<3> e = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
    float v[3] = {e.x(), e.y(), e.z()};

    float rawA = v[abduction_channel];
    float rawR = v[rotation_channel];

    float abductionAngle = angleDifference(rawA, baseline_euler[abduction_channel]);
    float rotationAngle  = angleDifference(rawR, baseline_euler[rotation_channel]);

    // Smooth both
    smoothA_buf[smooth_idx] = abductionAngle;
    smoothR_buf[smooth_idx] = rotationAngle;
    smooth_idx = (smooth_idx + 1) % SMOOTH_N;
    float sumA=0, sumR=0;
    for (int i=0;i<SMOOTH_N;i++) {
      sumA += smoothA_buf[i];
      sumR += smoothR_buf[i];
    }
    float smoothedA = sumA / SMOOTH_N;
    float smoothedR = sumR / SMOOTH_N;

    // Print
    Serial.print("F:");
    Serial.print(elbowAngle, 1);
    Serial.print(", A:");
    Serial.print(smoothedA, 1);
    Serial.print(", R:");
    Serial.println(smoothedR, 1);

    BTSerial.print("F:");
    BTSerial.print(elbowAngle, 1);
    BTSerial.print(", A:");
    BTSerial.print(smoothedA, 1);
    BTSerial.print(", R:");
    BTSerial.println(smoothedR, 1);
  }
  delay(200);
}

// ----------------- Flex Calibration -----------------
void handleFlexCalibration(int flexRaw) {
  if (calibState == 0) {
    baselineFlex = flexRaw;
    EEPROM.put(EEPROM_ADDR, baselineFlex);
    calibState = 1;
    Serial.println("Baseline (0°) recorded. Move to 90° and press button.");
  } else if (calibState == 1) {
    midFlex = flexRaw;
    EEPROM.put(EEPROM_MID_ADDR, midFlex);
    calibState = 2;
    Serial.println("Midpoint (90°) recorded. Move to ~130° and press button.");
  } else if (calibState == 2) {
    flexMax = flexRaw;
    EEPROM.put(EEPROM_MAX_ADDR, flexMax);
    calibState = 0;
    computeQuadratic();
    flexCalibrated = true;
    Serial.println("Max (130°) recorded. Calibration complete!");
    isCalibrating = false;
  }
}

// ----------------- Compute Quadratic -----------------
void computeQuadratic() {
  c = 0;
  float x1 = midFlex - baselineFlex;
  float y1 = 90;
  float x2 = flexMax - baselineFlex;
  float y2 = 130;
  b = (y1*x2*x2 - y2*x1*x1) / (x2*x1*x2 - x1*x2*x1);
  a = (y1 - b*x1) / (x1*x1);
  Serial.print("Quadratic coeffs: a="); Serial.print(a);
  Serial.print(", b="); Serial.println(b);
}

// ----------------- BNO Functions -----------------
void captureBaseline() {
  Serial.println("Capturing BNO baseline...");
  float sum[3] = {0,0,0};
  for (int i=0;i<BA;i++){
    imu::Vector<3> e = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
    sum[0] += e.x(); sum[1] += e.y(); sum[2] += e.z();
    delay(12);
  }
  for (int i=0;i<3;i++) baseline_euler[i] = sum[i]/BA;
  Serial.print("Baseline set: ");
  Serial.print(baseline_euler[0]); Serial.print(", ");
  Serial.print(baseline_euler[1]); Serial.print(", ");
  Serial.println(baseline_euler[2]);
}

void detectAbductionChannel() {
  Serial.println("Detecting abduction channel...");
  float mean[3]={0,0,0}, sqsum[3]={0,0,0};
  for (int i=0;i<DETECT_SAMPLES;i++){
    imu::Vector<3> e = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
    float v[3]={e.x(), e.y(), e.z()};
    for (int k=0;k<3;k++){
      mean[k]+=v[k];
      sqsum[k]+=v[k]*v[k];
    }
    delay(12);
  }
  float var[3];
  for (int k=0;k<3;k++){
    mean[k]/=DETECT_SAMPLES;
    var[k]=sqsum[k]/DETECT_SAMPLES - mean[k]*mean[k];
  }
  int best=0;
  if (var[1]>var[best]) best=1;
  if (var[2]>var[best]) best=2;
  abduction_channel=best;

  // Automatically assign rotation channel (different axis)
  rotation_channel = (best == 0) ? 2 : 0;

  Serial.print("Detected abduction channel: ");
  if (best==0) Serial.println("Heading (yaw)");
  if (best==1) Serial.println("Roll");
  if (best==2) Serial.println("Pitch");

  Serial.print("Rotation channel set to: ");
  if (rotation_channel==0) Serial.println("Heading (yaw)");
  if (rotation_channel==1) Serial.println("Roll");
  if (rotation_channel==2) Serial.println("Pitch");
}

// ----------------- Utility -----------------
float angleDifference(float a, float b) {
  float diff = a - b;
  while (diff > 180.0) diff -= 360.0;
  while (diff < -180.0) diff += 360.0;
  return diff;
}
