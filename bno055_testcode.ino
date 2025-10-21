/* 
  BNO055 thigh abduction/adduction example
  - Reads Euler angles from BNO055
  - Calibrates a neutral baseline
  - Detects which Euler channel corresponds to abduction (by asking user to do a small abduction)
  - Streams abduction angle = (measured_channel - baseline) with simple smoothing

  Wiring:
  - BNO055 VCC -> 3.3V (or 5V if your breakout supports it)
  - GND -> GND
  - SDA -> A4 (Uno) or SDA pin on your board
  - SCL -> A5 (Uno) or SCL pin on your board
*/

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

Adafruit_BNO055 bno = Adafruit_BNO055(55);

const int BA = 200; // baseline averaging samples
const int DETECT_SAMPLES = 150;
const int SMOOTH_N = 6; // moving average window for output

float baseline_euler[3] = {0,0,0}; // heading, roll, pitch
int abduction_channel = -1; // 0 = heading, 1 = roll, 2 = pitch
float smooth_buf[SMOOTH_N];
int smooth_idx = 0;

void setup() {
  Serial.begin(115200);
  delay(100);
  Serial.println("BNO055 abduction/adduction demo");
  
  if (!bno.begin()) {
    Serial.println("BNO055 not detected. Check wiring.");
    while (1) delay(10);
  }
  // Use NDOF fusion mode
  bno.setExtCrystalUse(true);
  delay(100);
  
  Serial.println("Move the sensor in a figure-8 to calibrate magnetometer if not already calibrated.");
  Serial.println("Open Serial Monitor at 115200.");
  Serial.println();
  Serial.println("1) Place subject in neutral standing position and type 'z' then ENTER to set baseline.");
  Serial.println("2) After baseline, type 'd' then ENTER and perform a small abduction (lift leg slightly sideways) repeatedly for ~4 seconds");
  Serial.println("   —the code will detect which Euler channel changes most and choose it as abduction channel.");
  Serial.println("3) Type 's' to start streaming abduction angle.");
}

void loop() {
  if (Serial.available()) {
    char c = Serial.read();
    if (c == 'z') {
      captureBaseline();
    } else if (c == 'd') {
      detectAbductionChannel();
    } else if (c == 's') {
      if (abduction_channel < 0) {
        Serial.println("Abduction channel not detected yet. Run 'd' first.");
      } else {
        Serial.println("Streaming abduction angle. Send 'q' to quit streaming.");
        streamLoop();
      }
    }
    // flush remaining line
    while (Serial.available()) Serial.read();
  }
  delay(20);
}

void captureBaseline() {
  Serial.println("Capturing baseline: hold neutral pose steady...");
  float sum[3] = {0,0,0};
  for (int i=0;i<BA;i++){
    imu::Vector<3> e = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
    // Adafruit returns Euler: heading (z), roll (x), pitch (y) in degrees
    float heading = e.x();
    float roll    = e.y();
    float pitch   = e.z();
    // map to array: [heading, roll, pitch] matching doc (we'll keep this mapping)
    sum[0] += heading;
    sum[1] += roll;
    sum[2] += pitch;
    delay(12);
  }
  baseline_euler[0] = sum[0]/BA;
  baseline_euler[1] = sum[1]/BA;
  baseline_euler[2] = sum[2]/BA;
  Serial.print("Baseline set: heading=");
  Serial.print(baseline_euler[0]);
  Serial.print("  roll=");
  Serial.print(baseline_euler[1]);
  Serial.print("  pitch=");
  Serial.println(baseline_euler[2]);
}

void detectAbductionChannel() {
  Serial.println("Detecting abduction channel. -> Now perform a small abduction back-and-forth for ~4 seconds.");
  float var[3] = {0,0,0};
  float mean[3] = {0,0,0};
  float sqsum[3] = {0,0,0};
  
  for (int i=0;i<DETECT_SAMPLES;i++){
    imu::Vector<3> e = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
    float v[3];
    v[0] = e.x(); // heading
    v[1] = e.y(); // roll
    v[2] = e.z(); // pitch
    for (int k=0;k<3;k++){
      mean[k] += v[k];
      sqsum[k] += v[k]*v[k];
    }
    delay(12);
  }
  for (int k=0;k<3;k++){
    mean[k] /= DETECT_SAMPLES;
    float variance = sqsum[k]/DETECT_SAMPLES - mean[k]*mean[k];
    var[k] = variance;
  }
  // choose channel with largest variance
  int best = 0;
  if (var[1] > var[best]) best = 1;
  if (var[2] > var[best]) best = 2;
  abduction_channel = best;
  Serial.print("Detection complete. Channel: ");
  if (best==0) Serial.println("0 = heading (yaw)");
  if (best==1) Serial.println("1 = roll");
  if (best==2) Serial.println("2 = pitch");
  Serial.print("Variances (heading, roll, pitch): ");
  Serial.print(var[0]); Serial.print(", ");
  Serial.print(var[1]); Serial.print(", ");
  Serial.println(var[2]);
  Serial.println("Now press 's' to stream abduction angle relative to baseline.");
}

void streamLoop() {
  // initialize smoothing buffer
  for (int i=0;i<SMOOTH_N;i++) smooth_buf[i]=0;
  smooth_idx = 0;
  while (true) {
    if (Serial.available()) {
      char c = Serial.read();
      if (c=='q') {
        Serial.println("Stopping stream.");
        break;
      }
    }
    imu::Vector<3> e = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
    float v[3];
    v[0] = e.x();
    v[1] = e.y();
    v[2] = e.z();
    float rawAngle = v[abduction_channel];
    float ang = angleDifference(rawAngle, baseline_euler[abduction_channel]);
    // smooth (moving average)
    smooth_buf[smooth_idx] = ang;
    smooth_idx = (smooth_idx+1) % SMOOTH_N;
    float s=0;
    for (int i=0;i<SMOOTH_N;i++) s += smooth_buf[i];
    float out = s / SMOOTH_N;
    Serial.print("Abd angle (deg): ");
    Serial.println(out, 2);
    delay(20); // ~50Hz
  }
}

// computes shortest signed difference between two angles in degrees
float angleDifference(float a, float b) {
  float diff = a - b;
  while (diff > 180.0) diff -= 360.0;
  while (diff < -180.0) diff += 360.0;
  return diff;
}
 