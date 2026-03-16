#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <SD.h>
#include <SPI.h>

// Built in SD card for Teensy 4.1
const int chipSelect = BUILTIN_SDCARD;
File dataFile;

// Motors can be disabled for hardware demo
// Since the reaction wheels cant generate enough torque to overcome friction
// of the frame demo can be limited to showing the controller run and the system response
const bool motorsEnabled = false;

// PWM pins for each reaction wheel
const int ESC_PIN[4] = {0, 1, 2, 3};

// ESC constants (must match mc_parameters.c in MC SDK)
const int PULSE_NEUTRAL       = 1500;
const int PULSE_FWD_MIN       = 1551;
const int PULSE_FWD_MAX       = 1940;
const int PULSE_REV_MIN       = 1449;
const int PULSE_REV_MAX       = 1060;
const int PULSE_DEADBAND_LOW  = 1450;
const int PULSE_DEADBAND_HIGH = 1550;

// Angular velocity bounds for reaction wheels
const float RPM_MIN = 200.0f;  // Motor driver struggles to ramp at low speeds
const float RPM_MAX = 6000.0f;

// SMC parameters
const float lambda = 1.0f;    // sliding surface slope
const float Kd     = 0.08f;   // velocity damping
const float Kp     = 0.04f;   // attitude error proportional gain
const float phi    = 0.30f;   // boundary layer thickness

// Adaptive gain parameters
const float K_adapt_init = 0.001f;
const float Gamma        = 0.5f;
const float sigma        = 0.05f;
const float K_min        = 0.001f;
const float K_max        = 0.5f;

// CubeSat parameters
const float Jx       = 0.002167f;  // Inertia matrix must consider centre of mass offsets
const float Jy       = 0.002167f;
const float Jz       = 0.002167f;
const float tau_max  = 0.005f;     // max wheel torque Nm
const float rw_Inertia  = 5.37e-6f;   // wheel moment of inertia kgm^2

// Torque allocation matrix A_pinv (4x3), precomputed from pinv(A)
//
//  A = [-0.5,  -0.5,  0.5,  0.5 ]
//      [ 0.5,  -0.5, -0.5,  0.5 ]
//      [ 0.707, 0.707, 0.707, 0.707 ]
const float A_pinv[4][3] = {
  { -0.5f,  0.5f,  0.353607f },
  { -0.5f, -0.5f,  0.353607f },
  {  0.5f, -0.5f,  0.353607f },
  {  0.5f,  0.5f,  0.353607f },
};

// Quaternion typedef
typedef struct {
  float w, x, y, z;
} Quaternion;

// Initial state
Quaternion q_d     = {0.707f, 0.0f, 0.0f, -0.707f};  // desired attitude
float K_adapt      = K_adapt_init;
float wheel_rpm[4] = {0, 0, 0, 0};

Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);  // Set I2C address for IMU
IntervalTimer controlTimer;  // Teensy specific interrupt
volatile bool runController = false;



// Math helper functions  ([w, x, y, z] quaternion format)

// Quaternion multiplication
void quatMult(const Quaternion* q1, const Quaternion* q2, Quaternion* out) {
  out->w = q1->w*q2->w - q1->x*q2->x - q1->y*q2->y - q1->z*q2->z;
  out->x = q1->w*q2->x + q1->x*q2->w + q1->y*q2->z - q1->z*q2->y;
  out->y = q1->w*q2->y - q1->x*q2->z + q1->y*q2->w + q1->z*q2->x;
  out->z = q1->w*q2->z + q1->x*q2->y - q1->y*q2->x + q1->z*q2->w;
}

// Inverse (conjugate - valid for unit quaternion)
void quatInv(const Quaternion* q, Quaternion* out) {
  out->w =  q->w;
  out->x = -q->x;
  out->y = -q->y;
  out->z = -q->z;
}

// Normalise in place
void quatNorm(Quaternion* q) {
  float magSq = q->w*q->w + q->x*q->x + q->y*q->y + q->z*q->z;
  if (magSq > 1e-12f) {  // 1e-12f prevents divide by zero
    float n = sqrtf(magSq);
    q->w /= n; q->x /= n; q->y /= n; q->z /= n;
  }
}

// Attitude error in degrees
float quatAngleError(const Quaternion* q, const Quaternion* qd) {
  Quaternion qd_inv, q_e;
  quatInv(qd, &qd_inv);
  quatMult(&qd_inv, q, &q_e);
  if (q_e.w < 0) { q_e.w = -q_e.w; }
  float w_val = constrain(q_e.w, -1.0f, 1.0f);
  return 2.0f * acosf(w_val) * (180.0f / PI);
}

// Cross product
void cross3(const float* a, const float* b, float* out) {
  out[0] = a[1]*b[2] - a[2]*b[1];
  out[1] = a[2]*b[0] - a[0]*b[2];
  out[2] = a[0]*b[1] - a[1]*b[0];
}

// Vector norm
float norm3(const float* v) {
  return sqrtf(v[0]*v[0] + v[1]*v[1] + v[2]*v[2]);
}

// Saturation function used in ASMC
float satf(float x, float phi_val) {
  return constrain(x / phi_val, -1.0f, 1.0f);
}




// PWM motor control
void sendPulse(int pin, int pulse_us) {
  if (!motorsEnabled) return;
  digitalWrite(pin, HIGH);
  delayMicroseconds(pulse_us);
  digitalWrite(pin, LOW);
}

// Translate commanded RPM to PWM pulse width
int rpmToPulse(float rpm) {
  if (fabsf(rpm) < 1.0f) return PULSE_NEUTRAL;
  float fraction = constrain((fabsf(rpm) - RPM_MIN) / (RPM_MAX - RPM_MIN), 0.0f, 1.0f);
  if (rpm > 0.0f)
    return (int)(PULSE_DEADBAND_HIGH + fraction * (PULSE_FWD_MAX - PULSE_DEADBAND_HIGH));
  else
    return (int)(PULSE_DEADBAND_LOW  - fraction * (PULSE_DEADBAND_LOW - PULSE_REV_MAX));
}

// Convert torque command to change in RPM, add to current wheel RPM
// tau = rw_Inertia * alpha,  alpha = d(omega)/dt,  omega_wheel in rad/s
// Over one control period dt: delta_omega = tau/rw_Inertia * dt
// Convert to RPM: delta_rpm = delta_omega * 60 / (2*pi)
float torqueToRPM(float tau_cmd, float current_rpm, float dt) {
  float delta_omega = (tau_cmd / rw_Inertia) * dt;
  float delta_rpm   = delta_omega * 60.0f / (2.0f * PI);
  float new_rpm     = constrain(current_rpm + delta_rpm, -RPM_MAX, RPM_MAX);
  // Never coast through zero - avoids repeated cold starts and motor heating
  if (new_rpm >= 0.0f && new_rpm < RPM_MIN)  new_rpm = RPM_MIN;
  if (new_rpm <  0.0f && new_rpm > -RPM_MIN) new_rpm = -RPM_MIN;
  return new_rpm;
}




// ESC ARMING  (skipped for bench demo)
void armESCs() {
  if (!motorsEnabled) {
    Serial.println("DEMO MODE: motors disabled, skipping arming.");
    return;
  }
  Serial.println("Arming ESCs...");
  unsigned long start = millis();
  while (millis() - start < 2000) {
    for (int i = 0; i < 4; i++) sendPulse(ESC_PIN[i], PULSE_NEUTRAL);
    delay(20);
  }
  Serial.println("ESCs armed.");
}


// Control loop ISR
void controlLoop() { runController = true; }


// ASMC  (ported from MATLAB ASMC code)
void runASMC() {
  static float dt = 0.01f;  // 100Hz

  // 1. Read IMU
  imu::Quaternion q_imu = bno.getQuat();
  imu::Vector<3>  gyro  = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);

  Quaternion q = {
    (float)q_imu.w(), (float)q_imu.x(),
    (float)q_imu.y(), (float)q_imu.z()
  };
  float omega[3] = { (float)gyro.x(), (float)gyro.y(), (float)gyro.z() };

  // 2. Quaternion error: q_e = q_d^-1 * q
  Quaternion qd_inv, q_e;
  quatInv(&q_d, &qd_inv);
  quatMult(&qd_inv, &q, &q_e);

  // Hemisphere check - ensures shortest-arc rotation on 4D hypersphere
  if (q_e.w < 0.0f) {
    q_e.w = -q_e.w;
    q_e.x = -q_e.x;
    q_e.y = -q_e.y;
    q_e.z = -q_e.z;
  }

  // Vector part: axis * sin(theta/2)
  float e_q[3]     = { q_e.x, q_e.y, q_e.z };
  float e_omega[3] = { omega[0], omega[1], omega[2] };  // omega_d = 0

  // 3. Sliding surface: s = e_omega + lambda * e_q
  float s[3] = {
    e_omega[0] + lambda * e_q[0],
    e_omega[1] + lambda * e_q[1],
    e_omega[2] + lambda * e_q[2]
  };
  float s_norm = norm3(s);

  // 4. Gyroscopic compensation: tau_gyro = -omega x (J * omega)
  float Jw[3]       = { Jx*omega[0], Jy*omega[1], Jz*omega[2] };
  float omega_x_Jw[3];
  cross3(omega, Jw, omega_x_Jw);
  float tau_gyro[3] = { -omega_x_Jw[0], -omega_x_Jw[1], -omega_x_Jw[2] };

  // 5. PD term: tau_pd = J * (-Kp*e_q - Kd*e_omega)
  float tau_pd[3] = {
    Jx * (-Kp * e_q[0] - Kd * e_omega[0]),
    Jy * (-Kp * e_q[1] - Kd * e_omega[1]),
    Jz * (-Kp * e_q[2] - Kd * e_omega[2])
  };

  // 6. Adaptive switching term: tau_sw = -J * K_adapt * sat(s/phi)
  float tau_sw[3] = {
    -Jx * K_adapt * satf(s[0], phi),
    -Jy * K_adapt * satf(s[1], phi),
    -Jz * K_adapt * satf(s[2], phi)
  };

  // 7. Total torque
  float tau[3] = {
    tau_pd[0] + tau_sw[0] + tau_gyro[0],
    tau_pd[1] + tau_sw[1] + tau_gyro[1],
    tau_pd[2] + tau_sw[2] + tau_gyro[2]
  };

  // 8. Torque allocation: tau_wheels = A_pinv * tau
  float tau_wheels[4];
  for (int i = 0; i < 4; i++) {
    tau_wheels[i] = 0;
    for (int j = 0; j < 3; j++) tau_wheels[i] += A_pinv[i][j] * tau[j];
    tau_wheels[i] = constrain(tau_wheels[i], -tau_max, tau_max);
  }

  // 9. Adaptive gain update
  float K_adapt_dot = Gamma * s_norm - sigma * (K_adapt - K_min);
  K_adapt = constrain(K_adapt + K_adapt_dot * dt, K_min, K_max);

  // 10. PWM output (no-op in demo mode, RPM still tracked for display)
  for (int i = 0; i < 4; i++) {
    float new_rpm = torqueToRPM(tau_wheels[i], wheel_rpm[i], dt);
    bool direction_changed = (new_rpm > 0 && wheel_rpm[i] < 0) ||
                             (new_rpm < 0 && wheel_rpm[i] > 0);
    if (direction_changed) {
      wheel_rpm[i] = 0;
      sendPulse(ESC_PIN[i], PULSE_NEUTRAL);
    } else {
      wheel_rpm[i] = new_rpm;
      sendPulse(ESC_PIN[i], rpmToPulse(wheel_rpm[i]));
    }
  }

  // 11. Serial telemetry - CSV for MATLAB live visualiser
  // Format: T,time_ms,qw,qx,qy,qz,ox,oy,oz,err,s_norm,K,rpm1,rpm2,rpm3,rpm4,tx,ty,tz
  static int telem_count = 0;
  if (++telem_count >= 2) {   // 50Hz telemetry (every 2 control loops)
    telem_count = 0;
    float err_deg = quatAngleError(&q, &q_d);
    Serial.printf("T,%lu,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.2f,%.4f,%.4f,%.0f,%.0f,%.0f,%.0f,%.5f,%.5f,%.5f\n",
      millis(),
      q.w, q.x, q.y, q.z,
      omega[0], omega[1], omega[2],
      err_deg, s_norm, K_adapt,
      wheel_rpm[0], wheel_rpm[1], wheel_rpm[2], wheel_rpm[3],
      tau[0], tau[1], tau[2]
    );
  }
}


void setup() {
  for (int i = 0; i < 4; i++) {
    pinMode(ESC_PIN[i], OUTPUT);
    digitalWrite(ESC_PIN[i], LOW);
  }

  // Send neutral before anything else - prevents motor driver entering fault mode
  unsigned long pre_init = millis();
  while (millis() - pre_init < 500) {
    for (int i = 0; i < 4; i++) sendPulse(ESC_PIN[i], PULSE_NEUTRAL);
    delay(20);
  }

  Serial.begin(115200);
  while (!Serial && millis() < 3000);
  Serial.println(motorsEnabled ? "motors ENABLED" : "DEMO MODE (motors DISABLED)");

  // BNO055 init
  // Wire.setClock must be called before Wire.begin
  Wire.setClock(400000);
  Wire.begin();
  if (!bno.begin()) {
    Serial.println("ERROR: BNO055 not found");
    while (1);
  }

  // setExtCrystalUse must be called before setMode
  bno.setExtCrystalUse(true);

  // IMUPLUS: accelerometer + gyro only, no magnetometer
  // Required because motor magnets interfere with BNO055 magnetometer
  bno.setMode(OPERATION_MODE_IMUPLUS);
  delay(800); 

  // Wait for gyro and accelerometer to calibrate
  // Keep sending neutral pulses so ESCs don't timeout during calibration wait
  Serial.println("Waiting for IMU calibration - hold still...");
  uint8_t sys_cal, g_cal, a_cal, m_cal = 0;
  while (g_cal < 1 || a_cal < 1) {
    bno.getCalibration(&sys_cal, &g_cal, &a_cal, &m_cal);
    Serial.printf("Calibrating: SYS:%d G:%d A:%d\n", sys_cal, g_cal, a_cal);
    for (int i = 0; i < 4; i++) sendPulse(ESC_PIN[i], PULSE_NEUTRAL);
    delay(200);
  }
  Serial.println("IMU calibrated.");

  armESCs();

  // Reset state after arming so K_adapt doesn't carry over any startup noise
  K_adapt = K_adapt_init;
  for (int i = 0; i < 4; i++) wheel_rpm[i] = 0;

  controlTimer.begin(controlLoop, 10000);  // 10000us = 10ms = 100Hz
  Serial.println("Running at 100Hz.");
}



// main
void loop() {
  if (runController) {
    runController = false;
    runASMC();
  }
}

/*
// Code used to identify and map the physical reaction wheels to the torque allocation matrix
// Not required anymore
void loop() {
  static bool test_done = false;
  if (test_done) {
    for (int i = 0; i < 4; i++) sendPulse(ESC_PIN[i], PULSE_NEUTRAL);
    return;
  }

  for (int motor = 0; motor < 4; motor++) {

    // Hold neutral on ALL motors for 3s (STOP_DURATION + margin)
    Serial.printf("\nPausing before motor %d...\n", motor);
    unsigned long t = millis();
    while (millis() - t < 3000) {
      for (int i = 0; i < 4; i++) sendPulse(ESC_PIN[i], PULSE_NEUTRAL);
      delay(20);
    }

    // Arm motor (500ms of neutral pulses)
    Serial.printf("Arming motor %d...\n", motor);
    t = millis();
    while (millis() - t < 1000) {
      for (int i = 0; i < 4; i++) sendPulse(ESC_PIN[i], PULSE_NEUTRAL);
      delay(20);
    }

    // Spin motor, others neutral
    Serial.printf(">>> Spinning ESC_PIN[%d] (pin %d) - watch which wheel moves\n",
      motor, ESC_PIN[motor]);
    t = millis();
    while (millis() - t < 5000) {
      for (int i = 0; i < 4; i++) {
        sendPulse(ESC_PIN[i], i == motor ? 1620 : PULSE_NEUTRAL);
      }
      delay(20);
    }

    // Stop motor
    Serial.printf("Stopping motor %d.\n", motor);
    t = millis();
    while (millis() - t < 500) {
      for (int i = 0; i < 4; i++) sendPulse(ESC_PIN[i], PULSE_NEUTRAL);
      delay(20);
    }
  }

  Serial.println("\nAll motors tested. Fill in your mapping table.");
  test_done = true;
}
*/

/*
// SD Card setup and logging, uncomment to use instead of live demo
void initSD() {
  if (!SD.begin(chipSelect)) {
    Serial.println("SD initialisation failed");
    return;
  }
  // Create a header for your CSV file
  dataFile = SD.open("datalog.csv", FILE_WRITE);
  if (dataFile) {
    dataFile.println("time_ms,err_deg,s_norm,K_adapt,rpm0,rpm1,rpm2,rpm3");
    dataFile.close();
    Serial.println("SD Initialised");
  }
}

void logTelemetry(float err, float s, float K, float* rpm) {
  // Open the file in append mode
  dataFile = SD.open("datalog.csv", FILE_WRITE);
  
  if (dataFile) {
    // Print timestamp first (crucial for documentation/plotting)
    dataFile.print(millis());
    dataFile.print(",");
    
    // Print the float values
    dataFile.print(err, 2);
    dataFile.print(",");
    dataFile.print(s, 4);
    dataFile.print(",");
    dataFile.print(K, 4);
    dataFile.print(",");
    
    // Print the RPM array
    dataFile.print(rpm[0], 0); dataFile.print(",");
    dataFile.print(rpm[1], 0); dataFile.print(",");
    dataFile.print(rpm[2], 0); dataFile.print(",");
    dataFile.println(rpm[3], 0); // println ends the CSV row
    
    dataFile.close(); // Close to ensure data is flushed to the card
  }
}
*/