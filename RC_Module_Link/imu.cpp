#include <Arduino.h>
#include <FaBo9Axis_MPU9250.h>

FaBo9Axis mpu;  // FaBo 9Axis MPU9250

// ===== globals (match your originals) =====
unsigned long lastMicros = 0;
float roll_angle = 0, pitch_angle = 0, yaw_angle = 0;
float roll_offset = 0.0f, pitch_offset = 0.0f;
float gyroBiasX = 0.0f, gyroBiasY = 0.0f, gyroBiasZ = 0.0f;

float roll_Input  = 0.0f;
float pitch_Input = 0.0f;
float yaw_Input   = 0.0f;

float yaw_ZeroOffset = 0.0f; // The initial magnetic heading is stored to force start to 0 degrees
bool yawZeroSet = false;          // first-run latch
const float MAG_DECLINATION = 0.0f;

const float PERMANENT_MAG_BIAS_X = 2.2476f;
const float PERMANENT_MAG_BIAS_Y = 1.8040f;
const float PERMANENT_MAG_BIAS_Z = -106.7940f;
float magBiasX = 0.0f, magBiasY = 0.0f, magBiasZ = 0.0f; // Runtime magnetic bias variables

static constexpr float RAD2DEG = 180.0f / PI;

// ===== init/calibrate/read =====
void initIMU() {
  Wire.begin();
  mpu.begin();

  // Match your old ranges: accel ±8g, gyro ±500 dps
  mpu.configMPU9250(MPU9250_GFS_500, MPU9250_AFS_8G);
  // Optional: configure mag to 16-bit, 100 Hz
  mpu.configAK8963(AK8963_MODE_C100HZ, AK8963_BIT_16);

}
void zeroYaw_init(){
   float mx, my, mz;
  mpu.readMagnetXYZ(&mx, &my, &mz);
  mx -= magBiasX;
  my -= magBiasY;
  mz -= magBiasZ;

  float roll_rad  = roll_angle / RAD2DEG;
  float pitch_rad = pitch_angle / RAD2DEG;
  float cosRoll = cosf(roll_rad);
  float sinRoll = sinf(roll_rad);
  float cosPitch = cosf(pitch_rad);
  float sinPitch = sinf(pitch_rad);

  // same tilt-compensation math you use later
  float magX_comp = mx * cosPitch + my * sinRoll * sinPitch + mz * cosRoll * sinPitch;
  float magY_comp = my * cosRoll - mz * sinRoll;

  float magYaw = atan2f(magY_comp, magX_comp) * RAD2DEG;
  if (magYaw < 0) magYaw += 360.0f;

  yaw_angle      = magYaw;        // seed yaw filter with current heading
  yaw_ZeroOffset = magYaw;        // make this heading = 0°
}

void calibrateIMU() {
  const int N = 1000;
  float sumAx=0, sumAy=0, sumAz=0;
  float sumGx=0, sumGy=0, sumGz=0;

  for (int i = 0; i < N; i++) {
    float ax, ay, az, gx, gy, gz;
    float mx, my, mz;

    mpu.readAccelXYZ(&ax, &ay, &az);   // accel in g (scale cancels in ratios below)
    mpu.readGyroXYZ(&gx, &gy, &gz);    // gyro in deg/s
    mpu.readMagnetXYZ(&mx, &my, &mz);  // not used for calibration here

    sumAx += ay; sumAy += ax; sumAz += az;
    sumGx += gy; sumGy += gx; sumGz += gz;
    delay(1);
  }

  const float axm = sumAx / N, aym = sumAy / N, azm = sumAz / N;

  // Tilt from averaged accel (units cancel, so g vs m/s^2 doesn't matter)
  float accRoll  = atan2f(aym, azm) * RAD2DEG;
  float accPitch = atan2f(-axm, sqrtf(aym*aym + azm*azm)) * RAD2DEG;

  roll_offset  = accRoll;
  pitch_offset = accPitch;

  // Gyro bias in deg/s (FaBo returns deg/s already; do NOT multiply by RAD2DEG)
  gyroBiasX = (sumGx / N);
  gyroBiasY = (sumGy / N);
  gyroBiasZ = (sumGz / N);

  magBiasX = PERMANENT_MAG_BIAS_X; 
  magBiasY = PERMANENT_MAG_BIAS_Y;
  magBiasZ = PERMANENT_MAG_BIAS_Z;

  roll_angle  = 0.0f;
  pitch_angle = 0.0f;
  lastMicros  = micros();
}

void read_IMU_and_Calculate_Angles() {
  float ax, ay, az, gx, gy, gz, mx, my, mz;
  mpu.readAccelXYZ(&ax, &ay, &az);   // accel in g
  mpu.readGyroXYZ(&gx, &gy, &gz);    // gyro in deg/s
  mpu.readMagnetXYZ(&mx, &my, &mz); 

  unsigned long now = micros();
  float dt = (now - lastMicros) * 1.0e-6f;
  lastMicros = now;
  if (dt < 1.0e-4f) dt = 1.0e-4f;
  if (dt > 0.05f)   dt = 0.05f;

  // Accel-only angles (deg) with offsets removed
  float accRoll  = atan2f(ax, az) * RAD2DEG - roll_offset;
  float accPitch = atan2f(-ay, sqrtf(ax*ax + az*az)) * RAD2DEG - pitch_offset;

  // Gyro (deg/s) with bias removed
  float gyroX_deg_s = gy - gyroBiasX;
  float gyroY_deg_s = gx - gyroBiasY;
  float gyroZ_deg_s = gz - gyroBiasZ;

  // Complementary filter
  const float alpha = 0.98f;
  roll_angle  = alpha * (roll_angle  + gyroX_deg_s * dt) + (1.0f - alpha) * accRoll;
  pitch_angle = alpha * (pitch_angle + gyroY_deg_s * dt) + (1.0f - alpha) * accPitch;
  //magnometer bias from calibration from a different sketch
  mx -= magBiasX;
  my -= magBiasY;
  mz -= magBiasZ;

  // 2. Tilt Compensation (Use Roll and Pitch in Radians)
  float roll_rad = roll_angle / RAD2DEG;
  float pitch_rad = pitch_angle / RAD2DEG;

  float cosRoll = cosf(roll_rad);
  float sinRoll = sinf(roll_rad);
  float cosPitch = cosf(pitch_rad);
  float sinPitch = sinf(pitch_rad);

  // Rotate magnetic vector to the horizontal plane
  // Hx = mx * cosPitch + my * sinRoll * sinPitch + mz * cosRoll * sinPitch
  // Hy = my * cosRoll - mz * sinRoll
  float magX_comp = mx * cosPitch + my * sinRoll * sinPitch + mz * cosRoll * sinPitch;
  float magY_comp = my * cosRoll - mz * sinRoll;

  // 3. Calculate Magnetic Yaw (Absolute reference, 0 to 360 degrees)
  float magYaw = atan2f(magY_comp, magX_comp) * RAD2DEG;
  if (magYaw < 0) magYaw += 360.0f; // Convert -180/180 range to 0/360

  // 4. Yaw Complementary Filter
  gyroZ_deg_s = gz - gyroBiasZ; // Gyro rate (Yaw)
  const float alpha_Y = 0.99f; // Higher weight on gyro for yaw
  
  // Integrate gyro rate
  yaw_angle += gyroZ_deg_s * dt;

  // Handle angle wrap-around for the integrated gyro yaw
  if (yaw_angle > 360.0f) yaw_angle -= 360.0f;
  if (yaw_angle < 0.0f) yaw_angle += 360.0f;

  // Calculate the shortest path error between Magnetic Yaw and Gyro Yaw
  float yaw_error_mag = magYaw - yaw_angle;
  if (yaw_error_mag > 180.0f) yaw_error_mag -= 360.0f;
  if (yaw_error_mag < -180.0f) yaw_error_mag += 360.0f;

  // Apply magnetic correction to the fused angle
  yaw_angle += yaw_error_mag * (1.0f - alpha_Y);
  
  // --- Yaw Zero-Reference and Output for PID ---
  
  // 5. Apply Zero Offset to make current heading 0 degrees at start
  float relative_yaw = yaw_angle - yaw_ZeroOffset;

  // 6. Normalize relative yaw to +/- 180 degrees for PID
  if (relative_yaw > 180.0f) relative_yaw -= 360.0f;
  if (relative_yaw < -180.0f) relative_yaw += 360.0f;

  


  // Deadbands
  if (abs(roll_angle)  <= 0.8f) roll_Input  = 0.0f; else roll_Input  = roll_angle;
  if (abs(pitch_angle) <= 0.8f) pitch_Input = 0.0f; else pitch_Input = pitch_angle;
  if (abs(relative_yaw) <= 2.0f) yaw_Input = 0.0f; else yaw_Input = relative_yaw;

}
