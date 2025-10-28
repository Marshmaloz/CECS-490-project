#E01-ML01DP5   RC Link (4-Channel Quadcopter) 🚁

This project implements a **4-channel RC link** with **Telemetry** and **PID stabilization** for a quadcopter, using two Arduinos and a pair of **E01-ML01DP5 nRF24L01(+) radios**.

The original 2-channel concept has been expanded to a full flight control system:
* **Transmitter**: Reads a 4-axis input (two joysticks), performs dead-zone/calibration, sends 4 control bytes (`ch1` to `ch4`), and displays **Roll, Pitch, and Yaw telemetry** received from the quadcopter.
* **Receiver (Quadcopter)**: Manages radio communication, reads sensor data from an **MPU9250 IMU**, calculates flight corrections using **PID controllers** (Roll, Pitch, Yaw), drives four ESCs/motors, and sends the current angle telemetry back to the Transmitter.
## 📂 Project Structure
```
RC_Module_Link/
├── README.md
├── Receiver_Code.ino
├── Transmitter_code.ino
├── imu.cpp
├── imu.h
├── joystick.cpp
└── joystick.h
```

## 🔧 Hardware & Wiring
*(Wiring details remain consistent with the 4-Channel quadcopter setup.)*

### Components
* Arduino Uno/Nano (or compatible boards) - **2 required**
* **E01-ML01DP5** nRF24L01(+) modules (decoupling capacitor recommended)
* **2×** 2-axis analog joysticks (4-axis input device)
* **4×** RC ESCs/motors (for quadcopter)
* **FaBo9Axis_MPU9250** (or similar 9-axis IMU)

### Wiring Summary

| Connection | Transmitter | Receiver (Quadcopter) |
| :--------- | :---------- | :-------------------- |
| **Radio SPI** | D9, D10, D11, D12, D13 | D9, D10, D11, D12, D13 |
| **Control** | A0, A1, A2, A4 (Joysticks) | N/A |
| **Output** | N/A | D2, D3, D4, D5 (ESCs/Servos) |
| **IMU** | | I²C (SDA/SCL) |

***

## ⚙️ In-Depth Flight Control Logic (Receiver)

The stabilization loop consists of three main stages: Attitude Estimation, PID Control, and Motor Mixing.

### 1. Attitude Estimation (`imu` Library)

The `imu` component is responsible for reading the raw $9$ degrees of freedom (DOF) from the MPU9250 (3-axis Accel, 3-axis Gyro, 3-axis Mag) and fusing them into stable **Roll, Pitch, and Yaw angles** for the PID controllers.

#### A. Calibration (`calibrateIMU()`)
The system performs a one-time calibration at startup to establish baseline errors:
1.  **Gyro Bias**: Averages raw gyroscope readings while the copter is stationary to find the constant drift in $^\circ/\text{s}$ (stored in `gyroBiasX`, `gyroBiasY`, `gyroBiasZ`).
2.  **Accelerometer Offset**: Calculates the static **Roll** and **Pitch** angle offsets based on the average orientation due to gravity. This ensures the zero-angle is relative to the board's mounting angle.
3.  **Magnetometer Bias**: Permanent biases (`PERMANENT_MAG_BIAS_X`, etc.) must be pre-calibrated from a separate process and loaded to correct the magnetic field readings.

#### B. The Complementary Filter (`read_IMU_and_Calculate_Angles()`)

This filter fuses the fast, short-term data (Gyro) with the slow, stable reference data (Accel/Mag) to avoid drift while rejecting noise. 

| Sensor Data | Role | Frequency/Stability |
| :--- | :--- | :--- |
| **Gyro** (Rates) | Measures *change* in angle. | Fast, but drifts over time. |
| **Accel** (Tilt) | Measures *absolute* Roll/Pitch angle relative to gravity. | Slow, noisy due to vibration/acceleration. |
| **Mag** (Heading) | Measures *absolute* Yaw angle relative to magnetic North. | Slow, noisy due to motors/wires. |

1.  **Roll & Pitch Fusion**:
    $$
    \text{Angle}_{\text{fused}} = \alpha \cdot (\text{Angle}_{\text{prev}} + \text{GyroRate} \cdot \Delta t) + (1-\alpha) \cdot \text{AccelAngle}
    $$
    Where $\alpha = 0.98$ gives a 98% weight to the gyroscope (for quick response) and 2% to the accelerometer (to correct long-term drift). The result is the stable `roll_angle` and `pitch_angle`.

2.  **Yaw (Heading) Fusion**:
    * **Tilt Compensation**: Raw magnetometer readings (`mx`, `my`, `mz`) are mathematically rotated into the horizontal plane using the calculated `roll_angle` and `pitch_angle`. This removes magnetic distortion caused by the copter being tilted.
    * **Magnetic Yaw**: The corrected vector is used to calculate the absolute magnetic heading (`magYaw`) via $\text{atan2f}$.
    * **Yaw Integration**: The yaw gyro rate is integrated into `yaw_angle`.
    * **Final Fusion**: A complementary filter with $\alpha=0.99$ fuses the integrated Gyro Yaw with the absolute Magnetic Yaw to create the drift-corrected `yaw_angle`.
    * **Yaw Reference**: Finally, the system subtracts the initial `yaw_ZeroOffset` (taken at boot) to make the current heading 0 degrees, and the result is clipped to $\pm 180^\circ$ for the PID input (`yaw_Input`).

The final, stable angle outputs are provided to the PID controllers via the **`roll_Input`**, **`pitch_Input`**, and **`yaw_Input`** global variables.

### 2. Proportional-Integral-Derivative (PID) Control

Three independent PID loops (**Roll**, **Pitch**, and **Yaw**) calculate the instantaneous motor thrust correction needed to drive the $\text{Input}$ (actual angle) toward the $\text{Setpoint}$ (desired angle/rate).

| Axis | PID Input (`*Input`) | PID Setpoint (`*Set`) | Function | Correction Output (`*Out`) |
| :--- | :--- | :--- | :--- | :--- |
| **Roll** | Current Roll Angle ($^\circ$) | Desired Roll Angle ($^\circ$) | Keep level / tilt left-right. | `rollOut` ($\mu\text{s}$) |
| **Pitch** | Current Pitch Angle ($^\circ$) | Desired Pitch Angle ($^\circ$) | Keep level / tilt forward-back. | `pitchOut` ($\mu\text{s}$) |
| **Yaw** | Current Yaw Rate ($^\circ/\text{s}$) | Desired Yaw Rate ($^\circ/\text{s}$) | Control rotation speed. | `yawOut` ($\mu\text{s}$) |

The PID outputs (`*Out`) are constrained by `MIX_LIMIT = 350\ \mu\text{s}$ to ensure the corrections don't exceed the safe bounds of the motor signals.

### 3. Motor Mixing Algorithm

The PID corrections are mathematically combined with the main Throttle signal to create the final thrust commands for the four motors.

The **X-Configuration** mixer is used to achieve the required differential thrust for each movement:

$$
\begin{array}{rcl}
M_1\text{ (Front-Left)} & = & \text{Throttle} + \text{PitchOut} + \text{RollOut} + \text{YawOut} \\
M_2\text{ (Front-Right)} & = & \text{Throttle} + \text{PitchOut} - \text{RollOut} - \text{YawOut} \\
M_3\text{ (Rear-Right)} & = & \text{Throttle} - \text{PitchOut} - \text{RollOut} + \text{YawOut} \\
M_4\text{ (Rear-Left)} & = & \text{Throttle} - \text{PitchOut} + \text{RollOut} - \text{YawOut}
\end{array}
$$

The final motor signals are constrained to the valid ESC range of **$1000\ \mu\text{s}$ (min) to $2000\ \mu\text{s}$ (max)** and sent to the servo pins (D2-D5).
## 📦 Software Setup

Install the following libraries: **RF24**, **Servo**, **SPI**, **PID_v1**, and **FaBo9Axis_MPU9250**.

### Radio Configuration

Both Transmitter and Receiver must share the same configuration. **AutoAck** and **AckPayload** are enabled to allow the Receiver to send **Telemetry** data back to the Transmitter immediately after receiving a command.

```cpp
const uint64_t my_radio_pipe = 0xE8E8F0F0E1LL;
radio.setDataRate(RF24_250KBPS); // Max Range
radio.setChannel(100);
radio.setAutoAck(true);
radio.enableAckPayload(); // Enables Telemetry (RX -> TX)


