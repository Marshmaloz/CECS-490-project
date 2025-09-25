# nRF24L01 RC Link (2-Channel)

A minimal **2-byte RC link** using two Arduinos, a pair of **nRF24L01(+) radios**, and two hobby servos.  

- **Transmitter**: Reads a 2-axis joystick (`A0`, `A1`), applies dead-zone + calibration, sends 2 bytes (`ch1`, `ch2`).  
- **Receiver**: Converts bytes into servo signals (`1000–2000 µs`) for pins **D2** and **D3**.

---

## 📂 Project Structure

RC_Module_Link/
 ├── README.md              # Documentation (setup, wiring, usage)
 ├── Transmitter_code.ino   # Reads joystick input & sends 2-byte data via nRF24L01
 └── Receiver_Code.ino      # Receives 2-byte data & controls two servos
---

## 🔧 Hardware

### Components
- Arduino Uno/Nano (or compatible boards)  
- nRF24L01(+) modules (with capacitor across VCC/GND)  
- 2-axis analog joystick  
- 2× RC servos  

### Wiring

#### nRF24L01 ↔ Arduino Uno/Nano
| nRF24L01 | Arduino |
|----------|---------|
| VCC      | 3.3 V   |
| GND      | GND     |
| CE       | D9      |
| CSN      | D10     |
| SCK      | D13     |
| MOSI     | D11     |
| MISO     | D12     |

#### Transmitter
- Joystick X → `A0`  
- Joystick Y → `A1`  
- Serial debug → `9600 baud`

#### Receiver
- Servo 1 → `D2`  
- Servo 2 → `D3`  
- Serial debug → `9600 baud`
---

## 📦 Software Setup

Install these Arduino libraries:
- [RF24](https://github.com/nRF24/RF24) by TMRh20  
- nRF24L01 (optional, sometimes bundled)  
- Servo (built-in)  
- SPI (built-in)  

Both sketches must use the same radio settings:
```cpp
const uint64_t pipe = 0xE8E8F0F0E1LL;
radio.setAutoAck(false);
radio.setDataRate(RF24_250KBPS);

