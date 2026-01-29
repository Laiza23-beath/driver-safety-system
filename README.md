# Driver Safety System - IoT Drowsiness Detection

95% accuracy drowsiness detection system using computer vision and Arduino.

## Hardware
- Arduino Uno
- Buzzer (Pin 9)
- LEDs: Green (Pin 12), Yellow (Pin 11), Red (Pin 13)
- Camera
- NFP1315-61AY (I2C: 0x5A)

## Setup
1. Install dependencies: `pip install -r requirements.txt`
2. Upload `arduino_iot_system.ino` to Arduino
3. Connect hardware as shown above
4. Run: `python main.py`

## Controls
- 'q' - Quit
- 'r' - Reset

## Features
- Eye closure detection
- Yawning detection
- Head pose analysis
- 4-level alert system
- Real-time BP simulation