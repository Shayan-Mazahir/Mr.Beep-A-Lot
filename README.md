# Mr. Beep-A-Lot

**Your Friendly Arduino Companion**

A multi-mode autonomous robot with personality that can navigate, scan environments, and follow humans with delightful LCD messages.

**Computer Engineering Final Project**

---

## Project Image

![Mr. Beep-A-Lot](images/mr_beep_a_lot.png)
*Mr. Beep-A-Lot with his signature dual ultrasonic "eyes" and elevated sensor head*

---

## Features

- **Obstacle Avoidance Mode** - Autonomous navigation with left/right scanning
- **Radar Mode** - 180° environment scanning with real-time visualization
- **Human Following Mode** - "POOKIE HOOMAN!!" - Follows detected person
- **IR Remote Control** - Wireless mode switching
- **LCD Personality** - Expressive messages that bring the robot to life
- **Serial Debugging** - Complete operational telemetry

---

## Hardware

### Core Components
- **Arduino UNO** - Main microcontroller
- **2× HC-SR04 Ultrasonic Sensors** - Dual sensor setup for obstacle detection and radar scanning
- **2× Servo Motors** - Head movement (obstacle avoidance) + radar sweep
- **L298N Motor Driver** - Dual DC motor control
- **16×2 I2C LCD Display** - Personality expressions and status messages
- **IR Receiver** - Wireless remote control
- **2× DC Motors** - Wheel drive
- **2× 9V Batteries** - Separate power for logic and motors

### Complete Parts List
See [presentation/Comp_Eng_ISU.pdf](presentation/Comp_Eng_ISU.pdf) for detailed bill of materials with costs.

---

## Software Stack

### Arduino (Embedded Code)
- **Language:** C/C++
- **IDE:** Arduino IDE 1.8+
- **Board:** Arduino UNO

### Processing (Visualization)
- **Language:** Java (Processing)
- **Version:** Processing 3.x or higher
- **Purpose:** Real-time radar display

### Communication Protocol
- **Serial:** 9600 baud
- **Format:** CSV (angle,distance)
- **Example:** `45,120` = 45° angle, 120cm distance

---

## Dependencies

### Arduino Libraries

| Library | Version | Purpose | Link |
|---------|---------|---------|------|
| **IRremote** | Latest | IR remote control | [GitHub](https://github.com/Arduino-IRremote/Arduino-IRremote) |
| **NewPing** | Latest | Ultrasonic sensor control | [Bitbucket](https://bitbucket.org/teckel12/arduino-new-ping/wiki/Home) |
| **Servo** | Built-in | Servo motor control | [GitHub](https://github.com/arduino-libraries/Servo) |
| **Wire** | Built-in | I2C communication | [Arduino Docs](https://docs.arduino.cc/language-reference/en/functions/communication/wire/) |
| **LiquidCrystal_I2C** | Latest | LCD display control | [GitHub](https://github.com/johnrickman/LiquidCrystal_I2C) |

### Processing Libraries

| Library | Version | Purpose | Link |
|---------|---------|---------|------|
| **Serial** | Built-in | Serial communication | Built into Processing |
| **SerialRecord** | 0.4.1 (optional) | Enhanced serial handling | Oliver Steele |

---

## Getting Started

### 1. Hardware Assembly

Refer to either:
-  [Tinkercad schematic](schematics/tinkercad_schematic.png)
**OR**
- [SVG XML schematic](schematics/schemetic.png) 

for complete wiring diagram.

**Key Connections:**
- Ultrasonic sensors → Pins 8,9 (main) and 12,13 (radar)
- Servos → Pins 6 (radar) and 11 (head)
- Motor driver → Pins 2,3,4,5
- IR receiver → Pin 10
- LCD → I2C (SDA/SCL)

### 2. Arduino Setup

```bash
# Install Arduino IDE
# Download from: https://www.arduino.cc/en/software

# Install required libraries via Arduino Library Manager:
# - IRremote
# - NewPing
# - LiquidCrystal_I2C

# Upload the code
1. Open beep_alot/beep_alot.ino
2. Select Board: Arduino UNO
3. Select Port: (your Arduino port)
4. Click Upload
```

### 3. Processing Setup

```bash
# Install Processing
# Download from: https://processing.org/download

# Run the visualization
1. Open radar/radar.pde
2. Update serial port if needed (Serial.list()[0])
3. Click Run
```

### 4. IR Remote Configuration

**Button Mapping:**
- **Button 69** → Obstacle Avoidance Mode
- **Button 70** → Radar Mode
- **Button 71** → Human Following Mode
- **Other buttons** → Idle Mode (with easter egg message!)

*Note: Button codes may vary by remote. Check Serial Monitor for your remote's codes.*

---

## Operation Modes

### Mode 1: Obstacle Avoidance
**Activation:** IR Button 69

Autonomous navigation that avoids obstacles:
1. Checks distance ahead with ultrasonic sensor
2. If obstacle detected (<15cm):
   - Stops and backs up
   - Scans left and right with servo
   - Turns toward more open space
3. Continues forward if path clear

**LCD Display:** `"Obstacle" / "Avoidance"`

---

### Mode 2: Radar Scanning
**Activation:** IR Button 70

Environment scanning with real-time visualization:
- Servo sweeps 0° to 90° continuously
- Measures distance at each angle
- Sends data to Processing via Serial
- Creates military-style radar display

**LCD Display:** `"Radar" / "Mode"`

**Serial Output Format:**
```
45,120
46,118
47,115
...
```
(angle in degrees, distance in cm)

---

### Mode 3: Human Following
**Activation:** IR Button 71

Follows detected person with personality:

| Distance | LCD Message | Action |
|----------|-------------|--------|
| < 15cm | `"Hooman too.." / "close"` | Stop (personal space!) |
| 15-100cm | `"POOKIE HOOMAN!!"` | Follow |
| > 100cm | `"Hooman? Where?"` | Stop (lost sight) |

---

## Personality Features

Mr. Beep-A-Lot expresses himself through LCD messages:

- **Startup:** `"MA LOOAARD!" / "MR. BEEP-A-LOT"`
- **Idle Mode:** `"I'M A MINOR!" / "(Idle Mode)"` (Easter egg for undefined buttons)
- **Following:** `"POOKIE HOOMAN!!"` (when tracking you)
- **Too Close:** `"Hooman too.. close"` (respect the bubble!)
- **Lost You:** `"Hooman? Where?"` (searching)

These messages give the robot character and make interaction delightful.

---

## Project Structure

```
Mr. BEEP-A-LOT/
├── beep_alot/
│   ├── beep_alot.ino          # Main Arduino code
├── radar/
│   └── radar.pde    # Radar display visualization
├── presentation/
│   └── Comp Eng ISU.pdf          # Full project presentation
├── images/
│   └── beep_alot.jpg         # Robot photo
├── schematics/
│   └── tinkercad_schematic.png   # Wiring diagram
└── README.md                      # This file
```

---

## Troubleshooting

### Arduino Not Responding
- Check USB connection
- Verify correct port selected in Arduino IDE
- Press reset button on Arduino

### Motors Not Moving
- Check battery voltage (both 9V batteries charged)
- Verify L298N connections
- Check motor power supply separate from Arduino logic

### LCD Shows Garbage Characters
- Check I2C address (default 0x27, may be 0x3F)
- Verify SDA/SDL connections
- Adjust contrast potentiometer on LCD

### IR Remote Not Working
- Check button codes in Serial Monitor
- Verify IR receiver wiring
- Ensure clear line of sight to receiver

### Processing Can't Find Arduino
- Print `Serial.list()` in Processing setup
- Change `Serial.list()[0]` to correct index
- Close Arduino Serial Monitor (can't share port)

### Radar Visualization Not Updating
- Ensure Arduino is in Radar Mode (Button 70)
- Check baud rate matches (9600)
- Verify Serial cable connected

---

## Educational Value

### Concepts Demonstrated

**Embedded Systems:**
- Multi-mode state machine design
- Sensor fusion (dual ultrasonic sensors)
- Motor control (PWM, H-bridge)
- Serial communication protocols

**Hardware Integration:**
- I2C communication (LCD)
- IR signal decoding
- Servo position control
- Power management (separate logic/motor supplies)

**Software Engineering:**
- Modular function design
- Real-time data processing
- Inter-process communication (Arduino <----> Processing)
- Error handling and validation


---

## Project Achievements

- Computer Engineering Final Project
- Timeline: 4 weeks (Nov 18 - Dec 16, 2024)
- Complexity: Multi-system integration (hardware, embedded, visualization)
- Innovation: Personality-driven human-robot interaction

---

## License

This project is open source and available for educational purposes.

---

## Acknowledgments

- **Libraries:** Thanks to all library maintainers for excellent Arduino ecosystem support

---

## Contact

For questions about this project:
- **Discord:** mr.raza

---


*"POOKIE HOOMAN!!" - Mr. Beep-A-Lot*