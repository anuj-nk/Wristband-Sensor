# ThrowSense  
*A wearable system that measures throw speed and estimates throw distance, visualized through a physical stepper-motor gauge and an OLED display.*

---

## Project Overview

ThrowSense is a two-device interactive system consisting of a wearable sensing unit and a handheld or desk-mounted display unit. The wearable tracks the motion of a throw using inertial sensors and computes an estimated throw speed and distance. The display device presents this information using a physical needle gauge driven by a stepper motor, along with an OLED screen for numeric readouts and mode selection.

The system is designed to provide intuitive, glanceable feedback for different throwing contexts (e.g., football, baseball), emphasizing physical interaction and real-world sensing over screen-only visualization.

### General Physical Sketch
*(Insert image here: overall system showing wearable sensor + display device)*

---

## Sensing Device — Wearable Motion Tracker

### Device Overview

The sensing device is a wrist- or forearm-mounted wearable that captures high-frequency motion data during a throwing action. It detects when a throw occurs, estimates release velocity, and performs lightweight DSP and physics-based calculations before transmitting results to the display device.

### Hardware Components

- **Processor:** ESP32-C3
- **IMU Sensor:** MPU-6050  
  - 3-axis accelerometer  
  - 3-axis gyroscope  
- **Power:**  
  - 3.7V LiPo battery  

### Part Numbers

- ESP32-C3  
- MPU-6050

### How It Works

1. The MPU-6050 continuously samples acceleration and angular velocity.
2. Motion data is filtered and analyzed in real time to detect a throwing event.
3. Peak angular velocity and acceleration are extracted during the throw.
4. Release speed is estimated from IMU data.
5. A simplified physics-based model estimates throw distance based on:
   - selected object type (e.g., football, baseball)
   - assumed release angle
   - object-specific parameters (mass, drag heuristic).
6. Final computed values are transmitted to the display device.

### Sensor Device Sketch
*(Insert image here: detailed wearable device sketch with labeled components)*

---

## Display Device — Physical Gauge + OLED Interface

### Device Overview

The display device is a handheld or desk-mounted physical interface that presents throw data in both analog and digital forms. A stepper-motor-driven needle provides an immediate visual indication of throw speed, while an OLED display shows numeric values and the selected throw mode. A button allows the user to cycle through object types before throwing.

### Hardware Components

- **Processor:** ESP32-C3
- **Stepper Motor:** 28BYJ-48
- **Stepper Driver:** ULN2003
- **Display:** 0.96" I2C OLED (SSD1306)
- **User Input:** Tactile push button
- **Indicator:** Status LED
- **Power:** Wall power or battery (TBD)

### Part Numbers

- ESP32-C3  
- 28BYJ-48 Stepper Motor  
- ULN2003 Driver Board  
- SSD1306 OLED Display  

### How It Works

1. The display device receives processed throw data from the sensing device.
2. The stepper motor rotates the needle to a mapped angle corresponding to throw speed.
3. The OLED displays:
   - selected throw mode (e.g., football, baseball)
   - measured throw speed
   - estimated throw distance
4. The user presses a button to cycle through throw modes.
5. Mode changes are sent back to the sensing device to update calculation parameters.
6. The status LED indicates system state:
   - **Ready:** system prepared to detect a throw  
   - **Throwing:** motion detected  
   - **Processing:** calculations in progress  

### Display Device Sketch
*(Insert image here: detailed display device sketch showing gauge, OLED, button, LED, and internal components)*

---

## Device Communication & System Architecture

### Device Communication

- **Communication medium:** To be determined (wired or wireless)
- **Topology:** Sensing device → Display device
- The sensing device performs all DSP and physics calculations.
- The display device focuses on visualization and user interaction.

### System Diagram — High-Level Communication
*(Insert image here: block diagram showing IMU → sensing ESP32-C3 → display ESP32-C3 → motor/OLED/LED)*

### System Diagram — Data Flow
*(Insert image here: detailed flowchart of signal processing and control)*

**Data Flow Explanation:**

1. Raw IMU data is captured on the sensing device.
2. DSP techniques (filtering, thresholding, peak detection) identify a throw.
3. A physics-based model estimates speed and distance.
4. Final metrics are transmitted to the display device.
5. The display device maps speed to motor position and updates the OLED and LED indicators.

## Datasheets

All datasheets are included in the `/datasheets` directory, including:

- ESP32-C3 datasheet  
- MPU-6050 datasheet  
- 28BYJ-48 stepper motor datasheet  
- ULN2003 driver datasheet  
- SSD1306 OLED datasheet  
