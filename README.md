# NCAIR Autonomous Seeding Robot - "Àgbẹ̀"

## Overview

This repository contains the documentation and codebase for **Àgbẹ̀**, an autonomous seeding robot developed during a Robotics Internship with the National Center for Artificial Intelligence and Robotics (NCAIR) in Abuja, Nigeria.

### Project Philosophy

- **Low Ground Pressure Design**: Minimizes soil compaction to preserve agricultural land quality
- **Swarm Approach**: Enables multiple units to work collaboratively for increased efficiency
- **Low Cost Solution**: Designed to be accessible and scalable for Nigerian agricultural contexts

---

# Repository Structure

```
NCAIR-AutonomousSeeding/
├── docs/
│   ├── hardware/
│   │   └── cad/                    # Fusion 360 CAD files
│   └── images/
│       ├── renders/                # 3D renders of the robot
│       ├── testing/                # Field testing images
│       ├── wiring/                 # Wiring diagrams
│       └── system_diagram/         # System architecture diagrams
├── architecture.md                 # Detailed description of the system architecture
├── evaluation.md                   # Evaluation of the current system
├── software/
│   ├── firmware/
│   │   ├── AutonomousSeeding.ino   # Motor control & sensor integration
│   │   └── wifi_comms.ino          # WiFi communication module
│   └── webapp/                     # Web application for robot control
│   │   ├── index.html  
│   │   ├── kalman.js
│   │   ├── main.js
│   │   └── styles.css 
├── architecture_evaluation.pdf     # Detailed architecture & product evaluation
├── assembly_guide.pdf              # Step-by-step assembly instructions
└── README.md                       # This file
```

---

## Architecture

For comprehensive details on the system architecture and product evaluation, please refer to [architecture.md](/docs/architecture.md) in the docs directory.

### System Overview

The Àgbẹ̀ robot employs a distributed control architecture with the following key components:

#### Hardware Architecture

**Mechanical Systems:**
- Low ground pressure chassis design (see `docs/hardware/cad/` for Fusion 360 files)
- Seeding mechanism with precise seed dispensing
- Tricycle inspired design with heading 

**Electronics:**
- Microcontroller-based control system (Arduino-compatible)
- Sensor suite for navigation and environmental monitoring
- WiFi communication module for remote control
- Power management system

**Visual Resources:**
- **3D Models**: `docs/hardware/cad/` - Fusion 360 design files
- **Renders**: `docs/images/renders/` - High-quality visualizations
- **Wiring**: `docs/images/wiring/` - Complete wiring diagrams
- **System Diagrams**: `docs/images/system_diagram/` - Architecture overview

#### Software Architecture

**Firmware Layer:**

The robot operates on a dual-microcontroller architecture:

1. **Low-Level Control** (`software/firmware/AutonomousSeeding.ino`)
   - Motor control and PWM signal generation
   - Sensor data acquisition (GPS, IMU, proximity sensors)
   - Seeding mechanism actuation
   - Real-time control loops
   - Safety monitoring and emergency stop

2. **WiFi Communications** (`software/firmware/wifi_comms.ino`)
   - WiFi connectivity management
   - Remote command reception
   - Telemetry data transmission
   - Web interface integration

**Application Layer:**

3. **Web Application** (`software/webapp/`)
   - User interface for robot control
   - Real-time monitoring dashboard
   - Data visualization and logging
   - Framework: Vanilla JS

#### Communication Architecture

```
[Web Browser] <---> [Web App] <--WiFi--> [WiFi Comms Module] <--Serial--> [Low-Level Controller]

```
#### Technology Stack

**Embedded Systems:**
- Arduino-compatible microcontrollers
- C/C++ for firmware development
- Arduino Mega

**Web Technologies:**
- Frontend: Vanilla JS, Leaflet JS (GPS Visualisation) 
- WiFi communication protocol: HTTP

**Design Tools:**
- Autodesk Fusion 360 for mechanical design

---

## Documentation

### Core Documents

#### 📘 Architecture & Evaluation Documents
**File**: `architecture.md`

This comprehensive document covers:
- Detailed system architecture
- Hardware component specifications
- Software design rationale

**File**: `evaluation.md`
- Performance evaluation and testing results
- Design decisions and trade-offs
- Cost-benefit analysis
- Comparison with existing solutions

### Visual Documentation

#### `docs/images/renders/`
High-quality 3D renders showcasing:
- Complete robot assembly
- Individual subsystems

#### `docs/images/testing/`
Field testing documentation:
- Deployment photos
- Operational scenarios
- Performance demonstrations

#### `docs/images/wiring/`
Electrical documentation:
- Complete wiring diagrams
- Pin assignments
- Connection schematics
- Power distribution

#### `docs/images/system_diagram/`
System-level diagrams:
- Architecture overview
- Control system hierarchy
- Communication protocols

## Hardware Documentation

#### `docs/hardware/cad/`
Fusion 360 CAD files including:
- Complete assembly models
- Subsystem Model

---

## Getting Started

### Prerequisites

**Hardware Development:**
- Arduino IDE
- USB cable for microcontroller programming
- Components found in the [Bill Of Materials](/docs/BOM.md)

**CAD Viewing/Editing:**
- Autodesk Fusion 360 (for viewing/modifying CAD files)

**Web Development:**
- Webapp deployed locally from Python
- Web browser (Chrome/Firefox recommended)

### Assembly

1. **Mechanical Assembly**
   - Follow the complete instructions in `assembly_guide.pdf`
   - Use CAD files in `docs/hardware/cad/` for reference
   - Refer to `docs/images/renders/` for visual guidance

2. **Electronics Integration**
   - Follow wiring diagrams in `docs/images/wiring/`
   - Consult system diagrams in `docs/images/system_diagram/`
   - Double-check all connections before powering on

### Software Setup

#### Firmware Installation

**Low-Level Control Firmware**
```bash
    # Open AutonomousSeeding.ino in Arduino IDE
    # Select appropriate board and port
    # Open wifi_comms.ino in Arduino IDE
    # Configure WiFi credentials
    # Upload to primary microcontroller
```

#### Web Application Setup

```bash
cd software/webapp
pip install requirements.txt
python3 -m http.server 8080
```

---

## Usage

### Single Robot Operation

1. **Power On**: Flick Switch
2. **Connect**: Access web interface at [http://localhost:8080/] 
3. **Configure**: Set seeding parameters (spacing, depth, seed type)
4. **Deploy**: Define mission area and waypoints (Future implementation)
5. **Monitor**: Track progress via web dashboard

---

## Development

### Firmware Development

**Low-Level Control** (`software/firmware/AutonomousSeeding.ino`):
- Modify control algorithms
- Add new sensor support (Integration of rotary encoder)
- Adjust motor parameters
- Implement safety features

**WiFi Communications** (`software/firmware/wifi_comms.ino`):
- Update communication protocols
- Add new commands
- Improve telemetry
- Enhance swarm coordination

**Web Application** 
- Integration of kalman filter logic and tuning of kalman filter 
- Code structure optimisation

## Future Work

- [ ] Enhanced obstacle detection and avoidance
- [ ] Implement Boundary Detection and improved mapping
- [ ] Machine learning for optimal seeding patterns
- [ ] Extended battery life / solar integration
- [ ] Implement swarm algorithms for larger fleets
- [ ] Mobile app development
- [ ] Integration with precision agriculture sensors

---


## Acknowledgments

This project was completed as part of a Robotics Internship with the **National Center for Artificial Intelligence and Robotics (NCAIR)** in Abuja, Nigeria, under the National Information Technology Development Agency (NITDA).

### Project Name Origin

**Àgbẹ̀** - A Yoruba word meaning "farmer," reflecting the robot's purpose in supporting Nigerian agriculture.


## Quick Links

- 📘 [Architecture](/docs/architecture.md)
- 📘 [Evaluation](/docs/evaluation.md)
- 🔧 [Assembly Guide](/docs/assemblyGuide.md)
- 🎨 [3D Models (CAD)](./docs/hardware/cad/)
- 🖼️ [Renders](./docs/images/renders/)
- 📸 [Testing Photos](./docs/images/testing/)
- 🔌 [Wiring Diagrams](./docs/images/wiring/)
- 📊 [System Diagrams](./docs/images/system_diagram/)
- 💻 [Firmware](./software/firmware/)
- 🌐 [Web Application](./software/webapp/)