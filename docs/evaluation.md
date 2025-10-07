# Evaluation and Analysis of Current Implementation

## 1. Introduction

This document aims to critically analyse the current implementation and solution in addition to suggesting short-term and long-term improvements. Unfortunately a field test could not be completed, so it is likely further analysis needs to be conducted to understand how the robot performs in its operating environment. 

---

## 2. Mechanical Elements

### 2.1 Wheels

The current wheel design repurposes 3D printer filament spools as hubs, fitted with interlocking PLA treads. Due to the absence of a detailed assembly plan, inconsistencies arose during manufacturing—most notably, the rear wheel treads are slightly offset from center. This misalignment causes the raised tread sections to contact the servo mount, increasing friction, reducing drive efficiency, and occasionally stalling the motor. Such mechanical irregularities can introduce unpredictable behavior, complicating state estimation and localisation. To address this, future iterations should utilize the [Wheel Guide](/hardware/cad/WheelGuide.dxf) for precise assembly, and consider extending the guide to cover the full wheel circumference to prevent misalignment.

### 2.2 Structural Integrity 

The initial lack of a detailed assembly plan led to ad hoc design decisions regarding structural connections. While brass thermal inserts were considered for robust assembly, practical constraints necessitated alternative approaches, resulting in a structure with reduced rigidity. The modular design—comprising separate electronics housing, seed dispensing system, and rear wheel/servo modules—relies on a single laser-cut chassis piece for integration. Although this simplifies assembly, it introduces structural weak points and potential failure modes, particularly at the central chassis interface. For improved durability and reliability, future iterations should reinforce these connections and distribute mechanical loads across multiple attachment points to minimize dependence on a single structural element.

### 2.3 Design

Late-stage modifications to the electronics housing, such as adding an access port for Arduino debugging and firmware updates, were not fully integrated into the initial design. As a result, the port’s placement requires removal of the front right wheel for access, increasing maintenance complexity and accelerating wear on the wheel hub assembly. Similarly, the allocated space for battery recharging proved inadequate once the robot was fully assembled, complicating routine servicing. These oversights highlight the importance of design-for-maintenance principles; future iterations should prioritize accessible component placement and serviceability to streamline development, testing, and field repairs.

Assembly complexity was a significant challenge, particularly within the electronics hub. Once the battery and power distribution circuitry were installed, wiring became congested and disorganized, complicating both assembly and subsequent debugging. This increased build time and hindered fault isolation during testing. To address these issues, future iterations should implement systematic cable management and consider integrating a custom PCB to minimize external wiring and streamline assembly.

---

## 3. Software Elements 

### 3.1 Encoder

At this stage, the rotary encoder was not installed due to sourcing delays, limiting state estimation to low-frequency GPS updates. This constraint prevents accurate real-time odometry and degrades localisation performance, especially during maneuvers. Even with a rear-mounted encoder, wheel slippage can introduce measurement errors, and the absence of front wheel encoders impedes precise tracking during turns. While heading estimation can be partially compensated using gyroscope data, closed-loop speed and position control of the drive motors remains challenging without direct feedback. For the current semi-autonomous setup, users must manually estimate turn completion, which is inefficient and unreliable. Future designs should incorporate encoders on all driven wheels and consider mechanical improvements to minimise slippage, ensuring robust odometry for autonomous operation.

### 3.2 Sensor Fusion

While sensor fusion has been implemented on the PC side in javascript, further work needs be done to poll the sensors correctly from the robot. Currently, only GPS data is being transmitted so the remaining sensors have not been configured to send data properly, however timings can be inferred from the [mention file]

---

## 4. Further Implementation

### 4.1 Suspension

The current robot design lacks any form of suspension system, which significantly impacts its stability and operational reliability when traversing uneven or soft agricultural terrain. In the absence of suspension, shocks and vibrations from obstacles such as potholes, clods, or ruts are directly transmitted to the chassis and sensitive components, increasing the risk of mechanical wear, sensor inaccuracies, and loss of traction. This issue is exacerbated under load, where weight distribution can further amplify instability and wheel slip, potentially leading to erratic motion or stalling.

Integrating a suspension mechanism—such as spring-damper assemblies or flexible linkages—would decouple the chassis from ground-induced disturbances, maintaining consistent wheel contact and improving both traction and ride quality. This enhancement would not only protect onboard electronics and actuators from mechanical stress but also contribute to more accurate odometry and sensor readings, which are critical for reliable navigation and seed placement. Future iterations should prioritize the evaluation and implementation of suitable suspension architectures tailored for low-speed, lightweight field robots.

### 4.2 Navigation

As highlighted the current implementation is semi-autonomous but requires a lot of human input and monitoring for it be used. Implementing a navigation algorithm for path planning would enable this system to become fully autonomous. Below are some papers that tackle the issue of path planning for this exact use case.

**Relevant Links:**
- https://arxiv.org/abs/2210.07838
- https://www.mdpi.com/2073-4395/10/10/1608#:~:text=Agricultural%20field%20operations%20such%20as,ends%20at%20the%20same%20depot.


### 4.3 Modular Attachments

While this MVP is focused on seeding, the robot’s architecture is inherently modular, enabling rapid adaptation for a variety of agricultural tasks through interchangeable attachments. Adopting a tool-change paradigm similar to [OZ](https://www.naio-technologies.com/en/oz-robot/) by Naio Technologies, future iterations could support quick-swap modules for expanded functionality.

Potential modules include:
- **Camera Module:** Enables crop monitoring, weed and pest detection, and plant health assessment. Integrating an onboard computer vision pipeline would allow real-time analysis and decision-making in the field.
- **Weeding Module:** Mechanically removes weeds identified by the vision system, supporting precision agriculture and reducing manual labor. This could leverage existing mechanical or chemical weeding solutions, triggered by the detection module.

The modular approach not only increases the robot’s versatility but also streamlines maintenance and upgrades, as individual components can be serviced or replaced independently. Future work should focus on standardizing mechanical and electrical interfaces for attachments, and developing a robust software framework for dynamic tool recognition and control.

**Relevant Links:**
- https://link.springer.com/article/10.1007/s43154-022-00086-5

### 4.4 Swarm Approach

To further enhance operational efficiency, a swarm paradigm could be implemented, enabling multiple robots to collaboratively cover larger field areas. This approach allows the system to achieve throughput comparable to that of larger agricultural machinery, while minimising soil compaction and damage due to the distributed weight of smaller units. Swarm robotics also facilitates task specialisation, where individual robots can be equipped with different attachments (e.g., seeding, weeding, monitoring) and coordinate their actions to address diverse farm requirements. Implementing robust inter-robot communication and decentralized task allocation algorithms will be essential for effective collaboration and scalability in real-world agricultural environments.


