# Bill of Materials (BOM)

| Category        | Component                          | Quantity | Purpose                                                                 |
|-----------------|------------------------------------|----------|-------------------------------------------------------------------------|
| **Motion**      | Ultrasonic Sensor: HC-SR04         | 1        | Object detection to prevent collisions                         |
|                 | DC Motors            | 3        | Motion                                                                  |
|                 | Servo Motor (MG-996R)              | 1        | 1× back wheel rotation                                                  |
|                 | Motor Driver: L298N                | 2        | To drive the three wheels                                               |
|                 | Rotary Encoder (Optical) - HC-020K | 1        | Backup for DC motor with encoder                                        |
|                 | Neo 6m GPS                         | 1        | Robot localisation in space, used to create boundaries                  |
|                 | IMU - MPU6050                      | 1        | To calculate orientation in space                                       |
| **Seed Dispersion** | Stepper Motor: 28BYJ-48 (Box 068) | 2     | Turns the grate at set encoder readings to release grain at intervals   |
|                 | Stepper Motor Driver: ULN2003      | 1        | Drives the stepper motor                                                |
|                 | Ball Bearings                      | 2        | Furrow rotation                                                         |
| **MCU**         | Arduino Mega                       | 1        | Main controller (“Brain”)                                               |
| **Safety & Power** | Kill Switch                     | 1        | Stops the robot in case of malfunction                                  |
|                 | Battery (11.1V)                    | 1        | Power supply                                                            |
