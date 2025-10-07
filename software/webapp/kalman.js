// class KalmanFilter {
//   constructor({ R, Q, A, B, H }) {
//     this.R = R; // Measurement noise covariance
//     this.Q = Q; // Process noise covariance
//     this.A = A; // State transition
//     this.B = B; // Control matrix
//     this.H = H; // Observation model

//     this.x = math.zeros(2, 1); // Initial state [position; velocity]
//     this.P = math.identity(2); // Initial covariance
//   }

//   predict(u = 0) {
//     this.x = math.add(math.multiply(this.A, this.x), math.multiply(this.B, u));
//     this.P = math.add(math.multiply(math.multiply(this.A, this.P), math.transpose(this.A)), this.Q);
//     return this.x;
//   }

//   correct(z) {
//     const S = math.add(math.multiply(math.multiply(this.H, this.P), math.transpose(this.H)), this.R);
//     const K = math.multiply(this.P, math.multiply(math.transpose(this.H), math.inv(S)));
//     this.x = math.add(this.x, math.multiply(K, math.subtract(z, math.multiply(this.H, this.x))));
//     this.P = math.multiply(math.subtract(math.identity(this.P.size()[0]), math.multiply(K, this.H)), this.P);
//     return this.x;
//   }
// }


class KalmanFilter {
    constructor({ R, Q, A, B, H, dt = 0.1 }) {
        this.R = R; // Measurement noise covariance
        this.Q = Q; // Process noise covariance
        this.A = A; // State transition
        this.B = B; // Control matrix
        this.H = H; // Observation model
        this.dt = dt; // Time step

        // State vector: [x_pos, y_pos, velocity, heading]
        this.x = math.zeros(4, 1);
        this.P = math.multiply(math.identity(4), 10); // Initial covariance with high uncertainty
    }

    predict(u = 0) {
        this.x = math.add(math.multiply(this.A, this.x), math.multiply(this.B, u));
        this.P = math.add(
        math.multiply(math.multiply(this.A, this.P), math.transpose(this.A)),
        this.Q
        );
        return this.x;
    }

    correct(z) {
        const S = math.add(
        math.multiply(math.multiply(this.H, this.P), math.transpose(this.H)),
        this.R
        );
        const K = math.multiply(
        this.P,
        math.multiply(math.transpose(this.H), math.inv(S))
        );
        this.x = math.add(
        this.x,
        math.multiply(K, math.subtract(z, math.multiply(this.H, this.x)))
        );
        this.P = math.multiply(
        math.subtract(math.identity(this.P.size()[0]), math.multiply(K, this.H)),
        this.P
        );
        return this.x;
    }

    getState() {
        return {
        x: this.x.get([0, 0]),
        y: this.x.get([1, 0]),
        velocity: this.x.get([2, 0]),
        heading: this.x.get([3, 0])
        };
    }
}

class MultiSensorFusion {
    constructor(dt = 0.1) {
        this.dt = dt;

        // State transition matrix A
        // [x, y, v, heading]
        this.A = math.matrix([
        [1, 0, 0, 0], // x position
        [0, 1, 0, 0], // y position
        [0, 0, 1, 0], // velocity
        [0, 0, 0, 1]  // heading
        ]);

        // Control input matrix B (encoder velocity and gyro heading rate)
        this.B = math.matrix([
        [dt, 0],  // x += v*cos(heading)*dt
        [dt, 0],  // y += v*sin(heading)*dt
        [1, 0],   // velocity update from encoder
        [0, dt]   // heading += gyro_rate*dt
        ]);

        // Observation matrix H (GPS measures x, y position)
        this.H = math.matrix([
        [1, 0, 0, 0], // GPS x
        [0, 1, 0, 0]  // GPS y
        ]);

        // Process noise covariance Q
        this.Q = math.matrix([
        [0.1, 0, 0, 0],
        [0, 0.1, 0, 0],
        [0, 0, 0.5, 0],
        [0, 0, 0, 0.1]
        ]);

        // Measurement noise covariance R (GPS accuracy)
        this.R = math.matrix([
        [2.5, 0],   // GPS x noise (~2-5m typical)
        [0, 2.5]    // GPS y noise
        ]);

        this.kf = new KalmanFilter({
        R: this.R,
        Q: this.Q,
        A: this.A,
        B: this.B,
        H: this.H,
        dt: dt
        });

        // Encoder parameters
        this.encoderCountsPerRev = 20; // HC-020K has 20 counts per revolution
        this.wheelCircumference = 0.198; // meters (adjust for your wheel)
        this.lastEncoderCount = 0;
    }

    /**
     * Calculate velocity from encoder counts
     * @param {number} encoderCount - Current encoder count
     * @returns {number} velocity in m/s
     */
    calculateVelocity(encoderCount) {
        const deltaCount = encoderCount - this.lastEncoderCount;
        this.lastEncoderCount = encoderCount;
        
        const revolutions = deltaCount / this.encoderCountsPerRev;
        const distance = revolutions * this.wheelCircumference;
        const velocity = distance / this.dt;
        
        return velocity;
    }

    /**
     * Prediction step using encoder and gyroscope
     * @param {number} encoderCount - Current encoder count
     * @param {number} gyroZ - Gyroscope Z-axis (yaw rate) in rad/s
     */
    predict(encoderCount, gyroZ) {
        const velocity = this.calculateVelocity(encoderCount);
        
        // Control input: [velocity, gyro_rate]
        const u = math.matrix([[velocity], [gyroZ]]);
        
        // Update state transition to account for heading in position updates
        const currentHeading = this.kf.x.get([3, 0]);
        this.kf.A.set([0, 2], this.dt * Math.cos(currentHeading));
        this.kf.A.set([1, 2], this.dt * Math.sin(currentHeading));
        
        return this.kf.predict(u);
    }

    /**
     * Correction step using GPS position
     * @param {number} gpsLat - GPS latitude
     * @param {number} gpsLon - GPS longitude
     * @param {number} refLat - Reference latitude (origin)
     * @param {number} refLon - Reference longitude (origin)
     */
    correctWithGPS(gpsLat, gpsLon, refLat, refLon) {
        // Convert GPS coordinates to local x,y (meters)
        const x = (gpsLon - refLon) * 111320 * Math.cos(refLat * Math.PI / 180);
        const y = (gpsLat - refLat) * 110540;
        
        // Measurement vector: [x, y]
        const z = math.matrix([[x], [y]]);
        
        return this.kf.correct(z);
    }

    /**
    * Get current estimated state
    */
    getState() {
        return this.kf.getState();
    }

    /**
     * Reset the filter
     */
    reset() {
        this.kf.x = math.zeros(4, 1);
        this.kf.P = math.multiply(math.identity(4), 10);
        this.lastEncoderCount = 0;
    }
}
