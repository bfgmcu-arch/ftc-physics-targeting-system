# FTC Advanced Physics-Based Real-Time Flywheel Targeting System

A deterministic, zero-heap-allocation, physics-based targeting, trajectory calculation, localization fusion, and closed-loop flywheel control system for FIRST Tech Challenge (FTC) robots.

## Key Capabilities

- **First-Principles Physics Engine (`PhysicsEngine`)**: Models single-wheel and dual-wheel surface kinematics, rotational-to-translational kinetic energy transfer, inelastic projectile compression losses, speed-dependent wheel slip, quadratic aerodynamic drag, and Magnus backspin lift.
- **Zero-Allocation 3D Trajectory Solver (`TrajectoryCalculator`)**: Computes optimal launch elevation angles, required exit velocities, flywheel RPMs, 3D trajectory waypoints, and Rayleigh/Gaussian uncertainty propagation for shot confidence.
- **Latency-Compensated Localization & $SE(2)$ Launcher Offset (`RobotLocalization`)**: Fuses dead-wheel/Pinpoint/OTOS odometry with delayed AprilTag vision frames using a 32-slot historical pose ring buffer and transforms robot center pose/velocity to the physical launcher exit point.
- **Coupled Moving-Robot Predictive Targeting (`ShotPrediction`)**: Predicts future launcher release state over sensor + feeder + spool delays using $SE(2)$ constant-curvature kinematics and solves the coupled moving-frame boundary-value problem.
- **Voltage-Normalized Closed-Loop Flywheel Controller (`FlywheelController`)**: Combines an $\alpha$-$\beta$ velocity/acceleration observer, $k_S/k_V/k_A$ physics feedforward, battery voltage sag compensation, conditional anti-windup PID + Take-Back-Half (TBH), transient shot-recovery boost, and analytical recovery time prediction.
- **Hardware-In-The-Loop Launch Detector & Auto-Characterization (`LaunchEventDetector`, `FlywheelCharacterizationRoutine`)**: Reconstructs actual feeder actuation delay and projectile exit velocity from flywheel encoder deceleration spikes, classifies feed anomalies, and auto-tunes $(k_S, k_V, k_A, k_P, k_I, k_D)$ on the robot.
- **Physics-Informed Experimental Calibration (`ShotCalibration`, `CalibrationPersistenceManager`)**: Asynchronous Tikhonov-regularized (Ridge) Gauss-Newton solver that refines transfer efficiency, aerodynamic drag scale, and mechanical pitch bias from recorded shots without overriding core physics.

## Verified Performance

- **Average Physics + Trajectory Calculation Time:** `0.0057 ms` (`5.68 µs`)
- **Average Total End-to-End Targeting Cycle Time:** `0.0066 ms` (`6.55 µs`)
- **3D Moving & Stationary Impact Accuracy:** `< 1.0 mm` solver residual

## Running the Validation & Benchmark Suite

```powershell
javac -d build/classes src/main/java/org/firstinspires/ftc/teamcode/targeting/*.java
java -cp build/classes org.firstinspires.ftc.teamcode.targeting.TargetingValidationAndBenchmark
```
