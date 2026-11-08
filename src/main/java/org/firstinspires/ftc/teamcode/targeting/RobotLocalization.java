package org.firstinspires.ftc.teamcode.targeting;

import org.firstinspires.ftc.teamcode.targeting.TargetingTypes.Pose2d;
import org.firstinspires.ftc.teamcode.targeting.TargetingTypes.SensorSnapshot;
import org.firstinspires.ftc.teamcode.targeting.TargetingTypes.Velocity2d;

/**
 * Real-time Robot Localization & Launcher Exit Frame Kinematics.
 *
 * <p>Fuses high-rate odometry with latency-compensated AprilTag vision observations using a
 * lightweight constant-allocation Kalman filter, tracks pose uncertainty covariance, and transforms
 * the robot reference pose/velocity to the physical launcher exit point in SE(2).
 */
public final class RobotLocalization {

    private static final int HISTORY_CAPACITY = 32;

    private final ShooterConfig config;

    // Fused robot center pose & exponentially smoothed field velocity
    private final Pose2d fusedRobotPose = new Pose2d();
    private final Velocity2d smoothedRobotVelocity = new Velocity2d();

    // Computed launcher exit point pose & field velocity
    private final Pose2d launcherExitPose = new Pose2d();
    private final Velocity2d launcherExitVelocity = new Velocity2d();

    // Previous raw odometry reading for incremental dead-reckoning delta
    private final Pose2d lastRawOdometryPose = new Pose2d();
    private boolean initialized = false;

    // Pre-allocated ring buffer for latency-compensated AprilTag fusion
    private final double[] historyTimestampsSec = new double[HISTORY_CAPACITY];
    private final double[] historyXM = new double[HISTORY_CAPACITY];
    private final double[] historyYM = new double[HISTORY_CAPACITY];
    private final double[] historyHeadingRad = new double[HISTORY_CAPACITY];
    private int historyHead = 0;
    private int historySize = 0;
    private double elapsedClockSec = 0.0;

    // Uncertainty tracking (1-sigma standard deviations)
    private double positionVarianceM2 = 0.0004; // (2 cm)^2 initial variance
    private double headingVarianceRad2 = 0.0001; // (~0.57 deg)^2 initial variance

    public RobotLocalization(ShooterConfig config) {
        this.config = config;
    }

    /** Explicitly resets the localization pose and covariance. */
    public void resetPose(double xM, double yM, double headingRad) {
        fusedRobotPose.set(xM, yM, headingRad);
        lastRawOdometryPose.set(xM, yM, headingRad);
        smoothedRobotVelocity.set(0.0, 0.0, 0.0);
        positionVarianceM2 = 0.0004;
        headingVarianceRad2 = 0.0001;
        historyHead = 0;
        historySize = 0;
        initialized = true;
        updateLauncherKinematics();
    }

    /**
     * Updates the fused robot pose, uncertainty covariance, and launcher exit kinematics from the
     * latest atomic {@link SensorSnapshot}. Zero heap allocations.
     */
    public void update(SensorSnapshot sensors) {
        double dt = Math.max(1e-4, sensors.dtSec);
        elapsedClockSec += dt;

        if (!initialized) {
            resetPose(
                    sensors.odometryPose.xM,
                    sensors.odometryPose.yM,
                    sensors.odometryPose.headingRad);
        }

        // 1. Odometry Prediction Step
        double dOdomX = sensors.odometryPose.xM - lastRawOdometryPose.xM;
        double dOdomY = sensors.odometryPose.yM - lastRawOdometryPose.yM;
        double dOdomHeading =
                TrajectoryCalculator.normalizeAngleRadians(
                        sensors.odometryPose.headingRad - lastRawOdometryPose.headingRad);

        lastRawOdometryPose.copyFrom(sensors.odometryPose);

        fusedRobotPose.xM += dOdomX;
        fusedRobotPose.yM += dOdomY;
        fusedRobotPose.headingRad =
                TrajectoryCalculator.normalizeAngleRadians(fusedRobotPose.headingRad + dOdomHeading);

        // Grow uncertainty proportional to distance traveled (odometry wheel slip / drift)
        double stepDistM = Math.hypot(dOdomX, dOdomY);
        positionVarianceM2 = Math.min(0.25, positionVarianceM2 + 0.0015 * stepDistM + 0.00005 * dt);
        headingVarianceRad2 =
                Math.min(0.04, headingVarianceRad2 + 0.0002 * Math.abs(dOdomHeading) + 0.00001 * dt);

        // Record pose in ring buffer for latency compensation
        recordPoseHistory(elapsedClockSec, fusedRobotPose);

        // 2. Low-Pass Exponential Velocity Filtering (suppresses encoder quantization noise)
        double alphaVel = dt / (0.018 + dt); // ~18 ms time constant
        smoothedRobotVelocity.vxMpS +=
                alphaVel * (sensors.robotFieldVelocity.vxMpS - smoothedRobotVelocity.vxMpS);
        smoothedRobotVelocity.vyMpS +=
                alphaVel * (sensors.robotFieldVelocity.vyMpS - smoothedRobotVelocity.vyMpS);
        smoothedRobotVelocity.omegaRadPerSec +=
                alphaVel * (sensors.robotFieldVelocity.omegaRadPerSec - smoothedRobotVelocity.omegaRadPerSec);

        // 3. Latency-Compensated AprilTag Measurement Update
        if (sensors.aprilTagDetected && sensors.aprilTagAmbiguity < 0.45) {
            double captureTimeSec = elapsedClockSec - Math.max(0.0, sensors.aprilTagLatencySec);
            // Lookup historical pose at camera frame capture timestamp
            double histX = fusedRobotPose.xM;
            double histY = fusedRobotPose.yM;
            double histHeading = fusedRobotPose.headingRad;

            if (historySize > 0) {
                int bestIdx = findClosestHistoryIndex(captureTimeSec);
                histX = historyXM[bestIdx];
                histY = historyYM[bestIdx];
                histHeading = historyHeadingRad[bestIdx];
            }

            // Measurement noise R scales with tag ambiguity
            double measPosVar = 0.0012 + 0.025 * sensors.aprilTagAmbiguity * sensors.aprilTagAmbiguity;
            double measHeadingVar = 0.0008 + 0.015 * sensors.aprilTagAmbiguity;

            // Scalar Kalman gains K = P / (P + R)
            double kPos = positionVarianceM2 / (positionVarianceM2 + measPosVar);
            double kHeading = headingVarianceRad2 / (headingVarianceRad2 + measHeadingVar);

            double innovX = sensors.aprilTagPose.xM - histX;
            double innovY = sensors.aprilTagPose.yM - histY;
            double innovHeading =
                    TrajectoryCalculator.normalizeAngleRadians(
                            sensors.aprilTagPose.headingRad - histHeading);

            // Outlier rejection (Mahalanobis gating at ~3.5 sigma)
            if (Math.hypot(innovX, innovY) < 1.2) {
                double corrX = kPos * innovX;
                double corrY = kPos * innovY;
                double corrHeading = kHeading * innovHeading;

                fusedRobotPose.xM += corrX;
                fusedRobotPose.yM += corrY;
                fusedRobotPose.headingRad =
                        TrajectoryCalculator.normalizeAngleRadians(
                                fusedRobotPose.headingRad + corrHeading);

                // Shift history buffer by applied innovation so subsequent frames remain consistent
                for (int i = 0; i < historySize; i++) {
                    historyXM[i] += corrX;
                    historyYM[i] += corrY;
                    historyHeadingRad[i] =
                            TrajectoryCalculator.normalizeAngleRadians(historyHeadingRad[i] + corrHeading);
                }

                positionVarianceM2 *= (1.0 - kPos);
                headingVarianceRad2 *= (1.0 - kHeading);
            }
        }

        // 4. Transform Robot Pose & Velocity to Physical Launcher Exit Point in SE(2)
        updateLauncherKinematics();
    }

    private void updateLauncherKinematics() {
        double psi = fusedRobotPose.headingRad;
        double cosPsi = Math.cos(psi);
        double sinPsi = Math.sin(psi);
        double ox = config.launcherOffsetXM;
        double oy = config.launcherOffsetYM;

        // p_launcher = p_robot + R(psi) * offset
        double rotOffsetX = ox * cosPsi - oy * sinPsi;
        double rotOffsetY = ox * sinPsi + oy * cosPsi;

        launcherExitPose.set(
                fusedRobotPose.xM + rotOffsetX,
                fusedRobotPose.yM + rotOffsetY,
                TrajectoryCalculator.normalizeAngleRadians(psi + config.launcherYawMountOffsetRad));

        // v_launcher = v_robot + omega x (R(psi) * offset)
        double omega = smoothedRobotVelocity.omegaRadPerSec;
        double tangentialVx = -omega * rotOffsetY;
        double tangentialVy = omega * rotOffsetX;

        launcherExitVelocity.set(
                smoothedRobotVelocity.vxMpS + tangentialVx,
                smoothedRobotVelocity.vyMpS + tangentialVy,
                omega);
    }

    private void recordPoseHistory(double timeSec, Pose2d pose) {
        historyTimestampsSec[historyHead] = timeSec;
        historyXM[historyHead] = pose.xM;
        historyYM[historyHead] = pose.yM;
        historyHeadingRad[historyHead] = pose.headingRad;
        historyHead = (historyHead + 1) % HISTORY_CAPACITY;
        if (historySize < HISTORY_CAPACITY) {
            historySize++;
        }
    }

    private int findClosestHistoryIndex(double targetTimeSec) {
        int bestIdx = 0;
        double minDiff = Double.MAX_VALUE;
        for (int i = 0; i < historySize; i++) {
            double diff = Math.abs(historyTimestampsSec[i] - targetTimeSec);
            if (diff < minDiff) {
                minDiff = diff;
                bestIdx = i;
            }
        }
        return bestIdx;
    }

    public Pose2d getRobotPose() {
        return fusedRobotPose;
    }

    public Velocity2d getRobotVelocity() {
        return smoothedRobotVelocity;
    }

    public Pose2d getLauncherExitPose() {
        return launcherExitPose;
    }

    public Velocity2d getLauncherExitVelocity() {
        return launcherExitVelocity;
    }

    public double getPositionStdDevM() {
        return Math.sqrt(positionVarianceM2);
    }

    public double getHeadingStdDevRad() {
        return Math.sqrt(headingVarianceRad2);
    }

    public boolean isLocalizationConfident() {
        return getPositionStdDevM() < 0.18 && getHeadingStdDevRad() < 0.085;
    }
}
