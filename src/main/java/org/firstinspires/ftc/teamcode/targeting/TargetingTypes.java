package org.firstinspires.ftc.teamcode.targeting;

/**
 * Pre-allocated, zero-allocation data structures for real-time targeting, localization,
 * trajectory prediction, and shot validation.
 *
 * <p>All structures provide in-place mutators ({@code set} / {@code copyFrom}) so the real-time
 * control loop never allocates objects on the heap or triggers Android/JVM Garbage Collection.
 */
public final class TargetingTypes {

    private TargetingTypes() {}

    /** 2D robot or launcher pose on the FTC field (SI units: meters and radians). */
    public static final class Pose2d {
        public double xM;
        public double yM;
        public double headingRad;

        public Pose2d() {}

        public Pose2d(double xM, double yM, double headingRad) {
            this.xM = xM;
            this.yM = yM;
            this.headingRad = headingRad;
        }

        public void set(double xM, double yM, double headingRad) {
            this.xM = xM;
            this.yM = yM;
            this.headingRad = headingRad;
        }

        public void copyFrom(Pose2d other) {
            this.xM = other.xM;
            this.yM = other.yM;
            this.headingRad = other.headingRad;
        }
    }

    /** 2D translational and angular velocity in field or robot coordinates (m/s and rad/s). */
    public static final class Velocity2d {
        public double vxMpS;
        public double vyMpS;
        public double omegaRadPerSec;

        public Velocity2d() {}

        public Velocity2d(double vxMpS, double vyMpS, double omegaRadPerSec) {
            this.vxMpS = vxMpS;
            this.vyMpS = vyMpS;
            this.omegaRadPerSec = omegaRadPerSec;
        }

        public void set(double vxMpS, double vyMpS, double omegaRadPerSec) {
            this.vxMpS = vxMpS;
            this.vyMpS = vyMpS;
            this.omegaRadPerSec = omegaRadPerSec;
        }

        public void copyFrom(Velocity2d other) {
            this.vxMpS = other.vxMpS;
            this.vyMpS = other.vyMpS;
            this.omegaRadPerSec = other.omegaRadPerSec;
        }

        public double linearSpeedMpS() {
            return Math.hypot(vxMpS, vyMpS);
        }
    }

    /** 3D target specification on the field (SI units: meters and radians). */
    public static final class TargetGeometry {
        public double centerXM;
        public double centerYM;
        public double centerZM;
        /** Effective target opening radius or half-width for confidence & tolerance calculations (m). */
        public double apertureRadiusM;
        /** Preferred entry elevation angle into the target (radians, 0 if omnidirectional/vertical). */
        public double preferredEntryAngleRad;

        public TargetGeometry() {}

        public TargetGeometry(
                double centerXM,
                double centerYM,
                double centerZM,
                double apertureRadiusM,
                double preferredEntryAngleRad) {
            set(centerXM, centerYM, centerZM, apertureRadiusM, preferredEntryAngleRad);
        }

        public void set(
                double centerXM,
                double centerYM,
                double centerZM,
                double apertureRadiusM,
                double preferredEntryAngleRad) {
            this.centerXM = centerXM;
            this.centerYM = centerYM;
            this.centerZM = centerZM;
            this.apertureRadiusM = apertureRadiusM;
            this.preferredEntryAngleRad = preferredEntryAngleRad;
        }

        public void copyFrom(TargetGeometry other) {
            this.centerXM = other.centerXM;
            this.centerYM = other.centerYM;
            this.centerZM = other.centerZM;
            this.apertureRadiusM = other.apertureRadiusM;
            this.preferredEntryAngleRad = other.preferredEntryAngleRad;
        }
    }

    /** Single 3D point along a predicted projectile trajectory in field coordinates. */
    public static final class TrajectoryPoint {
        public double timeSec;
        public double xM;
        public double yM;
        public double zM;
        public double vxMpS;
        public double vyMpS;
        public double vzMpS;

        public void set(
                double timeSec,
                double xM,
                double yM,
                double zM,
                double vxMpS,
                double vyMpS,
                double vzMpS) {
            this.timeSec = timeSec;
            this.xM = xM;
            this.yM = yM;
            this.zM = zM;
            this.vxMpS = vxMpS;
            this.vyMpS = vyMpS;
            this.vzMpS = vzMpS;
        }
    }

    /** Complete pre-allocated trajectory buffer (fixed number of waypoints to avoid heap allocation). */
    public static final class TrajectoryBuffer {
        public static final int MAX_WAYPOINTS = 16;
        public final TrajectoryPoint[] points = new TrajectoryPoint[MAX_WAYPOINTS];
        public int count = 0;
        public double apexHeightM = 0.0;
        public double impactEntryAngleRad = 0.0;

        public TrajectoryBuffer() {
            for (int i = 0; i < MAX_WAYPOINTS; i++) {
                points[i] = new TrajectoryPoint();
            }
        }

        public void clear() {
            count = 0;
            apexHeightM = 0.0;
            impactEntryAngleRad = 0.0;
        }
    }

    /**
     * Output of the physics & trajectory targeting solver for a single control-loop cycle.
     */
    public static final class LaunchSolution {
        public boolean isAchievable;
        public String infeasibilityReason = "OK";

        /** Horizontal Euclidean distance from predicted launcher exit point to target center (m). */
        public double horizontalDistanceM;
        /** Vertical displacement (target Z minus launcher exit Z) (m). */
        public double verticalDisplacementM;

        /** Required projectile total exit speed relative to the launcher (m/s). */
        public double requiredExitVelocityMpS;
        /** Required projectile exit speed in the stationary field frame (m/s). */
        public double requiredFieldSpeedMpS;
        /** Optimal or fixed launch elevation angle above horizontal (radians). */
        public double launchAngleRad;
        /** Required field-frame robot heading to align the launcher with the lead-compensated target (radians). */
        public double requiredRobotHeadingRad;
        /** Required field-frame azimuth of the launcher barrel (radians). */
        public double requiredAimAzimuthRad;

        /** Commanded flywheel rotational speed (RPM). */
        public double requiredFlywheelRpm;
        /** Predicted RPM drop upon launching a single projectile (RPM). */
        public double predictedRpmDrop;
        /** Estimated time for the flywheel to recover after firing (seconds). */
        public double predictedRecoveryTimeSec;

        /** Predicted projectile flight time from exit to target plane (seconds). */
        public double predictedFlightTimeSec;
        /** Predicted 3D impact coordinates at the target plane/distance (m). */
        public double predictedImpactXM;
        public double predictedImpactYM;
        public double predictedImpactZM;
        /** Predicted vertical entry angle at target impact (radians, negative = descending). */
        public double impactEntryAngleRad;

        /** Estimated shot hit probability / confidence score in [0.0, 1.0]. */
        public double shotConfidence;

        public void reset() {
            isAchievable = false;
            infeasibilityReason = "UNINITIALIZED";
            horizontalDistanceM = 0.0;
            verticalDisplacementM = 0.0;
            requiredExitVelocityMpS = 0.0;
            requiredFieldSpeedMpS = 0.0;
            launchAngleRad = 0.0;
            requiredRobotHeadingRad = 0.0;
            requiredAimAzimuthRad = 0.0;
            requiredFlywheelRpm = 0.0;
            predictedRpmDrop = 0.0;
            predictedRecoveryTimeSec = 0.0;
            predictedFlightTimeSec = 0.0;
            predictedImpactXM = 0.0;
            predictedImpactYM = 0.0;
            predictedImpactZM = 0.0;
            impactEntryAngleRad = 0.0;
            shotConfidence = 0.0;
        }
    }

    /**
     * Snapshot of all robot sensors captured atomically in a single control loop iteration.
     */
    public static final class SensorSnapshot {
        public long timestampNanos;
        public double dtSec;

        public final Pose2d odometryPose = new Pose2d();
        public final Velocity2d robotFieldVelocity = new Velocity2d();

        public boolean aprilTagDetected;
        public final Pose2d aprilTagPose = new Pose2d();
        public double aprilTagLatencySec;
        public double aprilTagAmbiguity;

        public double primaryFlywheelRpm;
        public double secondaryFlywheelRpm;
        public double flywheelAccelerationRpmPerSec;
        public double batteryVoltageV;
        public double currentHoodAngleRad;
    }

    /**
     * Complete shot-gating validation and readiness state.
     */
    public static final class ShotReadiness {
        public boolean solutionAchievable;
        public boolean flywheelAtSpeed;
        public boolean flywheelStabilized;
        public boolean headingAligned;
        public boolean hoodAligned;
        public boolean localizationConfident;
        public boolean confidenceAboveThreshold;
        public boolean firePermitted;

        public double headingErrorRad;
        public double dynamicHeadingToleranceRad;
        public double flywheelRpmError;
        public String blockingDiagnostic = "NONE";

        public void evaluateOverallReadiness() {
            this.firePermitted =
                    solutionAchievable
                            && flywheelAtSpeed
                            && flywheelStabilized
                            && headingAligned
                            && hoodAligned
                            && localizationConfident
                            && confidenceAboveThreshold;

            if (firePermitted) {
                blockingDiagnostic = "READY_TO_FIRE";
            } else if (!solutionAchievable) {
                blockingDiagnostic = "TRAJECTORY_INFEASIBLE";
            } else if (!localizationConfident) {
                blockingDiagnostic = "HIGH_LOCALIZATION_UNCERTAINTY";
            } else if (!flywheelAtSpeed) {
                blockingDiagnostic = "FLYWHEEL_RPM_ERROR";
            } else if (!flywheelStabilized) {
                blockingDiagnostic = "FLYWHEEL_UNSTABLE_ACCEL";
            } else if (!headingAligned) {
                blockingDiagnostic = "HEADING_MISALIGNED";
            } else if (!hoodAligned) {
                blockingDiagnostic = "HOOD_ANGLE_MISALIGNED";
            } else {
                blockingDiagnostic = "LOW_SHOT_CONFIDENCE";
            }
        }
    }

    /**
     * Experimental shot record used by {@link ShotCalibration} to refine residual parameters
     * outside the real-time loop.
     */
    public static final class ShotRecord {
        public double robotXM;
        public double robotYM;
        public double shootingDistanceM;
        public double targetHeightM;
        public double launchAngleRad;
        public double commandedFlywheelRpm;
        public double actualFlywheelRpm;
        public double batteryVoltageV;
        public double predictedImpactXM;
        public double predictedImpactYM;
        public double predictedImpactZM;
        public double actualImpactXM;
        public double actualImpactYM;
        public double actualImpactZM;

        public ShotRecord(
                double robotXM,
                double robotYM,
                double shootingDistanceM,
                double targetHeightM,
                double launchAngleRad,
                double commandedFlywheelRpm,
                double actualFlywheelRpm,
                double batteryVoltageV,
                double predictedImpactXM,
                double predictedImpactYM,
                double predictedImpactZM,
                double actualImpactXM,
                double actualImpactYM,
                double actualImpactZM) {
            this.robotXM = robotXM;
            this.robotYM = robotYM;
            this.shootingDistanceM = shootingDistanceM;
            this.targetHeightM = targetHeightM;
            this.launchAngleRad = launchAngleRad;
            this.commandedFlywheelRpm = commandedFlywheelRpm;
            this.actualFlywheelRpm = actualFlywheelRpm;
            this.batteryVoltageV = batteryVoltageV;
            this.predictedImpactXM = predictedImpactXM;
            this.predictedImpactYM = predictedImpactYM;
            this.predictedImpactZM = predictedImpactZM;
            this.actualImpactXM = actualImpactXM;
            this.actualImpactYM = actualImpactYM;
            this.actualImpactZM = actualImpactZM;
        }

        /** Radial 3D Euclidean error between predicted and actual impact (m). */
        public double totalImpactErrorM() {
            double dx = actualImpactXM - predictedImpactXM;
            double dy = actualImpactYM - predictedImpactYM;
            double dz = actualImpactZM - predictedImpactZM;
            return Math.sqrt(dx * dx + dy * dy + dz * dz);
        }

        /** Vertical (range/height) error at target plane: positive means shot landed high/long. */
        public double verticalImpactErrorM() {
            return actualImpactZM - predictedImpactZM;
        }
    }
}
