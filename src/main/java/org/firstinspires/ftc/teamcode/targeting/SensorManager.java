package org.firstinspires.ftc.teamcode.targeting;

import org.firstinspires.ftc.teamcode.targeting.TargetingTypes.SensorSnapshot;

/**
 * Low-Latency Sensor Acquisition & Bulk-Read Hardware Manager.
 *
 * <p>Designed around FTC Control/Expansion Hub {@code LynxModule.BulkCachingMode.MANUAL} best
 * practices: clears bulk caches once at the start of the control loop, reads all encoder/sensor
 * channels into a single pre-allocated {@link SensorSnapshot}, and avoids redundant bus queries.
 */
public final class SensorManager {

    /**
     * Hardware callback interface implemented by the team's OpMode or RobotHardware subsystem.
     * Keeps the targeting core decoupled from specific FTC SDK version imports while allowing
     * direct zero-allocation hardware reads.
     */
    public interface HardwareSensorSource {
        /** Clears LynxModule bulk caches once per loop cycle. */
        default void clearBulkCaches() {}

        /** Writes current odometry pose (xM, yM, headingRad) into the 3-element array. */
        void readOdometryPose(double[] outXYHeading);

        /** Writes current robot field velocity (vxMpS, vyMpS, omegaRadPerSec) into the 3-element array. */
        void readRobotFieldVelocity(double[] outVxVyOmega);

        /**
         * Reads latest AprilTag vision localization if available.
         *
         * @param outPoseLatencyAmbiguity 5-element array: [xM, yM, headingRad, latencySec, ambiguity]
         * @return true if a fresh valid AprilTag detection is available this cycle
         */
        boolean readAprilTagObservation(double[] outPoseLatencyAmbiguity);

        /** Returns primary flywheel rotational speed in RPM. */
        double readPrimaryFlywheelRpm();

        /** Returns secondary flywheel rotational speed in RPM (or primary RPM if single encoder). */
        default double readSecondaryFlywheelRpm() {
            return readPrimaryFlywheelRpm();
        }

        /** Returns current battery voltage in Volts. */
        double readBatteryVoltage();

        /** Returns current hood elevation angle in radians. */
        double readCurrentHoodAngleRad();
    }

    private final HardwareSensorSource hardwareSource;
    private final SensorSnapshot snapshot = new SensorSnapshot();

    // Pre-allocated scratch arrays for zero-GC hardware callbacks
    private final double[] odomScratch = new double[3];
    private final double[] velScratch = new double[3];
    private final double[] visionScratch = new double[5];

    private long lastTimestampNanos = 0L;
    private double previousFlywheelRpm = 0.0;
    private double cachedBatteryVoltageV = 12.0;
    private int voltageReadDecimator = 0;

    public SensorManager(HardwareSensorSource hardwareSource) {
        this.hardwareSource = hardwareSource;
    }

    /**
     * Performs a single atomic sensor acquisition pass and returns the updated pre-allocated
     * {@link SensorSnapshot}. Zero heap allocations.
     */
    public SensorSnapshot pollSensors() {
        long nowNanos = System.nanoTime();
        double dtSec =
                (lastTimestampNanos == 0L)
                        ? 0.010
                        : Math.max(1e-5, (nowNanos - lastTimestampNanos) * 1e-9);
        lastTimestampNanos = nowNanos;

        // 1. Trigger single USB/UART bulk read cache refresh
        hardwareSource.clearBulkCaches();

        snapshot.timestampNanos = nowNanos;
        snapshot.dtSec = dtSec;

        // 2. Read odometry pose & velocity
        hardwareSource.readOdometryPose(odomScratch);
        snapshot.odometryPose.set(odomScratch[0], odomScratch[1], odomScratch[2]);

        hardwareSource.readRobotFieldVelocity(velScratch);
        snapshot.robotFieldVelocity.set(velScratch[0], velScratch[1], velScratch[2]);

        // 3. Read non-blocking AprilTag vision buffer
        boolean tagSeen = hardwareSource.readAprilTagObservation(visionScratch);
        snapshot.aprilTagDetected = tagSeen;
        if (tagSeen) {
            snapshot.aprilTagPose.set(visionScratch[0], visionScratch[1], visionScratch[2]);
            snapshot.aprilTagLatencySec = visionScratch[3];
            snapshot.aprilTagAmbiguity = visionScratch[4];
        }

        // 4. Read flywheel encoders
        double primaryRpm = hardwareSource.readPrimaryFlywheelRpm();
        snapshot.primaryFlywheelRpm = primaryRpm;
        snapshot.secondaryFlywheelRpm = hardwareSource.readSecondaryFlywheelRpm();
        snapshot.flywheelAccelerationRpmPerSec = (primaryRpm - previousFlywheelRpm) / dtSec;
        previousFlywheelRpm = primaryRpm;

        // 5. Decimate battery voltage read (every 5 cycles) to save bus overhead if not in bulk packet
        if ((voltageReadDecimator++ % 5) == 0) {
            cachedBatteryVoltageV = hardwareSource.readBatteryVoltage();
        }
        snapshot.batteryVoltageV = cachedBatteryVoltageV;
        snapshot.currentHoodAngleRad = hardwareSource.readCurrentHoodAngleRad();

        return snapshot;
    }

    public SensorSnapshot getLatestSnapshot() {
        return snapshot;
    }
}
