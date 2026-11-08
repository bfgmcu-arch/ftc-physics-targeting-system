package org.firstinspires.ftc.teamcode.targeting;

import org.firstinspires.ftc.teamcode.targeting.TargetingTypes.LaunchSolution;
import org.firstinspires.ftc.teamcode.targeting.TargetingTypes.ShotReadiness;

/**
 * Reference FTC OpMode Integration Template for {@link TargetingSystem}.
 *
 * <p>Demonstrates how to wire FTC SDK hardware ({@code LynxModule} bulk reads, {@code DcMotorEx}
 * flywheel motors, {@code Servo} hood & feeder, Odometry, and AprilTag {@code VisionPortal}) into
 * {@link SensorManager.HardwareSensorSource} with zero blocking operations in the main loop.
 */
public final class ExampleFtcTargetingOpMode {

    private TargetingSystem targetingSystem;
    private int telemetryDecimator = 0;

    /**
     * Call during {@code init()} of your FTC {@code LinearOpMode} or {@code OpMode}.
     */
    public void initTargeting(
            ShooterConfig robotSpecificConfig,
            SensorManager.HardwareSensorSource hardwareBridge,
            double goalXM,
            double goalYM,
            double goalZM,
            double goalApertureRadiusM,
            double goalEntryAngleRad) {

        this.targetingSystem = new TargetingSystem(robotSpecificConfig, hardwareBridge);
        this.targetingSystem.setTarget(
                goalXM, goalYM, goalZM, goalApertureRadiusM, goalEntryAngleRad);
        this.targetingSystem.setMinimumConfidenceThreshold(0.60);
        this.targetingSystem.setFlywheelArmed(true);
    }

    /**
     * Call once per iteration inside your main {@code while (opModeIsActive())} loop.
     *
     * @param driverFireButtonPressed True when the driver holds the fire trigger
     * @param driverAutoAlignPressed  True when the driver holds the auto-aim button
     * @param manualTurnInput         Driver right-stick X turn input in [-1, 1]
     * @param actuatorOut             4-element array receiving:
     *                                [0] = primary flywheel power [0, 1]
     *                                [1] = secondary flywheel power [0, 1]
     *                                [2] = chassis turn command [-1, 1]
     *                                [3] = 1.0 if feeder should fire projectile, 0.0 otherwise
     * @param telemetrySink           FTC {@code telemetry::addData} callback (decimated to 15 Hz)
     */
    public void loopTargeting(
            boolean driverFireButtonPressed,
            boolean driverAutoAlignPressed,
            double manualTurnInput,
            double[] actuatorOut,
            TargetingSystem.TelemetrySink telemetrySink) {

        // 1. Run ultra-fast physics, localization, prediction, and flywheel control update
        targetingSystem.update();

        FlywheelController fw = targetingSystem.getFlywheelController();
        TargetAlignment align = targetingSystem.getTargetAlignment();
        ShotReadiness readiness = targetingSystem.getShotReadiness();
        LaunchSolution sol = targetingSystem.getCurrentSolution();

        // 2. Output flywheel motor powers
        actuatorOut[0] = fw.getPrimaryMotorPower();
        actuatorOut[1] = fw.getSecondaryMotorPower();

        // 3. Output chassis yaw turn command (auto-align when requested, else manual driver input)
        actuatorOut[2] =
                driverAutoAlignPressed ? align.getRecommendedTurnCommand() : manualTurnInput;

        // 4. Gate projectile feeder strictly on ShotReadiness
        if (driverFireButtonPressed && readiness.firePermitted) {
            actuatorOut[3] = 1.0; // Actuate feeder servo/indexer
            fw.notifyProjectileLaunched(sol.predictedRpmDrop);
        } else {
            actuatorOut[3] = 0.0;
        }

        // 5. Decimate telemetry updates (every 6th loop ~ 15-20 Hz) to avoid Wi-Fi packet congestion
        if (telemetrySink != null && (++telemetryDecimator % 6 == 0)) {
            targetingSystem.publishTelemetry(telemetrySink);
        }
    }

    public TargetingSystem getTargetingSystem() {
        return targetingSystem;
    }
}
