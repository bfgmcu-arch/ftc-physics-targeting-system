package org.firstinspires.ftc.teamcode.targeting;

import org.firstinspires.ftc.teamcode.targeting.TargetingTypes.LaunchSolution;
import org.firstinspires.ftc.teamcode.targeting.TargetingTypes.Pose2d;
import org.firstinspires.ftc.teamcode.targeting.TargetingTypes.SensorSnapshot;
import org.firstinspires.ftc.teamcode.targeting.TargetingTypes.ShotReadiness;
import org.firstinspires.ftc.teamcode.targeting.TargetingTypes.TargetGeometry;
import org.firstinspires.ftc.teamcode.targeting.TargetingTypes.TrajectoryBuffer;
import org.firstinspires.ftc.teamcode.targeting.TargetingTypes.Velocity2d;

/**
 * Top-Level Real-Time FTC Flywheel Targeting System Orchestrator.
 *
 * <p>Coordinates all 7 pipeline modules in a deterministic, zero-allocation control loop:
 * <ol>
 *   <li>Sensor Acquisition ({@link SensorManager})</li>
 *   <li>Robot Localization & Launcher Frame Transformation ({@link RobotLocalization})</li>
 *   <li>Physics & Predictive Shot Calculation ({@link PhysicsEngine}, {@link TrajectoryCalculator}, {@link ShotPrediction})</li>
 *   <li>Closed-Loop Flywheel Control ({@link FlywheelController})</li>
 *   <li>Target Alignment Control ({@link TargetAlignment})</li>
 *   <li>Shot Validation & Fire Gating ({@link ShotReadiness})</li>
 *   <li>Performance Monitoring & Telemetry ({@link PerformanceMonitor}, {@link ShotCalibration})</li>
 * </ol>
 */
public final class TargetingSystem {

    /**
     * Adapter interface for FTC SDK {@code Telemetry} or FtcDashboard telemetry packets.
     * In an FTC OpMode, simply pass {@code telemetry::addData}.
     */
    public interface TelemetrySink {
        void addData(String caption, Object value);
    }

    private final ShooterConfig config;
    private final SensorManager sensorManager;
    private final PhysicsEngine physicsEngine;
    private final TrajectoryCalculator trajectoryCalculator;
    private final RobotLocalization localization;
    private final ShotPrediction shotPrediction;
    private final FlywheelController flywheelController;
    private final TargetAlignment targetAlignment;
    private final ShotCalibration shotCalibration;
    private final LaunchEventDetector launchEventDetector;
    private final PerformanceMonitor performanceMonitor;

    // Pre-allocated state objects
    private final TargetGeometry currentTarget = new TargetGeometry();
    private final LaunchSolution currentSolution = new LaunchSolution();
    private final TrajectoryBuffer predictedTrajectory = new TrajectoryBuffer();
    private final ShotReadiness shotReadiness = new ShotReadiness();

    // State-change cache to skip redundant ballistic solver iterations when stationary
    private double cachedLauncherXM = Double.NaN;
    private double cachedLauncherYM = Double.NaN;
    private double cachedLauncherHeadingRad = Double.NaN;
    private double cachedTargetXM = Double.NaN;
    private double cachedTargetYM = Double.NaN;
    private double cachedTargetZM = Double.NaN;
    private double cachedCalibrationScale = Double.NaN;

    private double minimumConfidenceThreshold = 0.55;
    private boolean flywheelArmed = true;

    public TargetingSystem(
            ShooterConfig config,
            SensorManager.HardwareSensorSource hardwareSensorSource) {
        this.config = config;
        this.sensorManager = new SensorManager(hardwareSensorSource);
        this.physicsEngine = new PhysicsEngine(config);
        this.trajectoryCalculator = new TrajectoryCalculator(physicsEngine);
        this.localization = new RobotLocalization(config);
        this.shotPrediction = new ShotPrediction(physicsEngine, trajectoryCalculator);
        this.flywheelController = new FlywheelController(config);
        this.targetAlignment = new TargetAlignment(config);
        this.shotCalibration = new ShotCalibration(physicsEngine);
        this.launchEventDetector = new LaunchEventDetector(config);
        this.performanceMonitor = new PerformanceMonitor();
    }

    /** Sets the active 3D target on the field. */
    public void setTarget(
            double centerXM,
            double centerYM,
            double centerZM,
            double apertureRadiusM,
            double preferredEntryAngleRad) {
        this.currentTarget.set(
                centerXM, centerYM, centerZM, apertureRadiusM, preferredEntryAngleRad);
    }

    /** Sets the minimum acceptable shot confidence in [0.0, 1.0] required to permit firing. */
    public void setMinimumConfidenceThreshold(double threshold) {
        this.minimumConfidenceThreshold = PhysicsEngine.clamp(threshold, 0.05, 0.99);
    }

    /** Enables or disables automatic flywheel spin-up to the calculated target RPM. */
    public void setFlywheelArmed(boolean armed) {
        this.flywheelArmed = armed;
        if (!armed) {
            flywheelController.setTargetRpm(0.0);
        }
    }

    /**
     * Executes the complete 11-step real-time targeting, localization, trajectory prediction,
     * flywheel control, alignment, and readiness pipeline.
     *
     * <p>Executes in ~5-15 microseconds with zero heap allocations.
     */
    public void update() {
        performanceMonitor.beginCycle();

        // Steps 1 & 2: Read robot current position, heading, velocity, vision, and flywheel encoders
        SensorSnapshot sensors = sensorManager.pollSensors();
        performanceMonitor.recordSensorAcquisitionComplete();

        // Step 3 & 4: Update fused localization and transform to physical launcher exit point
        localization.update(sensors);
        Pose2d robotPose = localization.getRobotPose();
        Velocity2d robotVel = localization.getRobotVelocity();
        Pose2d launcherPose = localization.getLauncherExitPose();
        Velocity2d launcherVel = localization.getLauncherExitVelocity();
        performanceMonitor.recordLocalizationComplete();

        // Step 5, 7, 8: Calculate required launch parameters, trajectory, and impact location
        // Check if robot is completely stationary and pose/target/calibration haven't changed
        boolean isStationary =
                (launcherVel.vxMpS * launcherVel.vxMpS + launcherVel.vyMpS * launcherVel.vyMpS) < 1e-6
                        && Math.abs(launcherVel.omegaRadPerSec) < 1e-4;
        boolean stateUnchanged =
                isStationary
                        && Math.abs(launcherPose.xM - cachedLauncherXM) < 1e-4
                        && Math.abs(launcherPose.yM - cachedLauncherYM) < 1e-4
                        && Math.abs(launcherPose.headingRad - cachedLauncherHeadingRad) < 1e-4
                        && Math.abs(currentTarget.centerXM - cachedTargetXM) < 1e-4
                        && Math.abs(currentTarget.centerYM - cachedTargetYM) < 1e-4
                        && Math.abs(currentTarget.centerZM - cachedTargetZM) < 1e-4
                        && Math.abs(physicsEngine.getCalibratedEfficiencyScale() - cachedCalibrationScale) < 1e-6;

        if (!stateUnchanged) {
            double estSpoolTimeSec =
                    flywheelController.predictRecoveryTimeSec(
                            flywheelController.getFilteredRpm(),
                            currentSolution.requiredFlywheelRpm > 100.0
                                    ? currentSolution.requiredFlywheelRpm
                                    : 3000.0,
                            sensors.batteryVoltageV);

            shotPrediction.calculatePredictiveShot(
                    robotPose,
                    robotVel,
                    currentTarget,
                    estSpoolTimeSec,
                    flywheelController.getFilteredRpm(),
                    sensors.currentHoodAngleRad,
                    localization.getPositionStdDevM(),
                    localization.getHeadingStdDevRad(),
                    currentSolution,
                    predictedTrajectory);

            if (isStationary) {
                cachedLauncherXM = launcherPose.xM;
                cachedLauncherYM = launcherPose.yM;
                cachedLauncherHeadingRad = launcherPose.headingRad;
                cachedTargetXM = currentTarget.centerXM;
                cachedTargetYM = currentTarget.centerYM;
                cachedTargetZM = currentTarget.centerZM;
                cachedCalibrationScale = physicsEngine.getCalibratedEfficiencyScale();
            } else {
                cachedLauncherXM = Double.NaN;
            }
        }

        // Update analytical flywheel recovery time for the solved target RPM
        if (currentSolution.isAchievable) {
            currentSolution.predictedRecoveryTimeSec =
                    flywheelController.predictRecoveryTimeSec(
                            flywheelController.getFilteredRpm() - currentSolution.predictedRpmDrop,
                            currentSolution.requiredFlywheelRpm,
                            sensors.batteryVoltageV);
        }
        performanceMonitor.recordPhysicsAndTrajectoryComplete();

        // Step 6 & 10: Set flywheel target RPM and execute closed-loop control
        if (flywheelArmed && currentSolution.isAchievable) {
            flywheelController.setTargetRpm(currentSolution.requiredFlywheelRpm);
        } else if (!flywheelArmed) {
            flywheelController.setTargetRpm(0.0);
        }
        flywheelController.update(
                sensors.primaryFlywheelRpm,
                sensors.secondaryFlywheelRpm,
                sensors.batteryVoltageV,
                sensors.dtSec);
        launchEventDetector.update(
                flywheelController.getFilteredRpm(),
                flywheelController.getFilteredAccelRpmPerSec(),
                sensors.dtSec);
        performanceMonitor.recordFlywheelControlComplete();

        // Step 9: Determine whether the robot and hood are properly aligned
        targetAlignment.update(
                robotPose,
                launcherPose,
                launcherVel,
                currentTarget,
                currentSolution,
                sensors.currentHoodAngleRad);

        // Step 11: Evaluate all shot gating conditions
        shotReadiness.solutionAchievable = currentSolution.isAchievable;
        shotReadiness.flywheelAtSpeed = flywheelController.isAtSpeed();
        shotReadiness.flywheelStabilized = flywheelController.isStabilized();
        shotReadiness.headingAligned = targetAlignment.isHeadingAligned();
        shotReadiness.hoodAligned = targetAlignment.isHoodAligned();
        shotReadiness.localizationConfident = localization.isLocalizationConfident();
        shotReadiness.confidenceAboveThreshold =
                currentSolution.shotConfidence >= minimumConfidenceThreshold;
        shotReadiness.headingErrorRad = targetAlignment.getHeadingErrorRad();
        shotReadiness.dynamicHeadingToleranceRad = targetAlignment.getDynamicHeadingToleranceRad();
        shotReadiness.flywheelRpmError = flywheelController.getRpmError();
        shotReadiness.evaluateOverallReadiness();

        performanceMonitor.endCycle();
    }

    /**
     * Publishes comprehensive driver and engineering telemetry to an FTC Telemetry / Dashboard sink.
     * Call this at a decimated rate (e.g., 10-20 Hz) in the OpMode so telemetry formatting never
     * slows down the high-speed control loop.
     */
    public void publishTelemetry(TelemetrySink telemetry) {
        Pose2d robot = localization.getRobotPose();
        Pose2d launcher = localization.getLauncherExitPose();

        telemetry.addData("01. Shot Status", shotReadiness.blockingDiagnostic);
        telemetry.addData("02. Fire Permitted", shotReadiness.firePermitted);
        telemetry.addData(
                "03. Robot Pose (m, deg)",
                String.format(
                        "(%.3f, %.3f, %.1f deg)",
                        robot.xM, robot.yM, Math.toDegrees(robot.headingRad)));
        telemetry.addData(
                "04. Launcher Exit Pose (m)",
                String.format("(%.3f, %.3f, z=%.3f)", launcher.xM, launcher.yM, config.launcherExitHeightM));
        telemetry.addData(
                "05. Target Coords (m)",
                String.format(
                        "(%.3f, %.3f, z=%.3f)",
                        currentTarget.centerXM, currentTarget.centerYM, currentTarget.centerZM));
        telemetry.addData(
                "06. Launcher-to-Target Dist (m)",
                String.format("%.3f m (dZ=%.3f m)", currentSolution.horizontalDistanceM, currentSolution.verticalDisplacementM));
        telemetry.addData(
                "07. Flywheel RPM (Req / Actual / Err)",
                String.format(
                        "%.0f / %.0f / %+.0f RPM [%s]",
                        currentSolution.requiredFlywheelRpm,
                        flywheelController.getFilteredRpm(),
                        flywheelController.getRpmError(),
                        flywheelController.getRecoveryState().name()));
        telemetry.addData(
                "08. Launch Angle & Exit Vel",
                String.format(
                        "%.2f deg | %.2f m/s (Drop: -%.0f RPM, Recov: %.0f ms)",
                        Math.toDegrees(currentSolution.launchAngleRad),
                        currentSolution.requiredExitVelocityMpS,
                        currentSolution.predictedRpmDrop,
                        currentSolution.predictedRecoveryTimeSec * 1000.0));
        telemetry.addData(
                "09. Flight Time & Impact Entry",
                String.format(
                        "%.3f s | Entry: %.1f deg",
                        currentSolution.predictedFlightTimeSec,
                        Math.toDegrees(currentSolution.impactEntryAngleRad)));
        telemetry.addData(
                "10. Predicted Impact (m)",
                String.format(
                        "(%.3f, %.3f, z=%.3f)",
                        currentSolution.predictedImpactXM,
                        currentSolution.predictedImpactYM,
                        currentSolution.predictedImpactZM));
        telemetry.addData(
                "11. Heading Alignment Error",
                String.format(
                        "%+.2f deg (Tol: +/-%.2f deg) | TurnCmd: %+.2f",
                        targetAlignment.getHeadingErrorDeg(),
                        targetAlignment.getDynamicHeadingToleranceDeg(),
                        targetAlignment.getRecommendedTurnCommand()));
        telemetry.addData(
                "12. Shot Confidence",
                String.format("%.1f%%", currentSolution.shotConfidence * 100.0));
        telemetry.addData(
                "13. Compute Time (Phys / Total / Max)",
                String.format(
                        "%.3f ms / %.3f ms / %.3f ms (Missed >5ms: %d)",
                        performanceMonitor.getRollingAvgPhysicsTimeMs(),
                        performanceMonitor.getRollingAvgTotalTimeMs(),
                        performanceMonitor.getMaxTotalTimeMs(),
                        performanceMonitor.getMissedComputeDeadlinesCount()));
        telemetry.addData(
                "14. Loop Frequency & Calibration",
                String.format(
                        "%.1f Hz | EtaScale=%.3f, DragScale=%.3f, PitchBias=%+.2f deg (N=%d)",
                        performanceMonitor.getUpdateFrequencyHz(),
                        physicsEngine.getCalibratedEfficiencyScale(),
                        physicsEngine.getCalibratedDragScale(),
                        Math.toDegrees(physicsEngine.getCalibratedPitchBiasRad()),
                        shotCalibration.getRecordCount()));
    }

    public PhysicsEngine getPhysicsEngine() {
        return physicsEngine;
    }

    public TrajectoryCalculator getTrajectoryCalculator() {
        return trajectoryCalculator;
    }

    public RobotLocalization getLocalization() {
        return localization;
    }

    public ShotPrediction getShotPrediction() {
        return shotPrediction;
    }

    public FlywheelController getFlywheelController() {
        return flywheelController;
    }

    public TargetAlignment getTargetAlignment() {
        return targetAlignment;
    }

    public ShotCalibration getShotCalibration() {
        return shotCalibration;
    }

    public LaunchEventDetector getLaunchEventDetector() {
        return launchEventDetector;
    }

    /**
     * Notifies both the closed-loop flywheel controller and the launch event detector that the
     * feeder has been actuated to launch a projectile.
     */
    public void notifyProjectileFed() {
        flywheelController.notifyProjectileLaunched(currentSolution.predictedRpmDrop);
        launchEventDetector.notifyFeederActuated(
                flywheelController.getFilteredRpm(), currentSolution.predictedRpmDrop);
    }

    public PerformanceMonitor getPerformanceMonitor() {
        return performanceMonitor;
    }

    public LaunchSolution getCurrentSolution() {
        return currentSolution;
    }

    public TrajectoryBuffer getPredictedTrajectory() {
        return predictedTrajectory;
    }

    public ShotReadiness getShotReadiness() {
        return shotReadiness;
    }
}
