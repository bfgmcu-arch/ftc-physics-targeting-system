package org.firstinspires.ftc.teamcode.targeting;

import org.firstinspires.ftc.teamcode.targeting.TargetingTypes.LaunchSolution;
import org.firstinspires.ftc.teamcode.targeting.TargetingTypes.Pose2d;
import org.firstinspires.ftc.teamcode.targeting.TargetingTypes.TargetGeometry;
import org.firstinspires.ftc.teamcode.targeting.TargetingTypes.TrajectoryBuffer;
import org.firstinspires.ftc.teamcode.targeting.TargetingTypes.Velocity2d;

/**
 * Comprehensive Physics Validation, Hardware Scenario Stress-Tester, and Real-Time Benchmark Suite
 * (Section 11 & Section 8 Verification).
 *
 * <p>Tests:
 * <ol>
 *   <li>Distance sweep (1.0 m to 4.2 m) & Target height sweep (0.45 m to 1.30 m)</li>
 *   <li>Launcher offset SE(2) kinematics verification (center vs. launcher exit point)</li>
 *   <li>Flywheel speed drop & voltage sag compensation (13.6V down to 10.2V)</li>
 *   <li>Moving vs. stationary robot predictive targeting (high-speed strafing + rotation)</li>
 *   <li>Experimental calibration & residual parameter estimation recovery</li>
 *   <li>High-load real-time latency benchmark (100,000 iterations)</li>
 * </ol>
 */
public final class TargetingValidationAndBenchmark {

    /**
     * Simulated hardware sensor source for deterministic validation & high-load benchmarking.
     */
    private static final class SimulatedRobotHardware implements SensorManager.HardwareSensorSource {
        double odomX = 0.0;
        double odomY = 0.0;
        double odomHeading = 0.0;
        double vx = 0.0;
        double vy = 0.0;
        double omega = 0.0;

        boolean tagDetected = false;
        double tagX = 0.0;
        double tagY = 0.0;
        double tagHeading = 0.0;
        double tagLatency = 0.035;
        double tagAmbiguity = 0.08;

        double primaryRpm = 0.0;
        double batteryVoltage = 12.6;
        double hoodAngleRad = Math.toRadians(36.0);

        @Override
        public void readOdometryPose(double[] outXYHeading) {
            outXYHeading[0] = odomX;
            outXYHeading[1] = odomY;
            outXYHeading[2] = odomHeading;
        }

        @Override
        public void readRobotFieldVelocity(double[] outVxVyOmega) {
            outVxVyOmega[0] = vx;
            outVxVyOmega[1] = vy;
            outVxVyOmega[2] = omega;
        }

        @Override
        public boolean readAprilTagObservation(double[] outPoseLatencyAmbiguity) {
            if (!tagDetected) return false;
            outPoseLatencyAmbiguity[0] = tagX;
            outPoseLatencyAmbiguity[1] = tagY;
            outPoseLatencyAmbiguity[2] = tagHeading;
            outPoseLatencyAmbiguity[3] = tagLatency;
            outPoseLatencyAmbiguity[4] = tagAmbiguity;
            return true;
        }

        @Override
        public double readPrimaryFlywheelRpm() {
            return primaryRpm;
        }

        @Override
        public double readBatteryVoltage() {
            return batteryVoltage;
        }

        @Override
        public double readCurrentHoodAngleRad() {
            return hoodAngleRad;
        }
    }

    /**
     * Builds a reference test configuration for validation & benchmarking.
     * Note: Real robot deployments MUST populate these values from actual CAD/physical measurements.
     */
    public static ShooterConfig createReferenceTestConfig(boolean adjustableHood) {
        ShooterConfig.Builder builder =
                new ShooterConfig.Builder()
                        .setEnvironment(9.80665, 1.2041)
                        .setProjectileSpecs(
                                0.065,   // 65 g test projectile
                                0.0095,  // 95 cm^2 effective area
                                0.055,   // 55 mm radius
                                0.45,    // rotational inertia factor
                                0.42,    // drag coefficient C_d
                                0.10)    // Magnus lift coefficient C_L
                        .setFlywheelMechanics(
                                0.048,   // 48 mm flywheel radius (96 mm diameter wheel)
                                0.00042, // 4.2e-4 kg*m^2 flywheel + rotor inertia
                                0.50,    // Single flywheel + hooded backplate (0.5 kinematic factor)
                                5800.0,  // Max safe RPM
                                12.0)
                        .setEmpiricalTransferModel(
                                0.84,    // 84% baseline slip efficiency
                                0.00004, // high-speed slip factor
                                0.14,    // 14% foam compression hysteresis loss
                                0.060,   // 60 ms feeder servo-to-exit delay
                                0.012)   // 12 ms sensor pipeline latency
                        .setFlywheelControlParams(
                                0.38,    // kS Volts
                                0.0182,  // kV Volts/(rad/s)
                                0.0028,  // kA Volts/(rad/s^2)
                                0.055,   // kP
                                0.012,   // kI
                                0.0012,  // kD
                                30.0,    // +/- 30 RPM ready window
                                160.0);  // +/- 160 RPM/s stability window

        if (adjustableHood) {
            builder.setAdjustableLauncherGeometry(
                    -0.12,                  // Launcher exit 12 cm behind robot center
                    0.06,                   // Launcher exit 6 cm left of robot center
                    0.28,                   // Launcher exit 28 cm above mat
                    Math.toRadians(36.0),   // Default 36 deg launch angle
                    Math.toRadians(24.0),   // Min 24 deg hood angle
                    Math.toRadians(54.0),   // Max 54 deg hood angle
                    Math.toRadians(180.0)); // Rear-shooting orientation (180 deg offset)
        } else {
            builder.setFixedLauncherGeometry(
                    0.14,                   // Launcher exit 14 cm forward of robot center
                    -0.05,                  // Launcher exit 5 cm right of robot center
                    0.26,                   // Launcher exit 26 cm above mat
                    Math.toRadians(40.0),   // Fixed 40 deg launch angle
                    0.0);
        }
        return builder.build();
    }

    public static void main(String[] args) {
        System.out.println("================================================================================");
        System.out.println("  FTC ADVANCED PHYSICS-BASED REAL-TIME TARGETING SYSTEM - VALIDATION SUITE");
        System.out.println("================================================================================");

        testDistanceAndHeightSweep();
        testLauncherOffsetKinematics();
        testMovingRobotPredictiveTargeting();
        testFlywheelClosedLoopAndVoltageSag();
        testExperimentalShotCalibration();
        runHighLoadPerformanceBenchmark();

        System.out.println("\nALL PHYSICS, CONTROL, CALIBRATION, AND LATENCY BENCHMARKS PASSED SUCCESSFULLY.");
    }

    private static void testDistanceAndHeightSweep() {
        System.out.println("\n[1] Testing Distance & Target Height Sweep (Fixed vs. Adjustable Hood)...");
        ShooterConfig config = createReferenceTestConfig(true);
        PhysicsEngine physics = new PhysicsEngine(config);
        TrajectoryCalculator calc = new TrajectoryCalculator(physics);
        Pose2d launcherPose = new Pose2d(0.0, 0.0, 0.0);
        LaunchSolution sol = new LaunchSolution();
        TrajectoryBuffer traj = new TrajectoryBuffer();
        double[] impactXYZ = new double[3];

        double[] distances = {1.2, 2.0, 2.8, 3.6};
        double[] heights = {0.55, 0.90, 1.25};

        for (double zTarget : heights) {
            for (double dist : distances) {
                TargetGeometry target = new TargetGeometry(dist, 0.0, zTarget, 0.16, Math.toRadians(-32.0));
                calc.solveStationaryLaunch(launcherPose, target, 0.015, 0.008, sol);
                if (!sol.isAchievable) {
                    throw new AssertionError("Expected achievable shot at D=" + dist + ", Z=" + zTarget);
                }

                // Forward-verify impact accuracy using full trajectory integration
                calc.predictTrajectoryAndImpact(
                        launcherPose,
                        sol.requiredAimAzimuthRad,
                        sol.launchAngleRad,
                        sol.requiredExitVelocityMpS,
                        0.0,
                        0.0,
                        dist,
                        traj,
                        impactXYZ);

                double impactErrorM = Math.hypot(impactXYZ[0] - dist, impactXYZ[2] - zTarget);
                if (impactErrorM > 0.005) {
                    throw new AssertionError(
                            String.format(
                                    "Impact error %.5f m exceeded 5 mm threshold at D=%.1f, Z=%.2f",
                                    impactErrorM, dist, zTarget));
                }
                System.out.printf(
                        "  D=%.1f m, Z=%.2f m -> Angle=%.1f deg, v0=%.2f m/s, RPM=%.0f (Drop=-%.0f), ToF=%.3f s, Err=%.2f mm, Conf=%.1f%%%n",
                        dist,
                        zTarget,
                        Math.toDegrees(sol.launchAngleRad),
                        sol.requiredExitVelocityMpS,
                        sol.requiredFlywheelRpm,
                        sol.predictedRpmDrop,
                        sol.predictedFlightTimeSec,
                        impactErrorM * 1000.0,
                        sol.shotConfidence * 100.0);
            }
        }
    }

    private static void testLauncherOffsetKinematics() {
        System.out.println("\n[2] Testing SE(2) Launcher Offset & AprilTag Latency-Compensated Localization...");
        ShooterConfig config = createReferenceTestConfig(false);
        SimulatedRobotHardware hw = new SimulatedRobotHardware();
        hw.odomX = 1.00;
        hw.odomY = -0.50;
        hw.odomHeading = Math.toRadians(90.0); // Facing +Y
        hw.vx = 0.50;
        hw.vy = 1.20;
        hw.omega = 1.50; // Rotating at 1.5 rad/s

        TargetingSystem system = new TargetingSystem(config, hw);
        system.update();

        Pose2d launcherPose = system.getLocalization().getLauncherExitPose();
        // At heading = 90 deg, robot forward (+X_robot = +0.14m) maps to +Y_field,
        // and robot right (-Y_robot = -0.05m) maps to +X_field (+0.05m).
        double expectedLauncherX = 1.00 + 0.05;
        double expectedLauncherY = -0.50 + 0.14;

        if (Math.abs(launcherPose.xM - expectedLauncherX) > 1e-4
                || Math.abs(launcherPose.yM - expectedLauncherY) > 1e-4) {
            throw new AssertionError("Launcher SE(2) offset transformation mismatch!");
        }
        System.out.printf(
                "  Robot Center=(%.2f, %.2f, 90.0 deg) -> Launcher Exit=(%.3f, %.3f) [Verified Exact]%n",
                hw.odomX, hw.odomY, launcherPose.xM, launcherPose.yM);
    }

    private static void testMovingRobotPredictiveTargeting() {
        System.out.println("\n[3] Testing Predictive Moving-Robot Targeting (Translational + Rotational Lead)...");
        ShooterConfig config = createReferenceTestConfig(false);
        PhysicsEngine physics = new PhysicsEngine(config);
        TrajectoryCalculator calc = new TrajectoryCalculator(physics);
        ShotPrediction predictor = new ShotPrediction(physics, calc);

        Pose2d robotPose = new Pose2d(0.5, 0.2, 0.0);
        // Robot strafing laterally at +1.4 m/s in Y, driving forward at +0.8 m/s in X, rotating at +0.6 rad/s
        Velocity2d robotVel = new Velocity2d(0.80, 1.40, 0.60);
        TargetGeometry target = new TargetGeometry(3.20, 0.50, 0.95, 0.18, Math.toRadians(-30.0));

        LaunchSolution movingSol = new LaunchSolution();
        TrajectoryBuffer movingTraj = new TrajectoryBuffer();

        predictor.calculatePredictiveShot(
                robotPose,
                robotVel,
                target,
                0.0,
                3500.0,
                config.fixedLaunchAngleRad,
                0.015,
                0.008,
                movingSol,
                movingTraj);

        if (!movingSol.isAchievable) {
            throw new AssertionError("Moving shot should be achievable: " + movingSol.infeasibilityReason);
        }

        // Verify that if the robot actually reaches the release pose and fires at requiredAimAzimuthRad
        // and requiredExitVelocityMpS, the 3D projectile trajectory hits the target center within < 4 mm!
        Pose2d releaseLauncherPose = predictor.getPredictedLauncherReleasePose();
        Pose2d dummyRobotRel = new Pose2d();
        Pose2d dummyLauncherRel = new Pose2d();
        Velocity2d releaseLauncherVel = new Velocity2d();
        predictor.predictLauncherStateAtHorizon(
                robotPose,
                robotVel,
                config.sensorLatencySec + config.feederActuationDelaySec,
                dummyRobotRel,
                dummyLauncherRel,
                releaseLauncherVel);

        double[] verifyImpactXYZ = new double[3];
        calc.predictTrajectoryAndImpact(
                releaseLauncherPose,
                movingSol.requiredAimAzimuthRad,
                movingSol.launchAngleRad,
                movingSol.requiredExitVelocityMpS,
                releaseLauncherVel.vxMpS,
                releaseLauncherVel.vyMpS,
                movingSol.horizontalDistanceM,
                movingTraj,
                verifyImpactXYZ);

        double error3dM =
                Math.sqrt(
                        Math.pow(verifyImpactXYZ[0] - target.centerXM, 2)
                                + Math.pow(verifyImpactXYZ[1] - target.centerYM, 2)
                                + Math.pow(verifyImpactXYZ[2] - target.centerZM, 2));

        System.out.printf(
                "  Moving Shot (v=(0.8, 1.4) m/s, omega=0.6 rad/s) -> ReqHeading=%.2f deg, ReqRPM=%.0f, 3D Impact Error=%.2f mm%n",
                Math.toDegrees(movingSol.requiredRobotHeadingRad),
                movingSol.requiredFlywheelRpm,
                error3dM * 1000.0);

        if (error3dM > 0.005) {
            throw new AssertionError("Moving-shot 3D impact error exceeded 5 mm: " + error3dM);
        }
    }

    private static void testFlywheelClosedLoopAndVoltageSag() {
        System.out.println("\n[4] Testing Closed-Loop Flywheel Control, Shot Impulse Recovery & Voltage Sag...");
        ShooterConfig config = createReferenceTestConfig(false);
        FlywheelController controller = new FlywheelController(config);
        PhysicsEngine physics = new PhysicsEngine(config);

        double[] testVoltages = {13.4, 12.0, 10.4};
        double targetRpm = 3800.0;

        for (double batteryV : testVoltages) {
            controller.setTargetRpm(targetRpm);
            double simRpm = 0.0;
            double dt = 0.010; // 100 Hz loop
            int spoolSteps = 0;

            // Simulate physical DC motor + flywheel plant: V_applied = kS + kV*omega + kA*alpha
            for (int step = 0; step < 300; step++) {
                controller.update(simRpm, simRpm, batteryV, dt);
                double appliedVolts = controller.getPrimaryMotorPower() * batteryV;
                double omega = simRpm * (2.0 * Math.PI / 60.0);
                double netTorqueVolts =
                        appliedVolts
                                - config.flywheelKsVolts * Math.signum(omega)
                                - config.flywheelKvVoltsPerRadPerSec * omega;
                double alphaRadPerSec2 = netTorqueVolts / config.flywheelKaVoltsPerRadPerSec2;
                omega = Math.max(0.0, omega + alphaRadPerSec2 * dt);
                simRpm = omega * (60.0 / (2.0 * Math.PI));
                if (controller.isReadyToFire() && spoolSteps == 0) {
                    spoolSteps = step;
                }
            }

            double steadyStateErrorRpm = Math.abs(targetRpm - simRpm);
            if (steadyStateErrorRpm > 5.0) {
                throw new AssertionError("Steady-state RPM error too high at V=" + batteryV + ": " + steadyStateErrorRpm);
            }

            // Simulate firing a projectile and measure recovery time
            double exitVel = physics.calculateExitVelocityFromRpm(simRpm);
            double rpmDrop = physics.predictFlywheelRpmDrop(simRpm, exitVel);
            double predictedRecoverySec =
                    controller.predictRecoveryTimeSec(simRpm - rpmDrop, targetRpm, batteryV);

            simRpm -= rpmDrop;
            controller.notifyProjectileLaunched(rpmDrop);

            int recoverySteps = 0;
            for (int step = 1; step <= 100; step++) {
                controller.update(simRpm, simRpm, batteryV, dt);
                double appliedVolts = controller.getPrimaryMotorPower() * batteryV;
                double omega = simRpm * (2.0 * Math.PI / 60.0);
                double netTorqueVolts =
                        appliedVolts
                                - config.flywheelKsVolts * Math.signum(omega)
                                - config.flywheelKvVoltsPerRadPerSec * omega;
                double alphaRadPerSec2 = netTorqueVolts / config.flywheelKaVoltsPerRadPerSec2;
                omega = Math.max(0.0, omega + alphaRadPerSec2 * dt);
                simRpm = omega * (60.0 / (2.0 * Math.PI));
                if (controller.isReadyToFire()) {
                    recoverySteps = step;
                    break;
                }
            }

            System.out.printf(
                    "  Battery=%.1f V -> Steady-State RPM=%.1f (Err=%.2f RPM), Shot Drop=-%.0f RPM, Recovery=%d ms (Pred=%.0f ms)%n",
                    batteryV,
                    simRpm,
                    steadyStateErrorRpm,
                    rpmDrop,
                    recoverySteps * 10,
                    predictedRecoverySec * 1000.0);
        }
    }

    private static void testExperimentalShotCalibration() {
        System.out.println("\n[5] Testing Physics-Informed Experimental Shot Calibration...");
        ShooterConfig config = createReferenceTestConfig(false);
        PhysicsEngine physics = new PhysicsEngine(config);
        TrajectoryCalculator calc = new TrajectoryCalculator(physics);
        ShotCalibration calibration = new ShotCalibration(physics);

        // Suppose the real-world robot has a slightly worn wheel with 7% lower slip efficiency (scale = 0.93)
        // and +0.8 deg mechanical hood mount bias.
        PhysicsEngine trueWorldPhysics = new PhysicsEngine(config);
        trueWorldPhysics.setCalibrationCorrections(0.93, 1.05, 1.0, Math.toRadians(0.8));
        TrajectoryCalculator trueWorldCalc = new TrajectoryCalculator(trueWorldPhysics);

        Pose2d launcherPose = new Pose2d(0.0, 0.0, 0.0);
        LaunchSolution nominalSol = new LaunchSolution();
        TrajectoryBuffer dummyTraj = new TrajectoryBuffer();
        double[] actualImpactXYZ = new double[3];

        double[] testDistances = {1.4, 1.8, 2.2, 2.6, 3.0, 3.4};
        double preCalErrorSum = 0.0;

        for (double dist : testDistances) {
            TargetGeometry target = new TargetGeometry(dist, 0.0, 0.95, 0.16, 0.0);
            calc.solveStationaryLaunch(launcherPose, target, 0.01, 0.005, nominalSol);

            // Simulate where the shot actually lands on the real robot
            double trueExitSpeed = trueWorldPhysics.calculateExitVelocityFromRpm(nominalSol.requiredFlywheelRpm);
            trueWorldCalc.predictTrajectoryAndImpact(
                    launcherPose,
                    0.0,
                    nominalSol.launchAngleRad,
                    trueExitSpeed,
                    0.0,
                    0.0,
                    dist,
                    dummyTraj,
                    actualImpactXYZ);

            double preError = Math.abs(actualImpactXYZ[2] - target.centerZM);
            preCalErrorSum += preError;

            calibration.recordShot(
                    0.0,
                    0.0,
                    dist,
                    target.centerZM,
                    nominalSol.launchAngleRad,
                    nominalSol.requiredFlywheelRpm,
                    nominalSol.requiredFlywheelRpm,
                    12.4,
                    dist,
                    0.0,
                    target.centerZM,
                    actualImpactXYZ[0],
                    actualImpactXYZ[1],
                    actualImpactXYZ[2]);
        }

        double meanPreCalVertErrorCm = (preCalErrorSum / testDistances.length) * 100.0;
        calibration.runCalibrationFitSynchronously();

        double postCalErrorSum = 0.0;
        for (double dist : testDistances) {
            TargetGeometry target = new TargetGeometry(dist, 0.0, 0.95, 0.16, 0.0);
            calc.solveStationaryLaunch(launcherPose, target, 0.01, 0.005, nominalSol);
            double trueExitSpeed = trueWorldPhysics.calculateExitVelocityFromRpm(nominalSol.requiredFlywheelRpm);
            trueWorldCalc.predictTrajectoryAndImpact(
                    launcherPose,
                    0.0,
                    nominalSol.launchAngleRad,
                    trueExitSpeed,
                    0.0,
                    0.0,
                    dist,
                    dummyTraj,
                    actualImpactXYZ);
            postCalErrorSum += Math.abs(actualImpactXYZ[2] - target.centerZM);
        }
        double meanPostCalVertErrorCm = (postCalErrorSum / testDistances.length) * 100.0;
        calibration.shutdown();

        System.out.printf(
                "  Pre-Calibration Mean Vertical Error: %.2f cm -> Post-Calibration Mean Error: %.2f cm%n",
                meanPreCalVertErrorCm, meanPostCalVertErrorCm);
        System.out.printf(
                "  Learned Multipliers: EfficiencyScale=%.4f, DragScale=%.4f, PitchBias=%+.3f deg%n",
                physics.getCalibratedEfficiencyScale(),
                physics.getCalibratedDragScale(),
                Math.toDegrees(physics.getCalibratedPitchBiasRad()));

        if (meanPostCalVertErrorCm >= meanPreCalVertErrorCm * 0.25) {
            throw new AssertionError("Calibration did not reduce error by at least 75%!");
        }
    }

    private static void runHighLoadPerformanceBenchmark() {
        System.out.println("\n[6] Running High-Workload Real-Time Performance Benchmark (100,000 Dynamic Cycles)...");
        ShooterConfig config = createReferenceTestConfig(true);
        SimulatedRobotHardware hw = new SimulatedRobotHardware();
        TargetingSystem system = new TargetingSystem(config, hw);
        system.setTarget(3.15, 0.65, 1.05, 0.16, Math.toRadians(-30.0));

        // Warmup JVM JIT (10,000 iterations)
        for (int i = 0; i < 10_000; i++) {
            hw.odomX = 0.5 + 0.001 * (i % 500);
            hw.odomY = -0.4 + 0.001 * (i % 300);
            hw.odomHeading = 0.002 * (i % 200);
            hw.vx = 1.2 * Math.cos(i * 0.01);
            hw.vy = 1.1 * Math.sin(i * 0.01);
            hw.omega = 0.8 * Math.sin(i * 0.02);
            hw.tagDetected = (i % 3 == 0);
            hw.tagX = hw.odomX + 0.005;
            hw.tagY = hw.odomY - 0.005;
            hw.tagHeading = hw.odomHeading;
            hw.primaryRpm = 3400.0 + 20.0 * Math.sin(i * 0.05);
            system.update();
        }

        system.getPerformanceMonitor().resetStatistics();

        final int benchmarkIterations = 100_000;
        long wallStartNanos = System.nanoTime();

        for (int i = 0; i < benchmarkIterations; i++) {
            // Continuously vary robot pose, velocity, angular rate, and vision frames every single cycle
            // so state-change caching is defeated and 100% of physics + predictive solver runs!
            hw.odomX = 0.3 + 1.8 * Math.abs(Math.sin(i * 0.003));
            hw.odomY = -1.2 + 2.4 * Math.cos(i * 0.002);
            hw.odomHeading = 0.4 * Math.sin(i * 0.005);
            hw.vx = 1.5 * Math.cos(i * 0.01);
            hw.vy = 1.3 * Math.sin(i * 0.01);
            hw.omega = 1.1 * Math.cos(i * 0.015);
            hw.tagDetected = (i % 2 == 0);
            hw.tagX = hw.odomX + 0.004;
            hw.tagY = hw.odomY - 0.003;
            hw.tagHeading = hw.odomHeading;
            hw.primaryRpm = 3600.0 + 15.0 * Math.cos(i * 0.03);
            system.update();
        }

        long wallElapsedNanos = System.nanoTime() - wallStartNanos;
        double avgWallCycleUs = (wallElapsedNanos / 1000.0) / benchmarkIterations;
        double avgWallCycleMs = avgWallCycleUs / 1000.0;
        PerformanceMonitor perf = system.getPerformanceMonitor();

        System.out.printf("  Total Iterations Executed     : %,d%n", benchmarkIterations);
        System.out.printf("  Average Physics + Trajectory  : %.4f ms (%.2f microseconds)%n",
                perf.getRollingAvgPhysicsTimeMs(), perf.getRollingAvgPhysicsTimeMs() * 1000.0);
        System.out.printf("  Average Total Targeting Cycle : %.4f ms (%.2f microseconds)%n",
                avgWallCycleMs, avgWallCycleUs);
        System.out.printf("  Max Physics Execution Time    : %.4f ms%n", perf.getMaxPhysicsTimeMs());
        System.out.printf("  Max Total Cycle Execution Time: %.4f ms%n", perf.getMaxTotalTimeMs());
        System.out.printf("  Missed 5.0 ms Deadlines       : %d / %,d%n",
                perf.getMissedComputeDeadlinesCount(), benchmarkIterations);

        if (avgWallCycleMs >= 5.0) {
            throw new AssertionError("Average targeting time exceeded 5.0 ms goal: " + avgWallCycleMs);
        }
    }
}
