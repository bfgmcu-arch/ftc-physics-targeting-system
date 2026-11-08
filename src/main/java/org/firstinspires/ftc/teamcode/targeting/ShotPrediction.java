package org.firstinspires.ftc.teamcode.targeting;

import org.firstinspires.ftc.teamcode.targeting.TargetingTypes.LaunchSolution;
import org.firstinspires.ftc.teamcode.targeting.TargetingTypes.Pose2d;
import org.firstinspires.ftc.teamcode.targeting.TargetingTypes.TargetGeometry;
import org.firstinspires.ftc.teamcode.targeting.TargetingTypes.TrajectoryBuffer;
import org.firstinspires.ftc.teamcode.targeting.TargetingTypes.Velocity2d;

/**
 * Real-Time Predictive Targeting & Moving-Robot Ballistic Solver.
 *
 * <p>Predicts the robot and launcher pose at the exact future instant of projectile release
 * (compensating for sensor latency, feeder actuation delay, and flywheel spool time) using exact
 * SE(2) constant-curvature kinematics, then solves the coupled moving-launcher aerodynamic
 * boundary-value problem by decomposing launcher velocity into line-of-sight parallel and
 * cross-track components.
 */
public final class ShotPrediction {

    private final PhysicsEngine physics;
    private final TrajectoryCalculator trajectoryCalculator;
    private final ShooterConfig config;

    // Pre-allocated scratch objects to guarantee zero heap allocation in the control loop
    private final Pose2d predictedRobotReleasePose = new Pose2d();
    private final Pose2d predictedLauncherReleasePose = new Pose2d();
    private final Velocity2d predictedLauncherReleaseVel = new Velocity2d();
    private final double[] rk4Scratch = new double[3];
    private final double[] impactScratch = new double[3];

    public ShotPrediction(PhysicsEngine physics, TrajectoryCalculator trajectoryCalculator) {
        this.physics = physics;
        this.trajectoryCalculator = trajectoryCalculator;
        this.config = physics.getConfig();
    }

    /**
     * Computes the complete motion-compensated launch solution and predicted trajectory.
     *
     * @param currentRobotPose          Fused current robot pose in field coordinates
     * @param currentRobotVel           Filtered current robot velocity in field coordinates
     * @param target                    Target 3D specification
     * @param estimatedSpoolTimeSec     Additional delay until flywheel reaches target RPM (0 if ready)
     * @param actualFlywheelRpm         Current measured flywheel RPM (for live impact prediction)
     * @param currentHoodAngleRad       Current hood elevation angle (radians)
     * @param posStdDevM                Localization position 1-sigma uncertainty (m)
     * @param headingStdDevRad          Localization heading 1-sigma uncertainty (rad)
     * @param outSolution               Populated in-place with required launch parameters
     * @param outPredictedTrajectory    Populated in-place with 3D trajectory waypoints
     */
    public void calculatePredictiveShot(
            Pose2d currentRobotPose,
            Velocity2d currentRobotVel,
            TargetGeometry target,
            double estimatedSpoolTimeSec,
            double actualFlywheelRpm,
            double currentHoodAngleRad,
            double posStdDevM,
            double headingStdDevRad,
            LaunchSolution outSolution,
            TrajectoryBuffer outPredictedTrajectory) {

        // 1. Total pre-release actuation & sensor latency horizon
        double releaseHorizonSec =
                config.sensorLatencySec
                        + config.feederActuationDelaySec
                        + Math.max(0.0, Math.min(0.25, estimatedSpoolTimeSec));

        // 2. Predict robot & launcher exit pose/velocity at projectile release via SE(2) exponential map
        predictLauncherStateAtHorizon(
                currentRobotPose,
                currentRobotVel,
                releaseHorizonSec,
                predictedRobotReleasePose,
                predictedLauncherReleasePose,
                predictedLauncherReleaseVel);

        double speedSq =
                predictedLauncherReleaseVel.vxMpS * predictedLauncherReleaseVel.vxMpS
                        + predictedLauncherReleaseVel.vyMpS * predictedLauncherReleaseVel.vyMpS;

        // Fast path: if robot is nearly stationary (< 2 cm/s), use direct stationary solver
        if (speedSq < 0.0004 && Math.abs(predictedLauncherReleaseVel.omegaRadPerSec) < 0.01) {
            trajectoryCalculator.solveStationaryLaunch(
                    predictedLauncherReleasePose,
                    target,
                    posStdDevM,
                    headingStdDevRad,
                    outSolution);

            if (outSolution.isAchievable) {
                double liveExitSpeed =
                        actualFlywheelRpm > 100.0
                                ? physics.calculateExitVelocityFromRpm(actualFlywheelRpm)
                                : outSolution.requiredExitVelocityMpS;
                double liveElevation =
                        config.adjustableAngle ? currentHoodAngleRad : config.fixedLaunchAngleRad;
                double currentLauncherAzimuth =
                        TrajectoryCalculator.normalizeAngleRadians(
                                predictedRobotReleasePose.headingRad + config.launcherYawMountOffsetRad);

                trajectoryCalculator.predictTrajectoryAndImpact(
                        predictedLauncherReleasePose,
                        currentLauncherAzimuth,
                        liveElevation,
                        liveExitSpeed,
                        0.0,
                        0.0,
                        outSolution.horizontalDistanceM,
                        outPredictedTrajectory,
                        impactScratch);
                outSolution.predictedImpactXM = impactScratch[0];
                outSolution.predictedImpactYM = impactScratch[1];
                outSolution.predictedImpactZM = impactScratch[2];
            }
            return;
        }

        // 3. Coupled Moving-Robot Ballistic Solver
        outSolution.reset();
        double dx = target.centerXM - predictedLauncherReleasePose.xM;
        double dy = target.centerYM - predictedLauncherReleasePose.yM;
        double rangeM = Math.hypot(dx, dy);
        double deltaZM = target.centerZM - config.launcherExitHeightM;

        outSolution.horizontalDistanceM = rangeM;
        outSolution.verticalDisplacementM = deltaZM;

        if (rangeM < 0.15) {
            outSolution.isAchievable = false;
            outSolution.infeasibilityReason = "TARGET_TOO_CLOSE";
            return;
        }

        // Unit vectors along and perpendicular to the line-of-sight (LOS) at release
        double uLosX = dx / rangeM;
        double uLosY = dy / rangeM;
        double uPerpX = -uLosY;
        double uPerpY = uLosX;

        // Decompose launcher field velocity into LOS-parallel and LOS-perpendicular components
        double vParallel =
                predictedLauncherReleaseVel.vxMpS * uLosX + predictedLauncherReleaseVel.vyMpS * uLosY;
        double vPerp =
                predictedLauncherReleaseVel.vxMpS * uPerpX + predictedLauncherReleaseVel.vyMpS * uPerpY;

        double launchAngleRad =
                trajectoryCalculator.selectOptimalLaunchAngle(
                        rangeM, deltaZM, target.preferredEntryAngleRad);
        outSolution.launchAngleRad = launchAngleRad;

        double effectiveElevation = launchAngleRad + physics.getCalibratedPitchBiasRad();
        double cosTheta = Math.cos(effectiveElevation);
        double sinTheta = Math.sin(effectiveElevation);

        // Obtain baseline stationary speed as initial guess
        if (!physics.solveRequiredVelocityAndFlightTime(rangeM, deltaZM, launchAngleRad, rk4Scratch)) {
            outSolution.isAchievable = false;
            outSolution.infeasibilityReason = "GEOMETRICALLY_UNREACHABLE_AT_ANGLE";
            return;
        }

        double stationaryFieldSpeed = rk4Scratch[0];
        // Initial guess for launcher-relative speed v0 accounting for parallel chassis motion
        double minRelSpeedForCrossTrack = (Math.abs(vPerp) / Math.max(0.1, cosTheta)) + 0.5;
        double v0 = Math.max(minRelSpeedForCrossTrack, stationaryFieldSpeed - vParallel * cosTheta);

        double kd = config.ballisticDragFactorPerM * physics.getCalibratedDragScale();
        double km = config.magnusFactor * physics.getCalibratedMagnusScale();

        // 2-step secant-Newton refinement on launcher-relative exit speed v0
        // Notice: with fixed hood angle, v_field_horiz = sqrt((v0*cosTheta)^2 - vPerp^2) + vParallel,
        // and v_field_z = v0 * sinTheta.
        double err0 = evaluateMovingVerticalResidual(
                v0, cosTheta, sinTheta, vParallel, vPerp, rangeM, deltaZM, kd, km, rk4Scratch);
        if (Double.isNaN(err0)) {
            outSolution.isAchievable = false;
            outSolution.infeasibilityReason = "MOVING_SHOT_STALLED";
            return;
        }

        double v1 = Math.max(minRelSpeedForCrossTrack, v0 + 0.35);
        double err1 = evaluateMovingVerticalResidual(
                v1, cosTheta, sinTheta, vParallel, vPerp, rangeM, deltaZM, kd, km, rk4Scratch);
        if (!Double.isNaN(err1) && Math.abs(err1 - err0) > 1e-6) {
            double vNext = v0 - err0 * (v1 - v0) / (err1 - err0);
            if (vNext >= minRelSpeedForCrossTrack && vNext < 50.0) {
                v0 = vNext;
                double err2 = evaluateMovingVerticalResidual(
                        v0, cosTheta, sinTheta, vParallel, vPerp, rangeM, deltaZM, kd, km, rk4Scratch);
                if (!Double.isNaN(err2) && Math.abs(err2) > 1e-3 && Math.abs(err2 - err1) > 1e-6) {
                    double vFinal = v0 - err2 * (v0 - v1) / (err2 - err1);
                    if (vFinal >= minRelSpeedForCrossTrack && vFinal < 50.0) {
                        v0 = vFinal;
                        evaluateMovingVerticalResidual(
                                v0, cosTheta, sinTheta, vParallel, vPerp, rangeM, deltaZM, kd, km, rk4Scratch);
                    }
                }
            }
        }

        // Compute exact lead azimuth offset to cancel cross-track velocity vPerp:
        // v0 * cosTheta * sin(leadAngle) = -vPerp
        double sinLead = -vPerp / (v0 * cosTheta);
        if (Math.abs(sinLead) >= 0.98) {
            outSolution.isAchievable = false;
            outSolution.infeasibilityReason = "LATERAL_SPEED_EXCEEDS_LAUNCHER_AUTHORITY";
            return;
        }
        double leadAngleRad = Math.asin(sinLead);
        double losAzimuthRad = Math.atan2(dy, dx);
        double requiredAimAzimuthRad =
                TrajectoryCalculator.normalizeAngleRadians(losAzimuthRad + leadAngleRad);
        double requiredRobotHeadingRad =
                TrajectoryCalculator.normalizeAngleRadians(
                        requiredAimAzimuthRad - config.launcherYawMountOffsetRad);

        // Account for rotational rate omega during feeder actuation by adding small angular lead if needed
        double reqRpm = physics.calculateRequiredRpmFromExitVelocity(v0);
        if (reqRpm > config.maxFlywheelRpm || Double.isNaN(reqRpm)) {
            outSolution.isAchievable = false;
            outSolution.infeasibilityReason = "EXCEEDS_MAX_FLYWHEEL_RPM";
            outSolution.requiredExitVelocityMpS = v0;
            outSolution.requiredFlywheelRpm = reqRpm;
            return;
        }

        double vHorizAlongLos =
                Math.sqrt(Math.max(0.0, (v0 * cosTheta) * (v0 * cosTheta) - vPerp * vPerp)) + vParallel;
        double vFieldTotal = Math.hypot(vHorizAlongLos, v0 * sinTheta);

        outSolution.isAchievable = true;
        outSolution.infeasibilityReason = "OK";
        outSolution.requiredExitVelocityMpS = v0;
        outSolution.requiredFieldSpeedMpS = vFieldTotal;
        outSolution.requiredAimAzimuthRad = requiredAimAzimuthRad;
        outSolution.requiredRobotHeadingRad = requiredRobotHeadingRad;
        outSolution.requiredFlywheelRpm = reqRpm;
        outSolution.predictedFlightTimeSec = rk4Scratch[1];
        outSolution.impactEntryAngleRad = rk4Scratch[2];
        outSolution.predictedRpmDrop = physics.predictFlywheelRpmDrop(reqRpm, v0);

        // Predict actual trajectory & 3D impact point based on current robot heading & flywheel RPM
        double liveExitSpeed =
                actualFlywheelRpm > 100.0
                        ? physics.calculateExitVelocityFromRpm(actualFlywheelRpm)
                        : v0;
        double liveElevation =
                config.adjustableAngle ? currentHoodAngleRad : config.fixedLaunchAngleRad;
        double currentLauncherAzimuth =
                TrajectoryCalculator.normalizeAngleRadians(
                        predictedRobotReleasePose.headingRad + config.launcherYawMountOffsetRad);

        trajectoryCalculator.predictTrajectoryAndImpact(
                predictedLauncherReleasePose,
                currentLauncherAzimuth,
                liveElevation,
                liveExitSpeed,
                predictedLauncherReleaseVel.vxMpS,
                predictedLauncherReleaseVel.vyMpS,
                rangeM,
                outPredictedTrajectory,
                impactScratch);

        outSolution.predictedImpactXM = impactScratch[0];
        outSolution.predictedImpactYM = impactScratch[1];
        outSolution.predictedImpactZM = impactScratch[2];

        // Inflate effective position uncertainty slightly proportional to robot speed & latency
        double motionInducedPosSigma =
                Math.hypot(
                        posStdDevM,
                        0.015 * Math.hypot(currentRobotVel.vxMpS, currentRobotVel.vyMpS));
        double motionInducedHeadingSigma =
                Math.hypot(headingStdDevRad, 0.015 * Math.abs(currentRobotVel.omegaRadPerSec));

        outSolution.shotConfidence =
                trajectoryCalculator.estimateShotConfidence(
                        rangeM,
                        deltaZM,
                        v0,
                        launchAngleRad,
                        outSolution.impactEntryAngleRad,
                        reqRpm,
                        target.apertureRadiusM,
                        motionInducedPosSigma,
                        motionInducedHeadingSigma);
    }

    /**
     * Evaluates the vertical residual z(D) - deltaZ for a projectile launched with launcher-relative
     * speed {@code v0} from a robot moving with line-of-sight parallel speed {@code vParallel} and
     * perpendicular speed {@code vPerp}.
     */
    private double evaluateMovingVerticalResidual(
            double v0,
            double cosTheta,
            double sinTheta,
            double vParallel,
            double vPerp,
            double rangeM,
            double deltaZM,
            double kd,
            double km,
            double[] outResult) {
        double vRelHoriz = v0 * cosTheta;
        if (vRelHoriz <= Math.abs(vPerp) + 0.1) {
            return Double.NaN;
        }
        double vFieldHoriz = Math.sqrt(vRelHoriz * vRelHoriz - vPerp * vPerp) + vParallel;
        if (vFieldHoriz <= 0.3) {
            return Double.NaN;
        }
        double vFieldZ = v0 * sinTheta;
        double effectiveFieldSpeed = Math.hypot(vFieldHoriz, vFieldZ);
        double effectiveFieldAngle = Math.atan2(vFieldZ, vFieldHoriz);

        return physics.evaluateVerticalResidualAtRange(
                effectiveFieldSpeed, effectiveFieldAngle, rangeM, deltaZM, kd, km, outResult);
    }

    /**
     * Predicts the robot center pose and launcher exit pose/velocity after {@code dtSec} using exact
     * SE(2) constant-curvature integration.
     */
    public void predictLauncherStateAtHorizon(
            Pose2d robotPose,
            Velocity2d robotVel,
            double dtSec,
            Pose2d outPredRobotPose,
            Pose2d outPredLauncherPose,
            Velocity2d outPredLauncherVel) {
        double vx = robotVel.vxMpS;
        double vy = robotVel.vyMpS;
        double omega = robotVel.omegaRadPerSec;

        double dx;
        double dy;
        double dTheta = omega * dtSec;

        if (Math.abs(omega) > 1e-4) {
            double sinWT = Math.sin(dTheta);
            double cosWT = Math.cos(dTheta);
            dx = (vx * sinWT + vy * (cosWT - 1.0)) / omega;
            dy = (-vx * (cosWT - 1.0) + vy * sinWT) / omega;
        } else {
            dx = vx * dtSec;
            dy = vy * dtSec;
        }

        double predPsi = TrajectoryCalculator.normalizeAngleRadians(robotPose.headingRad + dTheta);
        outPredRobotPose.set(robotPose.xM + dx, robotPose.yM + dy, predPsi);

        double cosPsi = Math.cos(predPsi);
        double sinPsi = Math.sin(predPsi);
        double ox = config.launcherOffsetXM;
        double oy = config.launcherOffsetYM;

        double rotOx = ox * cosPsi - oy * sinPsi;
        double rotOy = ox * sinPsi + oy * cosPsi;

        outPredLauncherPose.set(
                outPredRobotPose.xM + rotOx,
                outPredRobotPose.yM + rotOy,
                TrajectoryCalculator.normalizeAngleRadians(predPsi + config.launcherYawMountOffsetRad));

        // Rotate field translational velocity by dTheta if robot is turning in a curve, plus lever arm velocity
        double cosD = Math.cos(dTheta);
        double sinD = Math.sin(dTheta);
        double predVx = vx * cosD - vy * sinD;
        double predVy = vx * sinD + vy * cosD;

        outPredLauncherVel.set(
                predVx - omega * rotOy,
                predVy + omega * rotOx,
                omega);
    }

    public Pose2d getPredictedLauncherReleasePose() {
        return predictedLauncherReleasePose;
    }
}
