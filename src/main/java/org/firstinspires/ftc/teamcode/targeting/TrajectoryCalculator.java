package org.firstinspires.ftc.teamcode.targeting;

import org.firstinspires.ftc.teamcode.targeting.TargetingTypes.LaunchSolution;
import org.firstinspires.ftc.teamcode.targeting.TargetingTypes.Pose2d;
import org.firstinspires.ftc.teamcode.targeting.TargetingTypes.TargetGeometry;
import org.firstinspires.ftc.teamcode.targeting.TargetingTypes.TrajectoryBuffer;
import org.firstinspires.ftc.teamcode.targeting.TargetingTypes.TrajectoryPoint;

/**
 * Zero-allocation 3D Trajectory Calculator and Optimal Launch Parameter Solver.
 *
 * <p>Computes optimal launch angle (if adjustable), required projectile exit speed, flywheel RPM,
 * full 3D trajectory waypoints, expected target impact coordinates, and first-order uncertainty
 * propagation for shot confidence estimation.
 */
public final class TrajectoryCalculator {

    private final PhysicsEngine physics;
    private final ShooterConfig config;
    // Pre-allocated scratch array for [0]=speedMpS, [1]=flightTimeSec, [2]=impactAngleRad
    private final double[] solverScratch = new double[3];

    public TrajectoryCalculator(PhysicsEngine physics) {
        this.physics = physics;
        this.config = physics.getConfig();
    }

    /**
     * Selects the optimal launch elevation angle (radians) for a given horizontal distance and
     * vertical displacement.
     *
     * <p>If the launcher has a fixed angle, returns {@code config.fixedLaunchAngleRad}.
     * If adjustable, computes the minimum-sensitivity / preferred-entry-angle trajectory and clamps
     * to the physical hood servo limits {@code [minLaunchAngleRad, maxLaunchAngleRad]}.
     */
    public double selectOptimalLaunchAngle(
            double horizontalDistanceM,
            double verticalDisplacementM,
            double preferredEntryAngleRad) {
        if (!config.adjustableAngle) {
            return config.fixedLaunchAngleRad;
        }
        if (horizontalDistanceM <= 0.05) {
            return config.maxLaunchAngleRad;
        }
        // Minimum-energy launch angle: pi/4 + 0.5 * atan2(dz, D)
        double minEnergyAngle = 0.25 * Math.PI + 0.5 * Math.atan2(verticalDisplacementM, horizontalDistanceM);

        double desiredAngle = minEnergyAngle;
        if (preferredEntryAngleRad < -0.05) {
            // Parabolic identity: tan(theta_launch) + tan(phi_impact) = 2 * dz / D
            double entryBasedTan = (2.0 * verticalDisplacementM / horizontalDistanceM)
                    - Math.tan(preferredEntryAngleRad);
            if (entryBasedTan > 0.1) {
                double entryBasedAngle = Math.atan(entryBasedTan);
                // Blend 65% entry-angle preference with 35% minimum-energy angle for stability
                desiredAngle = 0.65 * entryBasedAngle + 0.35 * minEnergyAngle;
            }
        }
        return PhysicsEngine.clamp(desiredAngle, config.minLaunchAngleRad, config.maxLaunchAngleRad);
    }

    /**
     * Solves the stationary-frame launch parameters from a given launcher exit pose to the target.
     * Populates {@code outSolution} in-place with zero heap allocations.
     */
    public void solveStationaryLaunch(
            Pose2d launcherExitPose,
            TargetGeometry target,
            double localizationPositionStdDevM,
            double localizationHeadingStdDevRad,
            LaunchSolution outSolution) {
        outSolution.reset();

        double dx = target.centerXM - launcherExitPose.xM;
        double dy = target.centerYM - launcherExitPose.yM;
        double horizontalDist = Math.hypot(dx, dy);
        double deltaZ = target.centerZM - config.launcherExitHeightM;

        outSolution.horizontalDistanceM = horizontalDist;
        outSolution.verticalDisplacementM = deltaZ;

        if (horizontalDist < 0.15) {
            outSolution.isAchievable = false;
            outSolution.infeasibilityReason = "TARGET_TOO_CLOSE";
            return;
        }

        double aimAzimuthRad = Math.atan2(dy, dx);
        outSolution.requiredAimAzimuthRad = aimAzimuthRad;
        outSolution.requiredRobotHeadingRad =
                normalizeAngleRadians(aimAzimuthRad - config.launcherYawMountOffsetRad);

        double launchAngleRad =
                selectOptimalLaunchAngle(horizontalDist, deltaZ, target.preferredEntryAngleRad);
        outSolution.launchAngleRad = launchAngleRad;

        boolean solvable =
                physics.solveRequiredVelocityAndFlightTime(
                        horizontalDist, deltaZ, launchAngleRad, solverScratch);
        if (!solvable) {
            outSolution.isAchievable = false;
            outSolution.infeasibilityReason = "GEOMETRICALLY_UNREACHABLE_AT_ANGLE";
            return;
        }

        double reqVelocity = solverScratch[0];
        double flightTime = solverScratch[1];
        double impactAngle = solverScratch[2];

        double reqRpm = physics.calculateRequiredRpmFromExitVelocity(reqVelocity);
        if (reqRpm > config.maxFlywheelRpm || Double.isNaN(reqRpm)) {
            outSolution.isAchievable = false;
            outSolution.infeasibilityReason = "EXCEEDS_MAX_FLYWHEEL_RPM";
            outSolution.requiredExitVelocityMpS = reqVelocity;
            outSolution.requiredFlywheelRpm = reqRpm;
            return;
        }

        outSolution.isAchievable = true;
        outSolution.infeasibilityReason = "OK";
        outSolution.requiredExitVelocityMpS = reqVelocity;
        outSolution.requiredFieldSpeedMpS = reqVelocity;
        outSolution.requiredFlywheelRpm = reqRpm;
        outSolution.predictedFlightTimeSec = flightTime;
        outSolution.impactEntryAngleRad = impactAngle;
        outSolution.predictedRpmDrop = physics.predictFlywheelRpmDrop(reqRpm, reqVelocity);
        outSolution.predictedImpactXM = target.centerXM;
        outSolution.predictedImpactYM = target.centerYM;
        outSolution.predictedImpactZM = target.centerZM;

        outSolution.shotConfidence =
                estimateShotConfidence(
                        horizontalDist,
                        deltaZ,
                        reqVelocity,
                        launchAngleRad,
                        impactAngle,
                        reqRpm,
                        target.apertureRadiusM,
                        localizationPositionStdDevM,
                        localizationHeadingStdDevRad);
    }

    /**
     * Forward-predicts the 3D projectile trajectory and actual impact point at the target range
     * given actual/current launcher state, populates {@code outTrajectory} with 3D waypoints, and
     * writes the predicted impact coordinates into {@code outImpactXYZ} (length 3).
     */
    public void predictTrajectoryAndImpact(
            Pose2d launcherExitPose,
            double launchAzimuthRad,
            double launchElevationRad,
            double exitSpeedRelLauncherMpS,
            double launcherVxFieldMpS,
            double launcherVyFieldMpS,
            double targetRangeM,
            TrajectoryBuffer outTrajectory,
            double[] outImpactXYZ) {
        outTrajectory.clear();

        double effectiveElevation = launchElevationRad + physics.getCalibratedPitchBiasRad();
        double vHorizRel = exitSpeedRelLauncherMpS * Math.cos(effectiveElevation);
        double vz0 = exitSpeedRelLauncherMpS * Math.sin(effectiveElevation);

        double vx0 = vHorizRel * Math.cos(launchAzimuthRad) + launcherVxFieldMpS;
        double vy0 = vHorizRel * Math.sin(launchAzimuthRad) + launcherVyFieldMpS;

        double horizSpeed0 = Math.hypot(vx0, vy0);
        if (horizSpeed0 < 0.25 || targetRangeM <= 0.05) {
            outImpactXYZ[0] = launcherExitPose.xM;
            outImpactXYZ[1] = launcherExitPose.yM;
            outImpactXYZ[2] = config.launcherExitHeightM;
            return;
        }

        double dirX = vx0 / horizSpeed0;
        double dirY = vy0 / horizSpeed0;

        double kd = config.ballisticDragFactorPerM * physics.getCalibratedDragScale();
        double km = config.magnusFactor * physics.getCalibratedMagnusScale();
        double g = config.gravityMpS2;

        final int numSegments = TrajectoryBuffer.MAX_WAYPOINTS - 1;
        final double dx = targetRangeM / numSegments;

        double xDist = 0.0;
        double z = config.launcherExitHeightM;
        double t = 0.0;
        double vx = horizSpeed0;
        double vz = vz0;
        double maxZ = z;

        outTrajectory.points[0].set(0.0, launcherExitPose.xM, launcherExitPose.yM, z, vx * dirX, vx * dirY, vz);
        outTrajectory.count = 1;

        for (int i = 1; i <= numSegments; i++) {
            if (vx <= 0.25) {
                break;
            }
            double v1 = Math.hypot(vx, vz);
            double invVx1 = 1.0 / vx;
            double k1z = vz * invVx1;
            double k1t = invVx1;
            double k1vx = (-kd * v1 * vx - km * v1 * vz) * invVx1;
            double k1vz = (-g - kd * v1 * vz + km * v1 * vx) * invVx1;

            double vxMid = Math.max(0.25, vx + 0.5 * dx * k1vx);
            double vzMid = vz + 0.5 * dx * k1vz;
            double v2 = Math.hypot(vxMid, vzMid);
            double invVx2 = 1.0 / vxMid;
            double k2z = vzMid * invVx2;
            double k2t = invVx2;
            double k2vx = (-kd * v2 * vxMid - km * v2 * vzMid) * invVx2;
            double k2vz = (-g - kd * v2 * vzMid + km * v2 * vxMid) * invVx2;

            xDist += dx;
            z += dx * k2z;
            t += dx * k2t;
            vx = Math.max(0.25, vx + dx * k2vx);
            vz += dx * k2vz;

            if (z > maxZ) {
                maxZ = z;
            }

            TrajectoryPoint pt = outTrajectory.points[outTrajectory.count++];
            pt.set(
                    t,
                    launcherExitPose.xM + xDist * dirX,
                    launcherExitPose.yM + xDist * dirY,
                    z,
                    vx * dirX,
                    vx * dirY,
                    vz);
        }

        outTrajectory.apexHeightM = maxZ;
        outTrajectory.impactEntryAngleRad = Math.atan2(vz, vx);

        TrajectoryPoint last = outTrajectory.points[outTrajectory.count - 1];
        outImpactXYZ[0] = last.xM;
        outImpactXYZ[1] = last.yM;
        outImpactXYZ[2] = last.zM;
    }

    /**
     * Computes an analytical first-order uncertainty propagation score in [0.0, 1.0] representing
     * the probability that the projectile lands inside the target aperture.
     */
    public double estimateShotConfidence(
            double distanceM,
            double deltaZM,
            double exitVelocityMpS,
            double launchAngleRad,
            double impactAngleRad,
            double requiredRpm,
            double targetApertureRadiusM,
            double posStdDevM,
            double headingStdDevRad) {
        double cosTheta = Math.max(0.1, Math.cos(launchAngleRad));
        double tanTheta = Math.tan(launchAngleRad);
        double g = config.gravityMpS2;

        // Sensitivity of vertical impact height z to exit velocity v0: dz/dv0 = g * D^2 / (v0^3 * cos^2(theta))
        double dzDv0 = (g * distanceM * distanceM)
                / (exitVelocityMpS * exitVelocityMpS * exitVelocityMpS * cosTheta * cosTheta);
        // Sensitivity of vertical impact height z to distance D: dz/dD = |tan(impactAngle)|
        double dzDD = Math.abs(Math.tan(impactAngleRad));

        // Expected 1-sigma velocity variation from flywheel tolerance + feeding variance (~1.2% + tolerance ratio)
        double rpmSigma = Math.max(15.0, config.flywheelReadyToleranceRpm * 0.6);
        double velocitySigmaMpS = exitVelocityMpS * (0.012 + (rpmSigma / Math.max(500.0, requiredRpm)));

        double verticalSigmaM = Math.hypot(dzDv0 * velocitySigmaMpS, dzDD * posStdDevM);
        double lateralSigmaM = Math.hypot(posStdDevM, distanceM * Math.max(0.008, headingStdDevRad));
        double combinedRadialSigmaM = Math.hypot(verticalSigmaM, lateralSigmaM);

        double effectiveApertureM = Math.max(0.05, targetApertureRadiusM);
        // Rayleigh / Gaussian radial hit probability approximation: P(r <= R) = 1 - exp(-R^2 / (2 * sigma^2))
        double ratioSq = (effectiveApertureM * effectiveApertureM)
                / (2.0 * combinedRadialSigmaM * combinedRadialSigmaM + 1e-6);
        double hitProb = 1.0 - Math.exp(-ratioSq);

        // Penalize shots operating near the absolute RPM ceiling (>92% max RPM)
        double rpmMarginFactor = 1.0;
        if (requiredRpm > 0.92 * config.maxFlywheelRpm) {
            rpmMarginFactor = PhysicsEngine.clamp(
                    (config.maxFlywheelRpm - requiredRpm) / (0.08 * config.maxFlywheelRpm), 0.2, 1.0);
        }

        return PhysicsEngine.clamp(hitProb * rpmMarginFactor, 0.0, 1.0);
    }

    public static double normalizeAngleRadians(double angleRad) {
        double wrapped = (angleRad + Math.PI) % (2.0 * Math.PI);
        if (wrapped < 0.0) {
            wrapped += 2.0 * Math.PI;
        }
        return wrapped - Math.PI;
    }
}
