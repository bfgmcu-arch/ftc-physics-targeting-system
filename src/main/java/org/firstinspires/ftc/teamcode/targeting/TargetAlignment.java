package org.firstinspires.ftc.teamcode.targeting;

import org.firstinspires.ftc.teamcode.targeting.TargetingTypes.LaunchSolution;
import org.firstinspires.ftc.teamcode.targeting.TargetingTypes.Pose2d;
import org.firstinspires.ftc.teamcode.targeting.TargetingTypes.TargetGeometry;
import org.firstinspires.ftc.teamcode.targeting.TargetingTypes.Velocity2d;

/**
 * Target Alignment & Dynamic Angular Gating Controller.
 *
 * <p>Computes closed-loop chassis/turret rotational commands with line-of-sight angular rate
 * feedforward, hood pitch servo setpoints, and distance-adaptive angular alignment tolerances.
 */
public final class TargetAlignment {

    private final ShooterConfig config;

    // Heading PD + Feedforward gains (rad/s command or normalized turn power)
    private double kPHeading = 5.2;
    private double kDHeading = 0.32;
    private double kStaticTurn = 0.04;
    private double maxTurnCommand = 1.0;

    // Latest alignment state
    private double headingErrorRad = 0.0;
    private double dynamicHeadingToleranceRad = 0.035;
    private double angularFeedforwardRadPerSec = 0.0;
    private double recommendedTurnCommand = 0.0;
    private double hoodErrorRad = 0.0;
    private boolean headingAligned = false;
    private boolean hoodAligned = true;

    public TargetAlignment(ShooterConfig config) {
        this.config = config;
    }

    public void setHeadingControlGains(double kP, double kD, double kStatic, double maxCommand) {
        this.kPHeading = kP;
        this.kDHeading = kD;
        this.kStaticTurn = kStatic;
        this.maxTurnCommand = maxCommand;
    }

    /**
     * Updates target alignment errors, dynamic distance-scaled angular tolerances, and recommended
     * chassis/turret turn commands. Zero heap allocations.
     */
    public void update(
            Pose2d currentRobotPose,
            Pose2d currentLauncherPose,
            Velocity2d currentLauncherVel,
            TargetGeometry target,
            LaunchSolution solution,
            double currentHoodAngleRad) {

        if (!solution.isAchievable) {
            headingErrorRad = 0.0;
            recommendedTurnCommand = 0.0;
            headingAligned = false;
            hoodAligned = false;
            return;
        }

        // 1. Heading Error in [-pi, pi]
        headingErrorRad =
                TrajectoryCalculator.normalizeAngleRadians(
                        solution.requiredRobotHeadingRad - currentRobotPose.headingRad);

        // 2. Distance-Adaptive Angular Tolerance based on physical target aperture margin
        double usableRadiusM =
                Math.max(0.025, 0.65 * (target.apertureRadiusM - config.projectileRadiusM));
        double distM = Math.max(0.25, solution.horizontalDistanceM);
        dynamicHeadingToleranceRad =
                PhysicsEngine.clamp(Math.atan2(usableRadiusM, distM), 0.010, 0.105); // [0.57 deg, 6.0 deg]

        // 3. Line-of-Sight Angular Velocity Feedforward: omega_LOS = (r x (-v_launcher)) / |r|^2
        double dx = target.centerXM - currentLauncherPose.xM;
        double dy = target.centerYM - currentLauncherPose.yM;
        double distSq = Math.max(0.0625, dx * dx + dy * dy);
        angularFeedforwardRadPerSec =
                -(dx * currentLauncherVel.vyMpS - dy * currentLauncherVel.vxMpS) / distSq;

        // 4. PD + Feedforward Turn Command
        double errorRateRadPerSec = angularFeedforwardRadPerSec - currentLauncherVel.omegaRadPerSec;
        double rawTurn =
                kPHeading * headingErrorRad
                        + kDHeading * errorRateRadPerSec
                        + 0.18 * angularFeedforwardRadPerSec;
        if (Math.abs(headingErrorRad) > 0.005) {
            rawTurn += Math.copySign(kStaticTurn, headingErrorRad);
        }
        recommendedTurnCommand = PhysicsEngine.clamp(rawTurn, -maxTurnCommand, maxTurnCommand);

        // Require both angular position and angular velocity error to be bounded for clean release
        boolean rateSettled = Math.abs(errorRateRadPerSec) < 0.65;
        headingAligned = Math.abs(headingErrorRad) <= dynamicHeadingToleranceRad && rateSettled;

        // 5. Hood Elevation Alignment Check
        if (config.adjustableAngle) {
            hoodErrorRad = solution.launchAngleRad - currentHoodAngleRad;
            hoodAligned = Math.abs(hoodErrorRad) <= 0.015; // ~0.85 degrees
        } else {
            hoodErrorRad = 0.0;
            hoodAligned = true;
        }
    }

    public double getHeadingErrorRad() {
        return headingErrorRad;
    }

    public double getHeadingErrorDeg() {
        return Math.toDegrees(headingErrorRad);
    }

    public double getDynamicHeadingToleranceRad() {
        return dynamicHeadingToleranceRad;
    }

    public double getDynamicHeadingToleranceDeg() {
        return Math.toDegrees(dynamicHeadingToleranceRad);
    }

    public double getRecommendedTurnCommand() {
        return recommendedTurnCommand;
    }

    public double getAngularFeedforwardRadPerSec() {
        return angularFeedforwardRadPerSec;
    }

    public boolean isHeadingAligned() {
        return headingAligned;
    }

    public boolean isHoodAligned() {
        return hoodAligned;
    }
}
