package org.firstinspires.ftc.teamcode.targeting;

/**
 * Closed-Loop Flywheel Control System with Battery Voltage Compensation, Alpha-Beta Velocity/Accel
 * Observer, Take-Back-Half (TBH) Anti-Windup Integrator, Impulse Drop Recovery Boost, and
 * Analytical Spool/Recovery Time Prediction.
 */
public final class FlywheelController {

    private static final double RPM_TO_RAD_PER_SEC = (2.0 * Math.PI) / 60.0;
    private static final double RAD_PER_SEC_TO_RPM = 60.0 / (2.0 * Math.PI);

    public enum RecoveryState {
        IDLE,
        SPOOLING_UP,
        AT_SPEED_READY,
        RECOVERING_FROM_SHOT
    }

    private final ShooterConfig config;

    // Target setpoints
    private double targetRpm = 0.0;
    private double previousTargetRpm = 0.0;
    private double differentialSpinRatio = 1.0; // For dual-wheel independent top/bottom spin control

    // Alpha-Beta filter state for noise-free RPM and angular acceleration estimation
    private double filteredRpm = 0.0;
    private double filteredAccelRpmPerSec = 0.0;
    private boolean observerInitialized = false;

    // PID + Take-Back-Half (TBH) integrator state
    private double integralVolts = 0.0;
    private double tbhLastCrossVolts = 0.0;
    private double previousErrorRadPerSec = 0.0;

    // Shot drop detection & transient impulse boost state
    private RecoveryState recoveryState = RecoveryState.IDLE;
    private double impulseBoostTimerSec = 0.0;
    private int shotsDetectedCount = 0;

    // Latest computed motor duty cycles in [-1.0, 1.0]
    private double primaryMotorPower = 0.0;
    private double secondaryMotorPower = 0.0;

    public FlywheelController(ShooterConfig config) {
        this.config = config;
    }

    /** Sets the desired primary flywheel RPM. */
    public void setTargetRpm(double targetRpm) {
        this.targetRpm = PhysicsEngine.clamp(targetRpm, 0.0, config.maxFlywheelRpm);
    }

    /**
     * Optional ratio for dual independent flywheels (e.g., top wheel = ratio * targetRpm to induce
     * custom backspin or topspin). Default is 1.0.
     */
    public void setDifferentialSpinRatio(double ratio) {
        this.differentialSpinRatio = PhysicsEngine.clamp(ratio, 0.25, 1.75);
    }

    /**
     * Notifies the controller that a projectile is being fed into the wheel so it can preemptively
     * apply an impulse-recovery feedforward boost.
     */
    public void notifyProjectileLaunched(double predictedDropRpm) {
        if (targetRpm > 100.0) {
            recoveryState = RecoveryState.RECOVERING_FROM_SHOT;
            impulseBoostTimerSec = 0.085; // 85 ms high-authority torque boost window
            shotsDetectedCount++;
        }
    }

    /**
     * Executes one iteration of the closed-loop flywheel controller. Zero heap allocations.
     *
     * @param measuredPrimaryRpm   Raw encoder RPM of primary flywheel
     * @param measuredSecondaryRpm Raw encoder RPM of secondary flywheel (or equal to primary if 1 encoder)
     * @param batteryVoltageV      Current measured robot battery voltage (Volts)
     * @param dtSec                Loop timestep (seconds)
     */
    public void update(
            double measuredPrimaryRpm,
            double measuredSecondaryRpm,
            double batteryVoltageV,
            double dtSec) {
        double dt = Math.max(1e-4, dtSec);
        double safeVoltage = Math.max(7.5, batteryVoltageV);

        // 1. Alpha-Beta Observer Update for Primary RPM & Acceleration
        if (!observerInitialized) {
            filteredRpm = measuredPrimaryRpm;
            filteredAccelRpmPerSec = 0.0;
            observerInitialized = true;
        } else {
            double predRpm = filteredRpm + filteredAccelRpmPerSec * dt;
            double residual = measuredPrimaryRpm - predRpm;
            // Alpha = 0.55, Beta = 0.18 tuned for 100-200 Hz FTC bulk reads
            filteredRpm = predRpm + 0.55 * residual;
            filteredAccelRpmPerSec = filteredAccelRpmPerSec + (0.18 / dt) * residual;
        }

        if (targetRpm <= 10.0) {
            recoveryState = RecoveryState.IDLE;
            integralVolts = 0.0;
            tbhLastCrossVolts = 0.0;
            previousErrorRadPerSec = 0.0;
            primaryMotorPower = 0.0;
            secondaryMotorPower = 0.0;
            previousTargetRpm = targetRpm;
            return;
        }

        // 2. Automatic Shot-Drop Detection via sudden deceleration + RPM dip
        double rpmError = targetRpm - filteredRpm;
        if (recoveryState == RecoveryState.AT_SPEED_READY
                && rpmError > config.flywheelReadyToleranceRpm * 1.6
                && filteredAccelRpmPerSec < -800.0) {
            recoveryState = RecoveryState.RECOVERING_FROM_SHOT;
            impulseBoostTimerSec = 0.075;
            shotsDetectedCount++;
        }

        if (impulseBoostTimerSec > 0.0) {
            impulseBoostTimerSec = Math.max(0.0, impulseBoostTimerSec - dt);
        }

        // 3. Physics-Based DC Motor Feedforward (Volts)
        double omegaTargetRadPerSec = targetRpm * RPM_TO_RAD_PER_SEC;
        double omegaMeasuredRadPerSec = filteredRpm * RPM_TO_RAD_PER_SEC;
        double errorRadPerSec = omegaTargetRadPerSec - omegaMeasuredRadPerSec;

        double targetAccelRadPerSec2 =
                ((targetRpm - previousTargetRpm) * RPM_TO_RAD_PER_SEC) / dt;
        targetAccelRadPerSec2 = PhysicsEngine.clamp(targetAccelRadPerSec2, -300.0, 300.0);
        previousTargetRpm = targetRpm;

        double vFeedforward =
                config.flywheelKsVolts * Math.signum(omegaTargetRadPerSec)
                        + config.flywheelKvVoltsPerRadPerSec * omegaTargetRadPerSec
                        + config.flywheelKaVoltsPerRadPerSec2 * targetAccelRadPerSec2;

        // 4. PID + Conditional Anti-Windup + Take-Back-Half (TBH) Zero-Crossing Control
        double vProp = config.flywheelKp * errorRadPerSec;
        double omegaAccelRadPerSec2 = filteredAccelRpmPerSec * RPM_TO_RAD_PER_SEC;
        double vDeriv = -config.flywheelKd * omegaAccelRadPerSec2;

        // Conditional integration: only integrate when close to target or when output is not saturated
        double preIntegralVolts = vFeedforward + vProp + integralVolts + vDeriv;
        boolean saturatingHigh = (preIntegralVolts >= safeVoltage) && (errorRadPerSec > 0.0);
        boolean saturatingLow = (preIntegralVolts <= 0.0) && (errorRadPerSec < 0.0);
        if (!saturatingHigh && !saturatingLow && Math.abs(rpmError) < 400.0) {
            integralVolts = PhysicsEngine.clamp(
                    integralVolts + config.flywheelKi * errorRadPerSec * dt, -2.5, 2.5);
        }

        // Take-Back-Half: whenever error crosses zero, average integral with last crossing value
        // to eliminate overshoot after a shot or setpoint step!
        if ((errorRadPerSec > 0.0 && previousErrorRadPerSec < 0.0)
                || (errorRadPerSec < 0.0 && previousErrorRadPerSec > 0.0)) {
            integralVolts = 0.5 * (integralVolts + tbhLastCrossVolts);
            tbhLastCrossVolts = integralVolts;
        }
        previousErrorRadPerSec = errorRadPerSec;

        // 5. Transient Impulse Recovery Boost during/immediately after projectile contact
        double vImpulseBoost = 0.0;
        if (impulseBoostTimerSec > 0.0 && errorRadPerSec > 0.0) {
            vImpulseBoost = Math.min(4.0, 0.22 * errorRadPerSec);
        }

        double totalCommandedVolts = vFeedforward + vProp + integralVolts + vDeriv + vImpulseBoost;
        primaryMotorPower = PhysicsEngine.clamp(totalCommandedVolts / safeVoltage, 0.0, 1.0);

        // Compute secondary wheel command (supports either identical speed or differential spin)
        if (Math.abs(differentialSpinRatio - 1.0) < 1e-4) {
            secondaryMotorPower = primaryMotorPower;
        } else {
            double secTargetOmega = omegaTargetRadPerSec * differentialSpinRatio;
            double secMeasuredOmega = measuredSecondaryRpm * RPM_TO_RAD_PER_SEC;
            double secErr = secTargetOmega - secMeasuredOmega;
            double secVolts =
                    config.flywheelKsVolts * Math.signum(secTargetOmega)
                            + config.flywheelKvVoltsPerRadPerSec * secTargetOmega
                            + config.flywheelKp * secErr;
            secondaryMotorPower = PhysicsEngine.clamp(secVolts / safeVoltage, 0.0, 1.0);
        }

        // 6. Update Readiness State Machine
        boolean withinRpmWindow = Math.abs(rpmError) <= config.flywheelReadyToleranceRpm;
        boolean stabilized = Math.abs(filteredAccelRpmPerSec) <= config.flywheelStabilityDerivativeRpmPerSec;

        if (withinRpmWindow && stabilized) {
            recoveryState = RecoveryState.AT_SPEED_READY;
        } else if (recoveryState != RecoveryState.RECOVERING_FROM_SHOT) {
            recoveryState = RecoveryState.SPOOLING_UP;
        }
    }

    /**
     * Analytically predicts the time (seconds) required for the flywheel to accelerate from its
     * current RPM to {@code desiredRpm} and settle within the stability derivative window given
     * available battery voltage {@code batteryVoltageV}, solving the motor ODE:
     * {@code V_eff = kS + kV * omega + kA * d(omega)/dt} plus observer settling time.
     */
    public double predictRecoveryTimeSec(double currentRpm, double desiredRpm, double batteryVoltageV) {
        double rpmGap = desiredRpm - currentRpm;
        if (rpmGap <= config.flywheelReadyToleranceRpm && isStabilized()) {
            return 0.0;
        }
        double safeVoltage = Math.max(8.0, batteryVoltageV);
        double omegaCurr = Math.max(0.0, currentRpm) * RPM_TO_RAD_PER_SEC;
        double omegaTarget = Math.max(omegaCurr, (desiredRpm - 0.25 * config.flywheelReadyToleranceRpm) * RPM_TO_RAD_PER_SEC);

        double vSteadyTarget =
                config.flywheelKsVolts + config.flywheelKvVoltsPerRadPerSec * (desiredRpm * RPM_TO_RAD_PER_SEC);
        // Effective closed-loop excess voltage authority (bounded by battery ceiling and proportional/boost gains)
        double maxClosedLoopBoostVolts =
                Math.min(
                        safeVoltage - vSteadyTarget,
                        ( config.flywheelKp + 0.22 ) * Math.max(10.0, rpmGap * RPM_TO_RAD_PER_SEC));
        double effectiveDriveVolts = Math.min(safeVoltage, vSteadyTarget + Math.max(0.4, maxClosedLoopBoostVolts));

        double vAvailCurr = effectiveDriveVolts - config.flywheelKsVolts - config.flywheelKvVoltsPerRadPerSec * omegaCurr;
        double vAvailTarget = effectiveDriveVolts - config.flywheelKsVolts - config.flywheelKvVoltsPerRadPerSec * omegaTarget;

        double accelTimeSec;
        if (vAvailTarget <= 0.10 || vAvailCurr <= vAvailTarget) {
            accelTimeSec = (omegaTarget - omegaCurr) * config.flywheelKaVoltsPerRadPerSec2 / 0.5;
        } else {
            double timeConstantSec = config.flywheelKaVoltsPerRadPerSec2 / config.flywheelKvVoltsPerRadPerSec;
            accelTimeSec = Math.max(0.0, timeConstantSec * Math.log(vAvailCurr / vAvailTarget));
        }

        // Add closed-loop derivative observer settling horizon (~55 ms for shot recovery)
        double observerSettlingSec = (rpmGap > config.flywheelReadyToleranceRpm) ? 0.055 : 0.015;
        return accelTimeSec + observerSettlingSec;
    }

    public boolean isAtSpeed() {
        return targetRpm > 10.0 && Math.abs(targetRpm - filteredRpm) <= config.flywheelReadyToleranceRpm;
    }

    public boolean isStabilized() {
        return Math.abs(filteredAccelRpmPerSec) <= config.flywheelStabilityDerivativeRpmPerSec;
    }

    public boolean isReadyToFire() {
        return isAtSpeed() && isStabilized();
    }

    public double getTargetRpm() {
        return targetRpm;
    }

    public double getFilteredRpm() {
        return filteredRpm;
    }

    public double getRpmError() {
        return targetRpm - filteredRpm;
    }

    public double getFilteredAccelRpmPerSec() {
        return filteredAccelRpmPerSec;
    }

    public double getPrimaryMotorPower() {
        return primaryMotorPower;
    }

    public double getSecondaryMotorPower() {
        return secondaryMotorPower;
    }

    public RecoveryState getRecoveryState() {
        return recoveryState;
    }

    public int getShotsDetectedCount() {
        return shotsDetectedCount;
    }
}
