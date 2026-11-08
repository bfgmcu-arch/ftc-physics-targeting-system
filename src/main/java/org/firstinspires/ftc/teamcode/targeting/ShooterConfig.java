package org.firstinspires.ftc.teamcode.targeting;

/**
 * Hardware, geometric, and physical configuration for the FTC Flywheel Targeting System.
 *
 * <p>IMPORTANT ENGINEERING NOTE:
 * Physical constants (gravity, standard air density) are defined with standard SI values.
 * Robot-specific hardware parameters and game-element specifications MUST be populated
 * with actual measurements from your CAD/robot before competition use. Use {@link Builder}
 * to construct and validate that all required measurements have been explicitly supplied.
 */
public final class ShooterConfig {

    // =========================================================================
    // 1. UNIVERSAL PHYSICAL CONSTANTS (SI Units)
    // =========================================================================
    /** Standard gravitational acceleration (m/s^2). */
    public final double gravityMpS2;
    /** Ambient air density (kg/m^3). Standard sea-level at 20C is 1.2041 kg/m^3. */
    public final double airDensityKgPerM3;

    // =========================================================================
    // 2. PROJECTILE SPECIFICATIONS (Must match current FTC season game element)
    // =========================================================================
    /** Mass of a single projectile (kg). */
    public final double projectileMassKg;
    /** Effective cross-sectional area of the projectile facing the airflow (m^2). */
    public final double projectileCrossSectionM2;
    /** Effective outer radius or characteristic half-thickness of the projectile (m). */
    public final double projectileRadiusM;
    /** Moment of inertia factor k_p such that I_p = k_p * m_p * r_p^2 (dimensionless). */
    public final double projectileInertiaFactor;
    /** Baseline aerodynamic drag coefficient C_d (dimensionless). */
    public final double baseDragCoefficient;
    /** Baseline Magnus lift coefficient C_L per unit spin ratio (dimensionless). */
    public final double baseMagnusCoefficient;

    // =========================================================================
    // 3. LAUNCHER GEOMETRY & FLYWHEEL MECHANICS (Robot-Specific Measurements)
    // =========================================================================
    /** Forward (X) offset of launcher exit point from robot localization center (m). */
    public final double launcherOffsetXM;
    /** Lateral (Y, left-positive) offset of launcher exit point from robot center (m). */
    public final double launcherOffsetYM;
    /** Height (Z) of projectile center at launcher exit above the field mat (m). */
    public final double launcherExitHeightM;
    /** Fixed launch elevation angle above horizontal (radians), or minimum angle if adjustable. */
    public final double fixedLaunchAngleRad;
    /** Minimum adjustable hood angle (radians). Equal to fixedLaunchAngleRad if fixed. */
    public final double minLaunchAngleRad;
    /** Maximum adjustable hood angle (radians). Equal to fixedLaunchAngleRad if fixed. */
    public final double maxLaunchAngleRad;
    /** Whether the launcher hood/pitch angle is dynamically adjustable via servo. */
    public final boolean adjustableAngle;
    /** Angular offset of the launcher barrel relative to robot forward heading (radians). */
    public final double launcherYawMountOffsetRad;

    /** Flywheel outer radius (m). */
    public final double flywheelRadiusM;
    /** Combined rotational moment of inertia of the flywheel assembly referred to the wheel shaft (kg*m^2). */
    public final double flywheelMomentOfInertiaKgM2;
    /**
     * Launcher configuration factor:
     * 0.5 for a single active wheel against a stationary hood/backplate (v_ideal = 0.5 * omega * R),
     * 1.0 for dual counter-rotating wheels of equal surface speed (v_ideal = 1.0 * omega * R).
     */
    public final double surfaceVelocityKinematicFactor;
    /** Maximum safe operating RPM of the flywheel assembly. */
    public final double maxFlywheelRpm;
    /** Nominal battery voltage used for motor feedforward characterization (V, typically 12.0V). */
    public final double nominalVoltageV;

    // =========================================================================
    // 4. EMPIRICAL / CALIBRATED FLYWHEEL & FEEDING PARAMETERS
    // =========================================================================
    /** Baseline energy/velocity slip transfer efficiency eta_0 in (0, 1]. */
    public final double baseTransferEfficiency;
    /** First-order high-RPM slip decay coefficient eta_1 (s/rad). */
    public final double speedDependentSlipFactor;
    /** Parasitic friction energy loss ratio during compression (dimensionless, >= 0). */
    public final double compressionEnergyLossFactor;
    /** Mechanical feeder actuation delay from fire command to projectile exit (seconds). */
    public final double feederActuationDelaySec;
    /** Sensor acquisition and bulk-read latency estimate (seconds). */
    public final double sensorLatencySec;

    // =========================================================================
    // 5. CLOSED-LOOP FLYWHEEL CONTROL GAINS & TOLERANCES
    // =========================================================================
    /** Static friction feedforward voltage kS (Volts). */
    public final double flywheelKsVolts;
    /** Velocity feedforward gain kV (Volts per rad/s). */
    public final double flywheelKvVoltsPerRadPerSec;
    /** Acceleration feedforward gain kA (Volts per rad/s^2). */
    public final double flywheelKaVoltsPerRadPerSec2;
    /** Proportional gain kP (Volts per rad/s error). */
    public final double flywheelKp;
    /** Integral / Take-Back-Half gain kI (Volts per rad error). */
    public final double flywheelKi;
    /** Derivative gain kD (Volts per rad/s^2 error). */
    public final double flywheelKd;
    /** Maximum allowable RPM error to declare the flywheel ready to fire. */
    public final double flywheelReadyToleranceRpm;
    /** Maximum allowable RPM derivative (acceleration) to declare flywheel stabilized (RPM/s). */
    public final double flywheelStabilityDerivativeRpmPerSec;

    // =========================================================================
    // 6. PRECOMPUTED DERIVED CONSTANTS (Zero-Overhead Access in Real-Time Loop)
    // =========================================================================
    /** Precomputed ballistic drag factor: k_d = (rho * C_d * A) / (2 * m_p) (1/m). */
    public final double ballisticDragFactorPerM;
    /** Precomputed Magnus lift factor: k_m = (rho * C_L * A * r_p) / (2 * m_p) (dimensionless). */
    public final double magnusFactor;
    /** Precomputed effective projectile mass ratio for flywheel kinetic energy drop. */
    public final double effectiveProjectileInertialMassKg;

    private ShooterConfig(Builder b) {
        this.gravityMpS2 = b.gravityMpS2;
        this.airDensityKgPerM3 = b.airDensityKgPerM3;

        this.projectileMassKg = b.projectileMassKg;
        this.projectileCrossSectionM2 = b.projectileCrossSectionM2;
        this.projectileRadiusM = b.projectileRadiusM;
        this.projectileInertiaFactor = b.projectileInertiaFactor;
        this.baseDragCoefficient = b.baseDragCoefficient;
        this.baseMagnusCoefficient = b.baseMagnusCoefficient;

        this.launcherOffsetXM = b.launcherOffsetXM;
        this.launcherOffsetYM = b.launcherOffsetYM;
        this.launcherExitHeightM = b.launcherExitHeightM;
        this.fixedLaunchAngleRad = b.fixedLaunchAngleRad;
        this.minLaunchAngleRad = b.minLaunchAngleRad;
        this.maxLaunchAngleRad = b.maxLaunchAngleRad;
        this.adjustableAngle = b.adjustableAngle;
        this.launcherYawMountOffsetRad = b.launcherYawMountOffsetRad;

        this.flywheelRadiusM = b.flywheelRadiusM;
        this.flywheelMomentOfInertiaKgM2 = b.flywheelMomentOfInertiaKgM2;
        this.surfaceVelocityKinematicFactor = b.surfaceVelocityKinematicFactor;
        this.maxFlywheelRpm = b.maxFlywheelRpm;
        this.nominalVoltageV = b.nominalVoltageV;

        this.baseTransferEfficiency = b.baseTransferEfficiency;
        this.speedDependentSlipFactor = b.speedDependentSlipFactor;
        this.compressionEnergyLossFactor = b.compressionEnergyLossFactor;
        this.feederActuationDelaySec = b.feederActuationDelaySec;
        this.sensorLatencySec = b.sensorLatencySec;

        this.flywheelKsVolts = b.flywheelKsVolts;
        this.flywheelKvVoltsPerRadPerSec = b.flywheelKvVoltsPerRadPerSec;
        this.flywheelKaVoltsPerRadPerSec2 = b.flywheelKaVoltsPerRadPerSec2;
        this.flywheelKp = b.flywheelKp;
        this.flywheelKi = b.flywheelKi;
        this.flywheelKd = b.flywheelKd;
        this.flywheelReadyToleranceRpm = b.flywheelReadyToleranceRpm;
        this.flywheelStabilityDerivativeRpmPerSec = b.flywheelStabilityDerivativeRpmPerSec;

        // Precompute constants once at initialization to save floating-point divisions in loop
        this.ballisticDragFactorPerM =
                (this.airDensityKgPerM3 * this.baseDragCoefficient * this.projectileCrossSectionM2)
                        / (2.0 * this.projectileMassKg);
        this.magnusFactor =
                (this.airDensityKgPerM3 * this.baseMagnusCoefficient * this.projectileCrossSectionM2 * this.projectileRadiusM)
                        / (2.0 * this.projectileMassKg);
        // Accounts for both translational KE (1/2 m v^2) and rotational KE (1/2 I_p omega_p^2) + compression loss
        this.effectiveProjectileInertialMassKg =
                this.projectileMassKg * (1.0 + this.projectileInertiaFactor + this.compressionEnergyLossFactor);
    }

    public static final class Builder {
        private double gravityMpS2 = 9.80665;
        private double airDensityKgPerM3 = 1.2041;

        private double projectileMassKg = Double.NaN;
        private double projectileCrossSectionM2 = Double.NaN;
        private double projectileRadiusM = Double.NaN;
        private double projectileInertiaFactor = 0.5;
        private double baseDragCoefficient = 0.45;
        private double baseMagnusCoefficient = 0.12;

        private double launcherOffsetXM = Double.NaN;
        private double launcherOffsetYM = Double.NaN;
        private double launcherExitHeightM = Double.NaN;
        private double fixedLaunchAngleRad = Double.NaN;
        private double minLaunchAngleRad = Double.NaN;
        private double maxLaunchAngleRad = Double.NaN;
        private boolean adjustableAngle = false;
        private double launcherYawMountOffsetRad = 0.0;

        private double flywheelRadiusM = Double.NaN;
        private double flywheelMomentOfInertiaKgM2 = Double.NaN;
        private double surfaceVelocityKinematicFactor = 0.5; // 0.5 = single wheel + hood; 1.0 = dual wheel
        private double maxFlywheelRpm = 6000.0;
        private double nominalVoltageV = 12.0;

        private double baseTransferEfficiency = 0.82;
        private double speedDependentSlipFactor = 0.00005;
        private double compressionEnergyLossFactor = 0.15;
        private double feederActuationDelaySec = 0.065;
        private double sensorLatencySec = 0.015;

        private double flywheelKsVolts = 0.35;
        private double flywheelKvVoltsPerRadPerSec = 0.0185;
        private double flywheelKaVoltsPerRadPerSec2 = 0.0025;
        private double flywheelKp = 0.045;
        private double flywheelKi = 0.010;
        private double flywheelKd = 0.001;
        private double flywheelReadyToleranceRpm = 35.0;
        private double flywheelStabilityDerivativeRpmPerSec = 180.0;

        public Builder setEnvironment(double gravityMpS2, double airDensityKgPerM3) {
            this.gravityMpS2 = gravityMpS2;
            this.airDensityKgPerM3 = airDensityKgPerM3;
            return this;
        }

        public Builder setProjectileSpecs(
                double massKg,
                double crossSectionM2,
                double radiusM,
                double inertiaFactor,
                double dragCoeff,
                double magnusCoeff) {
            this.projectileMassKg = massKg;
            this.projectileCrossSectionM2 = crossSectionM2;
            this.projectileRadiusM = radiusM;
            this.projectileInertiaFactor = inertiaFactor;
            this.baseDragCoefficient = dragCoeff;
            this.baseMagnusCoefficient = magnusCoeff;
            return this;
        }

        public Builder setFixedLauncherGeometry(
                double offsetXM,
                double offsetYM,
                double exitHeightM,
                double launchAngleRad,
                double yawMountOffsetRad) {
            this.launcherOffsetXM = offsetXM;
            this.launcherOffsetYM = offsetYM;
            this.launcherExitHeightM = exitHeightM;
            this.fixedLaunchAngleRad = launchAngleRad;
            this.minLaunchAngleRad = launchAngleRad;
            this.maxLaunchAngleRad = launchAngleRad;
            this.adjustableAngle = false;
            this.launcherYawMountOffsetRad = yawMountOffsetRad;
            return this;
        }

        public Builder setAdjustableLauncherGeometry(
                double offsetXM,
                double offsetYM,
                double exitHeightM,
                double defaultLaunchAngleRad,
                double minLaunchAngleRad,
                double maxLaunchAngleRad,
                double yawMountOffsetRad) {
            this.launcherOffsetXM = offsetXM;
            this.launcherOffsetYM = offsetYM;
            this.launcherExitHeightM = exitHeightM;
            this.fixedLaunchAngleRad = defaultLaunchAngleRad;
            this.minLaunchAngleRad = minLaunchAngleRad;
            this.maxLaunchAngleRad = maxLaunchAngleRad;
            this.adjustableAngle = true;
            this.launcherYawMountOffsetRad = yawMountOffsetRad;
            return this;
        }

        public Builder setFlywheelMechanics(
                double radiusM,
                double momentOfInertiaKgM2,
                double surfaceVelocityKinematicFactor,
                double maxRpm,
                double nominalVoltageV) {
            this.flywheelRadiusM = radiusM;
            this.flywheelMomentOfInertiaKgM2 = momentOfInertiaKgM2;
            this.surfaceVelocityKinematicFactor = surfaceVelocityKinematicFactor;
            this.maxFlywheelRpm = maxRpm;
            this.nominalVoltageV = nominalVoltageV;
            return this;
        }

        public Builder setEmpiricalTransferModel(
                double baseTransferEfficiency,
                double speedDependentSlipFactor,
                double compressionEnergyLossFactor,
                double feederActuationDelaySec,
                double sensorLatencySec) {
            this.baseTransferEfficiency = baseTransferEfficiency;
            this.speedDependentSlipFactor = speedDependentSlipFactor;
            this.compressionEnergyLossFactor = compressionEnergyLossFactor;
            this.feederActuationDelaySec = feederActuationDelaySec;
            this.sensorLatencySec = sensorLatencySec;
            return this;
        }

        public Builder setFlywheelControlParams(
                double kS,
                double kV,
                double kA,
                double kP,
                double kI,
                double kD,
                double readyToleranceRpm,
                double stabilityDerivativeRpmPerSec) {
            this.flywheelKsVolts = kS;
            this.flywheelKvVoltsPerRadPerSec = kV;
            this.flywheelKaVoltsPerRadPerSec2 = kA;
            this.flywheelKp = kP;
            this.flywheelKi = kI;
            this.flywheelKd = kD;
            this.flywheelReadyToleranceRpm = readyToleranceRpm;
            this.flywheelStabilityDerivativeRpmPerSec = stabilityDerivativeRpmPerSec;
            return this;
        }

        public ShooterConfig build() {
            requireFinitePositive(projectileMassKg, "projectileMassKg");
            requireFinitePositive(projectileCrossSectionM2, "projectileCrossSectionM2");
            requireFinitePositive(projectileRadiusM, "projectileRadiusM");
            requireFinite(launcherOffsetXM, "launcherOffsetXM");
            requireFinite(launcherOffsetYM, "launcherOffsetYM");
            requireFinitePositive(launcherExitHeightM, "launcherExitHeightM");
            requireFinitePositive(fixedLaunchAngleRad, "fixedLaunchAngleRad");
            requireFinitePositive(flywheelRadiusM, "flywheelRadiusM");
            requireFinitePositive(flywheelMomentOfInertiaKgM2, "flywheelMomentOfInertiaKgM2");
            return new ShooterConfig(this);
        }

        private static void requireFinitePositive(double val, String name) {
            if (Double.isNaN(val) || Double.isInfinite(val) || val <= 0.0) {
                throw new IllegalStateException(
                        "Missing or invalid hardware measurement '" + name + "': " + val
                                + ". Supply actual robot measurements before building ShooterConfig.");
            }
        }

        private static void requireFinite(double val, String name) {
            if (Double.isNaN(val) || Double.isInfinite(val)) {
                throw new IllegalStateException(
                        "Missing hardware measurement '" + name + "'. Supply actual robot measurement.");
            }
        }
    }
}
