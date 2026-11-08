package org.firstinspires.ftc.teamcode.targeting;

/**
 * Core Physics Engine for the FTC Flywheel Targeting System.
 *
 * <p>Combines first-principles mechanics (rotational kinetic energy transfer, inelastic projectile
 * compression, quadratic aerodynamic drag, and Magnus lift) with calibrated empirical correction
 * factors. Designed for zero heap allocations and microsecond-level execution.
 */
public final class PhysicsEngine {

    private static final double RPM_TO_RAD_PER_SEC = (2.0 * Math.PI) / 60.0;
    private static final double RAD_PER_SEC_TO_RPM = 60.0 / (2.0 * Math.PI);

    private final ShooterConfig config;
    /** Precomputed inertial coupling denominator: sqrt(1 + (m_eff * k_v^2 * R^2) / I_w). */
    private final double inertialDropDenominator;
    /** Effective kinematic radius accounting for single/dual wheel factor and inertial drop. */
    private final double effectiveKinematicRadiusM;

    // Live calibration multipliers updated atomically by ShotCalibration (default = 1.0, 0.0)
    private volatile double calibratedEfficiencyScale = 1.0;
    private volatile double calibratedDragScale = 1.0;
    private volatile double calibratedMagnusScale = 1.0;
    private volatile double calibratedPitchBiasRad = 0.0;

    public PhysicsEngine(ShooterConfig config) {
        this.config = config;
        double kvR = config.surfaceVelocityKinematicFactor * config.flywheelRadiusM;
        this.inertialDropDenominator =
                Math.sqrt(1.0 + (config.effectiveProjectileInertialMassKg * kvR * kvR)
                        / config.flywheelMomentOfInertiaKgM2);
        this.effectiveKinematicRadiusM = kvR / this.inertialDropDenominator;
    }

    public ShooterConfig getConfig() {
        return config;
    }

    /**
     * Updates empirical calibration multipliers discovered from real-world shot logging.
     * Bounded to physically sane intervals so calibration never overrides core physics.
     */
    public void setCalibrationCorrections(
            double efficiencyScale,
            double dragScale,
            double magnusScale,
            double pitchBiasRad) {
        this.calibratedEfficiencyScale = clamp(efficiencyScale, 0.70, 1.30);
        this.calibratedDragScale = clamp(dragScale, 0.50, 2.00);
        this.calibratedMagnusScale = clamp(magnusScale, 0.00, 2.00);
        this.calibratedPitchBiasRad = clamp(pitchBiasRad, -0.087, 0.087); // +/- 5 degrees max
    }

    public double getCalibratedEfficiencyScale() {
        return calibratedEfficiencyScale;
    }

    public double getCalibratedDragScale() {
        return calibratedDragScale;
    }

    public double getCalibratedMagnusScale() {
        return calibratedMagnusScale;
    }

    public double getCalibratedPitchBiasRad() {
        return calibratedPitchBiasRad;
    }

    /**
     * Calculates projectile exit velocity v_0 (m/s) relative to the launcher given flywheel RPM.
     *
     * <p>Accounts for:
     * <ul>
     *   <li>Single vs. dual flywheel surface kinematics (\kappa_v * \omega * R)</li>
     *   <li>Flywheel speed drop during contact via conservation of energy</li>
     *   <li>Projectile rotational kinetic energy and foam/shell compression energy loss</li>
     *   <li>Speed-dependent wheel-to-projectile slip \eta(\omega)</li>
     * </ul>
     */
    public double calculateExitVelocityFromRpm(double flywheelRpm) {
        if (flywheelRpm <= 0.0) {
            return 0.0;
        }
        double omega = flywheelRpm * RPM_TO_RAD_PER_SEC;
        double slipEfficiency =
                calibratedEfficiencyScale
                        * (config.baseTransferEfficiency - config.speedDependentSlipFactor * omega);
        slipEfficiency = clamp(slipEfficiency, 0.15, 1.0);
        return slipEfficiency * effectiveKinematicRadiusM * omega;
    }

    /**
     * Inverse mechanical model: calculates required flywheel RPM to achieve a desired
     * launcher-relative projectile exit velocity v_0 (m/s).
     *
     * <p>Solves the quadratic slip relation:
     * {@code eta_cal * (eta_0 * omega - eta_1 * omega^2) * R_eff = v_0} analytically.
     */
    public double calculateRequiredRpmFromExitVelocity(double requiredExitVelocityMpS) {
        if (requiredExitVelocityMpS <= 0.0) {
            return 0.0;
        }
        double targetIdealOmega =
                requiredExitVelocityMpS / (calibratedEfficiencyScale * effectiveKinematicRadiusM);
        double eta0 = config.baseTransferEfficiency;
        double eta1 = config.speedDependentSlipFactor;

        double omegaRadPerSec;
        if (Math.abs(eta1) < 1e-9) {
            omegaRadPerSec = targetIdealOmega / eta0;
        } else {
            // Quadratic: eta1 * omega^2 - eta0 * omega + targetIdealOmega = 0
            double discriminant = eta0 * eta0 - 4.0 * eta1 * targetIdealOmega;
            if (discriminant < 0.0) {
                // Requested velocity exceeds physical slip ceiling; clamp to peak velocity omega
                omegaRadPerSec = eta0 / (2.0 * eta1);
            } else {
                // Take the smaller (physically stable, pre-saturation) root
                omegaRadPerSec = (eta0 - Math.sqrt(discriminant)) / (2.0 * eta1);
            }
        }
        return omegaRadPerSec * RAD_PER_SEC_TO_RPM;
    }

    /**
     * Predicts the instantaneous flywheel RPM drop when a single projectile is launched at the
     * given pre-shot RPM and exit velocity, using rotational vs. translational/compression energy balance:
     * {@code 0.5 * I_w * (omega_i^2 - omega_f^2) = 0.5 * m_eff * v_0^2}.
     */
    public double predictFlywheelRpmDrop(double preShotRpm, double exitVelocityMpS) {
        if (preShotRpm <= 0.0 || exitVelocityMpS <= 0.0) {
            return 0.0;
        }
        double omegaInitial = preShotRpm * RPM_TO_RAD_PER_SEC;
        double omegaSqDrop =
                (config.effectiveProjectileInertialMassKg * exitVelocityMpS * exitVelocityMpS)
                        / config.flywheelMomentOfInertiaKgM2;
        double omegaFinalSq = Math.max(0.0, omegaInitial * omegaInitial - omegaSqDrop);
        double omegaFinal = Math.sqrt(omegaFinalSq);
        return (omegaInitial - omegaFinal) * RAD_PER_SEC_TO_RPM;
    }

    /**
     * Solves for the required projectile launch speed (in the field frame) and flight time to hit
     * a target at horizontal range {@code distanceM} and vertical displacement {@code deltaZM}
     * given launch angle {@code launchAngleRad}.
     *
     * <p>Uses an exact vacuum analytical initializer followed by a perturbative drag/Magnus
     * starter and 2 fast secant-Newton refinements against the aerodynamic trajectory equations.
     *
     * @param distanceM      Horizontal distance from launcher exit to target center (m)
     * @param deltaZM        Vertical displacement z_target - z_launcher (m)
     * @param launchAngleRad Launch elevation angle above horizontal (radians)
     * @param outResult      2-element array populated in-place: [0] = required speed (m/s), [1] = flight time (s).
     * @return true if the target is physically reachable at this angle, false otherwise
     */
    public boolean solveRequiredVelocityAndFlightTime(
            double distanceM,
            double deltaZM,
            double launchAngleRad,
            double[] outResult) {
        if (distanceM <= 0.05) {
            return false;
        }
        double effectiveAngle = launchAngleRad + calibratedPitchBiasRad;
        double cosTheta = Math.cos(effectiveAngle);
        double sinTheta = Math.sin(effectiveAngle);
        if (cosTheta <= 0.05) {
            return false;
        }
        double tanTheta = sinTheta / cosTheta;
        double heightMargin = distanceM * tanTheta - deltaZM;
        // Target is above the vacuum line-of-launch tangent: physically impossible at any finite speed
        if (heightMargin <= 0.01) {
            return false;
        }

        double g = config.gravityMpS2;
        double kd = config.ballisticDragFactorPerM * calibratedDragScale;
        double km = config.magnusFactor * calibratedMagnusScale;

        // 1. Vacuum analytical solution
        double vVacSq = (g * distanceM * distanceM) / (2.0 * cosTheta * cosTheta * heightMargin);
        double vVac = Math.sqrt(vVacSq);

        // 2. First-order analytical perturbative correction for quadratic drag & Magnus backspin lift
        double pathLengthApprox = distanceM / cosTheta;
        double dragMultiplier = Math.exp(0.55 * kd * pathLengthApprox);
        // Magnus lift slightly reduces effective gravity when backspin is present
        double magnusLiftRatio = clamp(km * vVacSq * cosTheta / g, -0.25, 0.35);
        double vGuess = vVac * dragMultiplier * Math.sqrt(Math.max(0.5, 1.0 - 0.5 * magnusLiftRatio));

        // 3. Two-step fast secant refinement using 8-step RK4 vertical residual at x = distanceM
        double v0 = vGuess;
        double zErr0 = evaluateVerticalResidualAtRange(v0, effectiveAngle, distanceM, deltaZM, kd, km, outResult);
        if (Double.isNaN(zErr0)) {
            return false;
        }
        if (Math.abs(zErr0) > 1e-4) {
            // Analytical sensitivity dz/dv0 from vacuum + drag model for initial step
            double dzDvApprox = (2.0 * heightMargin) / v0;
            double v1 = Math.max(1.0, v0 - zErr0 / Math.max(0.05, dzDvApprox));
            double zErr1 = evaluateVerticalResidualAtRange(v1, effectiveAngle, distanceM, deltaZM, kd, km, outResult);
            if (!Double.isNaN(zErr1) && Math.abs(zErr1 - zErr0) > 1e-7) {
                double v2 = v1 - zErr1 * (v1 - v0) / (zErr1 - zErr0);
                if (v2 > 0.5 && v2 < 60.0) {
                    evaluateVerticalResidualAtRange(v2, effectiveAngle, distanceM, deltaZM, kd, km, outResult);
                    outResult[0] = v2;
                    return true;
                }
            }
            outResult[0] = v1;
            return true;
        }
        outResult[0] = v0;
        return true;
    }

    /**
     * Integrates the 2D planar aerodynamic equations of motion (gravity + quadratic drag + Magnus)
     * across horizontal distance {@code targetRangeM} using a fixed 8-step 4th-order Runge-Kutta
     * spatial march (parameterized by horizontal coordinate x so it terminates at exact range D).
     *
     * <p>Because dx is fixed ({@code targetRangeM / 8}), this requires zero branching or root-finding
     * for range termination and executes in ~1.2 microseconds.
     *
     * @param outResult Populates outResult[1] with exact flight time (s) and outResult[2] with impact angle (rad).
     * @return Vertical height error (z_actual(D) - deltaZM) in meters, or NaN if projectile stalls.
     */
    public double evaluateVerticalResidualAtRange(
            double launchSpeedMpS,
            double effectiveAngleRad,
            double targetRangeM,
            double targetDeltaZM,
            double kd,
            double km,
            double[] outResult) {
        final int steps = 8;
        final double dx = targetRangeM / steps;
        final double g = config.gravityMpS2;

        double vx = launchSpeedMpS * Math.cos(effectiveAngleRad);
        double vz = launchSpeedMpS * Math.sin(effectiveAngleRad);
        double z = 0.0;
        double t = 0.0;

        for (int i = 0; i < steps; i++) {
            if (vx <= 0.25) {
                return Double.NaN; // Projectile stalled horizontally before reaching target
            }
            // State derivatives with respect to horizontal distance x:
            // dz/dx = vz / vx
            // dt/dx = 1 / vx
            // dvx/dx = (-kd * v * vx - km * v * vz) / vx
            // dvz/dx = (-g - kd * v * vz + km * v * vx) / vx
            double v1 = Math.hypot(vx, vz);
            double invVx1 = 1.0 / vx;
            double k1z = vz * invVx1;
            double k1t = invVx1;
            double k1vx = (-kd * v1 * vx - km * v1 * vz) * invVx1;
            double k1vz = (-g - kd * v1 * vz + km * v1 * vx) * invVx1;

            double vx2 = vx + 0.5 * dx * k1vx;
            double vz2 = vz + 0.5 * dx * k1vz;
            if (vx2 <= 0.25) return Double.NaN;
            double v2 = Math.hypot(vx2, vz2);
            double invVx2 = 1.0 / vx2;
            double k2z = vz2 * invVx2;
            double k2t = invVx2;
            double k2vx = (-kd * v2 * vx2 - km * v2 * vz2) * invVx2;
            double k2vz = (-g - kd * v2 * vz2 + km * v2 * vx2) * invVx2;

            double vx3 = vx + 0.5 * dx * k2vx;
            double vz3 = vz + 0.5 * dx * k2vz;
            if (vx3 <= 0.25) return Double.NaN;
            double v3 = Math.hypot(vx3, vz3);
            double invVx3 = 1.0 / vx3;
            double k3z = vz3 * invVx3;
            double k3t = invVx3;
            double k3vx = (-kd * v3 * vx3 - km * v3 * vz3) * invVx3;
            double k3vz = (-g - kd * v3 * vz3 + km * v3 * vx3) * invVx3;

            double vx4 = vx + dx * k3vx;
            double vz4 = vz + dx * k3vz;
            if (vx4 <= 0.25) return Double.NaN;
            double v4 = Math.hypot(vx4, vz4);
            double invVx4 = 1.0 / vx4;
            double k4z = vz4 * invVx4;
            double k4t = invVx4;
            double k4vx = (-kd * v4 * vx4 - km * v4 * vz4) * invVx4;
            double k4vz = (-g - kd * v4 * vz4 + km * v4 * vx4) * invVx4;

            double sixthDx = dx / 6.0;
            z += sixthDx * (k1z + 2.0 * k2z + 2.0 * k3z + k4z);
            t += sixthDx * (k1t + 2.0 * k2t + 2.0 * k3t + k4t);
            vx += sixthDx * (k1vx + 2.0 * k2vx + 2.0 * k3vx + k4vx);
            vz += sixthDx * (k1vz + 2.0 * k2vz + 2.0 * k3vz + k4vz);
        }

        outResult[1] = t;
        if (outResult.length > 2) {
            outResult[2] = Math.atan2(vz, vx);
        }
        return z - targetDeltaZM;
    }

    public static double clamp(double val, double min, double max) {
        return Math.max(min, Math.min(max, val));
    }
}
