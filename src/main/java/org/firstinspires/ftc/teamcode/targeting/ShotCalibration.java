package org.firstinspires.ftc.teamcode.targeting;

import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.atomic.AtomicBoolean;
import org.firstinspires.ftc.teamcode.targeting.TargetingTypes.ShotRecord;

/**
 * Physics-Informed Experimental Shot Calibration & Residual Parameter Estimator.
 *
 * <p>Records experimental shot outcomes (commanded/actual RPM, robot position, distance, target
 * height, launch angle, predicted vs. actual impact location, and 3D shot error) and performs
 * Tikhonov-regularized (Ridge) Gauss-Newton parameter estimation outside the real-time loop.
 *
 * <p>Guaranteed Physics Primacy:
 * <ul>
 *   <li>Bayesian prior regularization (\Lambda) anchors parameters to theoretical physics when
 *       sample counts are small or collinear.</li>
 *   <li>Hard physical trust-region bounds prevent empirical noise or outliers from overriding
 *       the analytical physics foundation.</li>
 * </ul>
 */
public final class ShotCalibration {

    public static final int MAX_RECORDS = 128;

    private final PhysicsEngine physics;
    private final ShooterConfig config;

    // Pre-allocated ring buffer of ShotRecords
    private final ShotRecord[] records = new ShotRecord[MAX_RECORDS];
    private int recordHead = 0;
    private int recordCount = 0;

    // Summary error statistics across logged shots
    private volatile double meanRadialErrorM = 0.0;
    private volatile double rmseVerticalErrorM = 0.0;

    // Background single-thread executor for non-blocking model fitting
    private final ExecutorService calibrationExecutor;
    private final AtomicBoolean fittingInProgress = new AtomicBoolean(false);

    public ShotCalibration(PhysicsEngine physics) {
        this.physics = physics;
        this.config = physics.getConfig();
        for (int i = 0; i < MAX_RECORDS; i++) {
            records[i] = new ShotRecord(0, 0, 0, 0, 0, 0, 0, 12.0, 0, 0, 0, 0, 0, 0);
        }
        this.calibrationExecutor =
                Executors.newSingleThreadExecutor(
                        r -> {
                            Thread t = new Thread(r, "FTC-ShotCalibration-Fitter");
                            t.setDaemon(true);
                            t.setPriority(Thread.MIN_PRIORITY);
                            return t;
                        });
    }

    /**
     * Records a completed shot into the pre-allocated ring buffer in O(1) time with zero heap
     * allocations. Safe to call directly at the end of a shot event.
     */
    public synchronized void recordShot(
            double robotXM,
            double robotYM,
            double shootingDistanceM,
            double targetHeightM,
            double launchAngleRad,
            double commandedFlywheelRpm,
            double actualFlywheelRpm,
            double batteryVoltageV,
            double predictedImpactXM,
            double predictedImpactYM,
            double predictedImpactZM,
            double actualImpactXM,
            double actualImpactYM,
            double actualImpactZM) {

        ShotRecord slot = records[recordHead];
        slot.robotXM = robotXM;
        slot.robotYM = robotYM;
        slot.shootingDistanceM = shootingDistanceM;
        slot.targetHeightM = targetHeightM;
        slot.launchAngleRad = launchAngleRad;
        slot.commandedFlywheelRpm = commandedFlywheelRpm;
        slot.actualFlywheelRpm = actualFlywheelRpm;
        slot.batteryVoltageV = batteryVoltageV;
        slot.predictedImpactXM = predictedImpactXM;
        slot.predictedImpactYM = predictedImpactYM;
        slot.predictedImpactZM = predictedImpactZM;
        slot.actualImpactXM = actualImpactXM;
        slot.actualImpactYM = actualImpactYM;
        slot.actualImpactZM = actualImpactZM;

        recordHead = (recordHead + 1) % MAX_RECORDS;
        if (recordCount < MAX_RECORDS) {
            recordCount++;
        }
    }

    /**
     * Triggers an asynchronous model calibration fit on the low-priority background thread so the
     * real-time robot control loop is never blocked.
     */
    public void triggerAsyncCalibration() {
        if (recordCount < 3) {
            return;
        }
        if (fittingInProgress.compareAndSet(false, true)) {
            calibrationExecutor.submit(
                    () -> {
                        try {
                            runCalibrationFitSynchronously();
                        } finally {
                            fittingInProgress.set(false);
                        }
                    });
        }
    }

    /**
     * Runs Tikhonov-regularized (Ridge) physics-residual parameter estimation synchronously.
     * Call directly in unit tests, autonomous initialization, or between matches; use
     * {@link #triggerAsyncCalibration()} during live gameplay.
     */
    public synchronized void runCalibrationFitSynchronously() {
        if (recordCount == 0) {
            return;
        }

        // Compute current error metrics and build 3x3 Ridge Regression normal equations:
        // Parameter vector delta = [delta_efficiency_scale, delta_drag_scale, delta_pitch_bias_rad]^T
        // Prior regularization weights (Bayesian anchor to theoretical physics):
        final double lambdaEfficiency = 2.5;
        final double lambdaDrag = 6.0;
        final double lambdaPitch = 120.0;

        double[][] aMat = new double[3][3];
        aMat[0][0] = lambdaEfficiency;
        aMat[1][1] = lambdaDrag;
        aMat[2][2] = lambdaPitch;
        double[] bVec = new double[3];

        double sumRadialErr = 0.0;
        double sumSqVertErr = 0.0;
        int validSamples = 0;

        double g = config.gravityMpS2;
        double kdBase = config.ballisticDragFactorPerM;
        double kmBase = config.magnusFactor;
        double[] rk4Scratch = new double[3];

        // Temporarily evaluate against uncalibrated baseline (scale = 1.0) to fit total offset from theory
        double kvR = config.surfaceVelocityKinematicFactor * config.flywheelRadiusM;
        double inertialDenom =
                Math.sqrt(1.0 + (config.effectiveProjectileInertialMassKg * kvR * kvR)
                        / config.flywheelMomentOfInertiaKgM2);
        double effRadius = kvR / inertialDenom;

        for (int i = 0; i < recordCount; i++) {
            ShotRecord rec = records[i];
            sumRadialErr += rec.totalImpactErrorM();
            double liveVertErr = rec.verticalImpactErrorM();
            sumSqVertErr += liveVertErr * liveVertErr;

            if (rec.shootingDistanceM < 0.3 || rec.actualFlywheelRpm < 200.0) {
                continue;
            }

            // Compute uncalibrated baseline exit speed for actual measured RPM
            double omega = rec.actualFlywheelRpm * ((2.0 * Math.PI) / 60.0);
            double baseSlip =
                    PhysicsEngine.clamp(
                            config.baseTransferEfficiency - config.speedDependentSlipFactor * omega,
                            0.15,
                            1.0);
            double v0Base = baseSlip * effRadius * omega;

            double deltaZTarget = rec.targetHeightM - config.launcherExitHeightM;
            double zResidualVsTarget =
                    physics.evaluateVerticalResidualAtRange(
                            v0Base,
                            rec.launchAngleRad,
                            rec.shootingDistanceM,
                            deltaZTarget,
                            kdBase,
                            kmBase,
                            rk4Scratch);
            if (Double.isNaN(zResidualVsTarget)) {
                continue;
            }

            double zBasePredicted = config.launcherExitHeightM + deltaZTarget + zResidualVsTarget;
            double residualZ = rec.actualImpactZM - zBasePredicted;

            // Outlier gate: ignore gross mechanical jams (> 1.2m vertical error)
            if (Math.abs(residualZ) > 1.2) {
                continue;
            }

            double cosTheta = Math.max(0.15, Math.cos(rec.launchAngleRad));
            double dist = rec.shootingDistanceM;
            double v0Sq = v0Base * v0Base;

            // Analytical sensitivities dz / d(param):
            // 1. dz / d(s_eta) = (dz / dv0) * v0Base = g * D^2 / (v0^2 * cos^2(theta))
            double jEta = (g * dist * dist) / (v0Sq * cosTheta * cosTheta);
            // 2. dz / d(s_drag) = -0.5 * g * D^3 * kdBase / (v0^2 * cos^3(theta))
            double jDrag = -(0.55 * g * dist * dist * dist * kdBase) / (v0Sq * cosTheta * cosTheta * cosTheta);
            // 3. dz / d(pitchBias) = D / cos^2(theta)
            double jPitch = dist / (cosTheta * cosTheta);

            double[] jRow = {jEta, jDrag, jPitch};
            for (int r = 0; r < 3; r++) {
                bVec[r] += jRow[r] * residualZ;
                for (int c = 0; c < 3; c++) {
                    aMat[r][c] += jRow[r] * jRow[c];
                }
            }
            validSamples++;
        }

        meanRadialErrorM = sumRadialErr / recordCount;
        rmseVerticalErrorM = Math.sqrt(sumSqVertErr / recordCount);

        if (validSamples < 3) {
            return;
        }

        double[] delta = solve3x3LinearSystem(aMat, bVec);
        if (delta != null) {
            double newEfficiencyScale = PhysicsEngine.clamp(1.0 + delta[0], 0.75, 1.25);
            double newDragScale = PhysicsEngine.clamp(1.0 + delta[1], 0.65, 1.50);
            double newPitchBiasRad = PhysicsEngine.clamp(delta[2], -0.052, 0.052); // +/- 3 deg max
            physics.setCalibrationCorrections(
                    newEfficiencyScale,
                    newDragScale,
                    physics.getCalibratedMagnusScale(),
                    newPitchBiasRad);
        }
    }

    /** Direct Cramer/Gaussian 3x3 solver with partial pivoting (zero external library overhead). */
    private static double[] solve3x3LinearSystem(double[][] A, double[] b) {
        double[][] m = {
            {A[0][0], A[0][1], A[0][2], b[0]},
            {A[1][0], A[1][1], A[1][2], b[1]},
            {A[2][0], A[2][1], A[2][2], b[2]}
        };

        for (int col = 0; col < 3; col++) {
            int pivot = col;
            for (int row = col + 1; row < 3; row++) {
                if (Math.abs(m[row][col]) > Math.abs(m[pivot][col])) {
                    pivot = row;
                }
            }
            double[] tmp = m[col];
            m[col] = m[pivot];
            m[pivot] = tmp;

            if (Math.abs(m[col][col]) < 1e-10) {
                return null;
            }
            for (int row = col + 1; row < 3; row++) {
                double factor = m[row][col] / m[col][col];
                for (int k = col; k < 4; k++) {
                    m[row][k] -= factor * m[col][k];
                }
            }
        }

        double[] x = new double[3];
        for (int row = 2; row >= 0; row--) {
            double sum = m[row][3];
            for (int col = row + 1; col < 3; col++) {
                sum -= m[row][col] * x[col];
            }
            x[row] = sum / m[row][row];
        }
        return x;
    }

    public int getRecordCount() {
        return recordCount;
    }

    public synchronized ShotRecord getRecordAt(int index) {
        if (index < 0 || index >= recordCount) {
            throw new IndexOutOfBoundsException("Index: " + index + ", Size: " + recordCount);
        }
        return records[index];
    }

    public double getMeanRadialErrorM() {
        return meanRadialErrorM;
    }

    public double getRmseVerticalErrorM() {
        return rmseVerticalErrorM;
    }

    public void shutdown() {
        calibrationExecutor.shutdownNow();
    }
}
