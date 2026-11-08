package org.firstinspires.ftc.teamcode.targeting;

/**
 * Real-Time Projectile Launch Event Detector, Feeder Latency Tracker, and Kinetic Energy Analyzer.
 *
 * <p>Monitors high-rate flywheel encoder velocity and angular acceleration (plus optional optical
 * break-beam sensors) during projectile feeding to:
 * <ul>
 *   <li>Measure actual mechanical feeder actuation delay (\tau_feeder) from trigger command to
 *       wheel contact.</li>
 *   <li>Measure exact flywheel rotational kinetic energy loss \Delta E_w = 0.5 * I_w * (\omega_i^2 - \omega_f^2)
 *       and reconstruct actual projectile exit velocity v_0 without external radar/chronographs.</li>
 *   <li>Classify feed quality ({@code NORMAL_SHOT}, {@code DRY_FIRE_OR_MISFEED},
 *       {@code UNDER_COMPRESSED_SHOT}, {@code DOUBLE_FEED_OR_JAM}).</li>
 * </ul>
 */
public final class LaunchEventDetector {

    private static final double RPM_TO_RAD_PER_SEC = (2.0 * Math.PI) / 60.0;

    public enum FeedAnomalyType {
        NONE,
        NORMAL_SHOT,
        DRY_FIRE_OR_MISFEED,
        UNDER_COMPRESSED_SHOT,
        DOUBLE_FEED_OR_JAM
    }

    private final ShooterConfig config;

    // State machine for tracking an active feed-to-launch event
    private boolean awaitingContact = false;
    private boolean inContactTrough = false;
    private double timeSinceFeederTriggerSec = 0.0;
    private double preShotRpm = 0.0;
    private double troughMinRpm = 0.0;
    private double expectedRpmDrop = 0.0;

    // Live telemetry & learned feeder delay
    private double estimatedFeederDelaySec;
    private double lastMeasuredFeederDelaySec = 0.0;
    private double lastMeasuredRpmDrop = 0.0;
    private double lastInferredExitVelocityMpS = 0.0;
    private FeedAnomalyType lastFeedClassification = FeedAnomalyType.NONE;
    private int totalNormalShots = 0;
    private int totalAnomalousFeeds = 0;

    public LaunchEventDetector(ShooterConfig config) {
        this.config = config;
        this.estimatedFeederDelaySec = config.feederActuationDelaySec;
    }

    /**
     * Call when the feeder servo/indexer is commanded to push a projectile into the flywheel.
     */
    public void notifyFeederActuated(double currentFilteredRpm, double predictedRpmDrop) {
        this.awaitingContact = true;
        this.inContactTrough = false;
        this.timeSinceFeederTriggerSec = 0.0;
        this.preShotRpm = currentFilteredRpm;
        this.troughMinRpm = currentFilteredRpm;
        this.expectedRpmDrop = Math.max(25.0, predictedRpmDrop);
    }

    /**
     * Updates the launch detector state machine each control-loop cycle. Zero heap allocations.
     *
     * @param filteredRpm            Current observer-filtered flywheel RPM
     * @param filteredAccelRpmPerSec Current observer-filtered flywheel angular acceleration (RPM/s)
     * @param dtSec                  Control loop timestep (seconds)
     */
    public void update(double filteredRpm, double filteredAccelRpmPerSec, double dtSec) {
        double dt = Math.max(1e-4, dtSec);

        if (awaitingContact) {
            timeSinceFeederTriggerSec += dt;

            // Detect onset of projectile contact via sharp deceleration or significant RPM drop
            double currentDrop = preShotRpm - filteredRpm;
            if (filteredAccelRpmPerSec < -650.0 || currentDrop > 0.30 * expectedRpmDrop) {
                awaitingContact = false;
                inContactTrough = true;
                troughMinRpm = filteredRpm;
                lastMeasuredFeederDelaySec = timeSinceFeederTriggerSec;

                // Update exponentially weighted moving average of feeder delay within sane bounds
                if (lastMeasuredFeederDelaySec >= 0.010 && lastMeasuredFeederDelaySec <= 0.220) {
                    estimatedFeederDelaySec =
                            0.80 * estimatedFeederDelaySec + 0.20 * lastMeasuredFeederDelaySec;
                }
            } else if (timeSinceFeederTriggerSec > 0.250) {
                // Feeder timed out without any flywheel speed drop -> dry fire or empty magazine
                awaitingContact = false;
                lastMeasuredRpmDrop = 0.0;
                lastInferredExitVelocityMpS = 0.0;
                lastFeedClassification = FeedAnomalyType.DRY_FIRE_OR_MISFEED;
                totalAnomalousFeeds++;
            }
            return;
        }

        if (inContactTrough) {
            timeSinceFeederTriggerSec += dt;
            if (filteredRpm < troughMinRpm) {
                troughMinRpm = filteredRpm;
            }

            // Once acceleration turns positive (flywheel begins recovering) or 50ms after contact,
            // lock in the trough minimum RPM and reconstruct kinetic energy transfer!
            if (filteredAccelRpmPerSec >= 0.0 || timeSinceFeederTriggerSec > lastMeasuredFeederDelaySec + 0.055) {
                inContactTrough = false;
                lastMeasuredRpmDrop = Math.max(0.0, preShotRpm - troughMinRpm);

                double omegaInitial = preShotRpm * RPM_TO_RAD_PER_SEC;
                double omegaFinal = Math.max(0.0, troughMinRpm) * RPM_TO_RAD_PER_SEC;
                double deltaOmegaSq = Math.max(0.0, omegaInitial * omegaInitial - omegaFinal * omegaFinal);

                // 0.5 * I_w * (omega_i^2 - omega_f^2) = 0.5 * m_eff * v_0^2
                lastInferredExitVelocityMpS =
                        Math.sqrt(
                                (config.flywheelMomentOfInertiaKgM2 * deltaOmegaSq)
                                        / config.effectiveProjectileInertialMassKg);

                double dropRatio = lastMeasuredRpmDrop / expectedRpmDrop;
                if (dropRatio < 0.55) {
                    lastFeedClassification = FeedAnomalyType.UNDER_COMPRESSED_SHOT;
                    totalAnomalousFeeds++;
                } else if (dropRatio > 1.55) {
                    lastFeedClassification = FeedAnomalyType.DOUBLE_FEED_OR_JAM;
                    totalAnomalousFeeds++;
                } else {
                    lastFeedClassification = FeedAnomalyType.NORMAL_SHOT;
                    totalNormalShots++;
                }
            }
        }
    }

    public double getEstimatedFeederDelaySec() {
        return estimatedFeederDelaySec;
    }

    public double getLastMeasuredFeederDelaySec() {
        return lastMeasuredFeederDelaySec;
    }

    public double getLastMeasuredRpmDrop() {
        return lastMeasuredRpmDrop;
    }

    public double getLastInferredExitVelocityMpS() {
        return lastInferredExitVelocityMpS;
    }

    public FeedAnomalyType getLastFeedClassification() {
        return lastFeedClassification;
    }

    public int getTotalNormalShots() {
        return totalNormalShots;
    }

    public int getTotalAnomalousFeeds() {
        return totalAnomalousFeeds;
    }
}
