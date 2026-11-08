package org.firstinspires.ftc.teamcode.targeting;

/**
 * Zero-Allocation Real-Time Performance Profiler & Deadline Monitor.
 *
 * <p>Tracks microsecond-precision execution times for sensor acquisition, localization, physics &
 * trajectory solving, flywheel control, and total end-to-end targeting cycle latency.
 */
public final class PerformanceMonitor {

    public static final double TARGETING_COMPUTE_DEADLINE_MS = 5.0;
    public static final double FULL_CYCLE_DEADLINE_MS = 20.0;

    private static final int WINDOW_SIZE = 128;

    private final double[] totalUpdateTimesMs = new double[WINDOW_SIZE];
    private final double[] physicsCalcTimesMs = new double[WINDOW_SIZE];
    private int ringIndex = 0;
    private int sampleCount = 0;

    private long cycleStartNanos;
    private long stageStartNanos;
    private long lastCycleCompletionNanos = 0L;

    // Latest cycle stage durations in milliseconds
    private double lastSensorLatencyMs = 0.0;
    private double lastLocalizationTimeMs = 0.0;
    private double lastPhysicsAndTrajectoryTimeMs = 0.0;
    private double lastFlywheelControlTimeMs = 0.0;
    private double lastTotalTargetingTimeMs = 0.0;

    // Aggregate statistics
    private double maxPhysicsTimeMs = 0.0;
    private double maxTotalTimeMs = 0.0;
    private double rollingAvgPhysicsTimeMs = 0.0;
    private double rollingAvgTotalTimeMs = 0.0;
    private double updateFrequencyHz = 0.0;

    private long totalCyclesExecuted = 0L;
    private long missedComputeDeadlinesCount = 0L; // > 5 ms
    private long missedLoopIntervalDeadlinesCount = 0L; // > 20 ms loop-to-loop interval

    public void beginCycle() {
        long now = System.nanoTime();
        if (lastCycleCompletionNanos != 0L) {
            double loopIntervalMs = (now - lastCycleCompletionNanos) * 1e-6;
            if (loopIntervalMs > FULL_CYCLE_DEADLINE_MS) {
                missedLoopIntervalDeadlinesCount++;
            }
            double instantHz = 1000.0 / Math.max(0.1, loopIntervalMs);
            updateFrequencyHz =
                    (updateFrequencyHz == 0.0)
                            ? instantHz
                            : (0.90 * updateFrequencyHz + 0.10 * instantHz);
        }
        lastCycleCompletionNanos = now;
        cycleStartNanos = now;
        stageStartNanos = now;
    }

    public void recordSensorAcquisitionComplete() {
        long now = System.nanoTime();
        lastSensorLatencyMs = (now - stageStartNanos) * 1e-6;
        stageStartNanos = now;
    }

    public void recordLocalizationComplete() {
        long now = System.nanoTime();
        lastLocalizationTimeMs = (now - stageStartNanos) * 1e-6;
        stageStartNanos = now;
    }

    public void recordPhysicsAndTrajectoryComplete() {
        long now = System.nanoTime();
        lastPhysicsAndTrajectoryTimeMs = (now - stageStartNanos) * 1e-6;
        stageStartNanos = now;
    }

    public void recordFlywheelControlComplete() {
        long now = System.nanoTime();
        lastFlywheelControlTimeMs = (now - stageStartNanos) * 1e-6;
        stageStartNanos = now;
    }

    public void endCycle() {
        long now = System.nanoTime();
        lastTotalTargetingTimeMs = (now - cycleStartNanos) * 1e-6;
        totalCyclesExecuted++;

        if (lastTotalTargetingTimeMs > TARGETING_COMPUTE_DEADLINE_MS) {
            missedComputeDeadlinesCount++;
        }
        if (lastPhysicsAndTrajectoryTimeMs > maxPhysicsTimeMs) {
            maxPhysicsTimeMs = lastPhysicsAndTrajectoryTimeMs;
        }
        if (lastTotalTargetingTimeMs > maxTotalTimeMs) {
            maxTotalTimeMs = lastTotalTargetingTimeMs;
        }

        // Update rolling ring buffer average in O(1) time
        double oldTotal = totalUpdateTimesMs[ringIndex];
        double oldPhys = physicsCalcTimesMs[ringIndex];
        totalUpdateTimesMs[ringIndex] = lastTotalTargetingTimeMs;
        physicsCalcTimesMs[ringIndex] = lastPhysicsAndTrajectoryTimeMs;
        ringIndex = (ringIndex + 1) % WINDOW_SIZE;

        if (sampleCount < WINDOW_SIZE) {
            sampleCount++;
            double sumTot = 0.0;
            double sumPhys = 0.0;
            for (int i = 0; i < sampleCount; i++) {
                sumTot += totalUpdateTimesMs[i];
                sumPhys += physicsCalcTimesMs[i];
            }
            rollingAvgTotalTimeMs = sumTot / sampleCount;
            rollingAvgPhysicsTimeMs = sumPhys / sampleCount;
        } else {
            rollingAvgTotalTimeMs += (lastTotalTargetingTimeMs - oldTotal) / WINDOW_SIZE;
            rollingAvgPhysicsTimeMs += (lastPhysicsAndTrajectoryTimeMs - oldPhys) / WINDOW_SIZE;
        }
    }

    public void resetStatistics() {
        sampleCount = 0;
        ringIndex = 0;
        maxPhysicsTimeMs = 0.0;
        maxTotalTimeMs = 0.0;
        rollingAvgPhysicsTimeMs = 0.0;
        rollingAvgTotalTimeMs = 0.0;
        totalCyclesExecuted = 0L;
        missedComputeDeadlinesCount = 0L;
        missedLoopIntervalDeadlinesCount = 0L;
        lastCycleCompletionNanos = 0L;
    }

    public double getLastSensorLatencyMs() {
        return lastSensorLatencyMs;
    }

    public double getLastLocalizationTimeMs() {
        return lastLocalizationTimeMs;
    }

    public double getLastPhysicsAndTrajectoryTimeMs() {
        return lastPhysicsAndTrajectoryTimeMs;
    }

    public double getLastFlywheelControlTimeMs() {
        return lastFlywheelControlTimeMs;
    }

    public double getLastTotalTargetingTimeMs() {
        return lastTotalTargetingTimeMs;
    }

    public double getRollingAvgPhysicsTimeMs() {
        return rollingAvgPhysicsTimeMs;
    }

    public double getRollingAvgTotalTimeMs() {
        return rollingAvgTotalTimeMs;
    }

    public double getMaxPhysicsTimeMs() {
        return maxPhysicsTimeMs;
    }

    public double getMaxTotalTimeMs() {
        return maxTotalTimeMs;
    }

    public double getUpdateFrequencyHz() {
        return updateFrequencyHz;
    }

    public long getTotalCyclesExecuted() {
        return totalCyclesExecuted;
    }

    public long getMissedComputeDeadlinesCount() {
        return missedComputeDeadlinesCount;
    }

    public long getMissedLoopIntervalDeadlinesCount() {
        return missedLoopIntervalDeadlinesCount;
    }
}
