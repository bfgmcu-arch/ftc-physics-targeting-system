package org.firstinspires.ftc.teamcode.targeting;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileInputStream;
import java.io.FileOutputStream;
import java.io.FileWriter;
import java.io.IOException;
import java.util.Locale;
import java.util.Properties;
import org.firstinspires.ftc.teamcode.targeting.TargetingTypes.ShotRecord;

/**
 * Non-Volatile Calibration & Shot-Log Persistence Manager.
 *
 * <p>Persists learned physics calibration multipliers ({@code efficiencyScale}, {@code dragScale},
 * {@code magnusScale}, {@code pitchBiasRad}) across Autonomous-to-TeleOp transitions and Control
 * Hub power cycles, and exports recorded {@link ShotRecord} data to CSV for post-match analysis.
 *
 * <p>Default REV Control Hub directory: {@code /sdcard/FIRST/targeting/}
 */
public final class CalibrationPersistenceManager {

    public static final String DEFAULT_CONTROL_HUB_DIR = "/sdcard/FIRST/targeting";
    public static final String CALIBRATION_FILENAME = "targeting_calibration.properties";
    public static final String SHOT_LOG_CSV_FILENAME = "shot_calibration_log.csv";

    private CalibrationPersistenceManager() {}

    /**
     * Saves the current calibrated multipliers from {@link PhysicsEngine} to the specified file.
     * Call during {@code stop()} of an OpMode or after an offline calibration run.
     */
    public static boolean saveCalibrationParameters(PhysicsEngine physics, File targetFile) {
        try {
            File parent = targetFile.getParentFile();
            if (parent != null && !parent.exists()) {
                parent.mkdirs();
            }
            Properties props = new Properties();
            props.setProperty(
                    "calibratedEfficiencyScale",
                    Double.toString(physics.getCalibratedEfficiencyScale()));
            props.setProperty(
                    "calibratedDragScale",
                    Double.toString(physics.getCalibratedDragScale()));
            props.setProperty(
                    "calibratedMagnusScale",
                    Double.toString(physics.getCalibratedMagnusScale()));
            props.setProperty(
                    "calibratedPitchBiasRad",
                    Double.toString(physics.getCalibratedPitchBiasRad()));

            try (FileOutputStream fos = new FileOutputStream(targetFile)) {
                props.store(fos, "FTC Physics Targeting System Calibrated Parameters");
            }
            return true;
        } catch (IOException | SecurityException e) {
            return false;
        }
    }

    /**
     * Loads previously saved calibration multipliers into {@link PhysicsEngine}.
     * Call during {@code init()} of Autonomous and TeleOp OpModes.
     */
    public static boolean loadCalibrationParameters(PhysicsEngine physics, File sourceFile) {
        if (sourceFile == null || !sourceFile.exists() || !sourceFile.isFile()) {
            return false;
        }
        try (FileInputStream fis = new FileInputStream(sourceFile)) {
            Properties props = new Properties();
            props.load(fis);

            double effScale =
                    parseDoubleOrDefault(
                            props.getProperty("calibratedEfficiencyScale"),
                            physics.getCalibratedEfficiencyScale());
            double dragScale =
                    parseDoubleOrDefault(
                            props.getProperty("calibratedDragScale"),
                            physics.getCalibratedDragScale());
            double magnusScale =
                    parseDoubleOrDefault(
                            props.getProperty("calibratedMagnusScale"),
                            physics.getCalibratedMagnusScale());
            double pitchBias =
                    parseDoubleOrDefault(
                            props.getProperty("calibratedPitchBiasRad"),
                            physics.getCalibratedPitchBiasRad());

            physics.setCalibrationCorrections(effScale, dragScale, magnusScale, pitchBias);
            return true;
        } catch (IOException | SecurityException e) {
            return false;
        }
    }

    /**
     * Exports all logged {@link ShotRecord} entries from {@link ShotCalibration} to a CSV file.
     */
    public static boolean exportShotRecordsToCsv(ShotCalibration calibration, File csvFile) {
        try {
            File parent = csvFile.getParentFile();
            if (parent != null && !parent.exists()) {
                parent.mkdirs();
            }
            try (BufferedWriter writer = new BufferedWriter(new FileWriter(csvFile, false))) {
                writer.write(
                        "index,robotXM,robotYM,shootingDistanceM,targetHeightM,launchAngleDeg,"
                                + "commandedRpm,actualRpm,batteryVoltageV,"
                                + "predImpactXM,predImpactYM,predImpactZM,"
                                + "actualImpactXM,actualImpactYM,actualImpactZM,"
                                + "totalErrorM,verticalErrorM\n");

                int count = calibration.getRecordCount();
                for (int i = 0; i < count; i++) {
                    ShotRecord r = calibration.getRecordAt(i);
                    writer.write(
                            String.format(
                                    Locale.US,
                                    "%d,%.4f,%.4f,%.4f,%.4f,%.2f,%.1f,%.1f,%.2f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f%n",
                                    i,
                                    r.robotXM,
                                    r.robotYM,
                                    r.shootingDistanceM,
                                    r.targetHeightM,
                                    Math.toDegrees(r.launchAngleRad),
                                    r.commandedFlywheelRpm,
                                    r.actualFlywheelRpm,
                                    r.batteryVoltageV,
                                    r.predictedImpactXM,
                                    r.predictedImpactYM,
                                    r.predictedImpactZM,
                                    r.actualImpactXM,
                                    r.actualImpactYM,
                                    r.actualImpactZM,
                                    r.totalImpactErrorM(),
                                    r.verticalImpactErrorM()));
                }
            }
            return true;
        } catch (IOException | SecurityException e) {
            return false;
        }
    }

    private static double parseDoubleOrDefault(String value, double fallback) {
        if (value == null) {
            return fallback;
        }
        try {
            double parsed = Double.parseDouble(value.trim());
            return Double.isFinite(parsed) ? parsed : fallback;
        } catch (NumberFormatException e) {
            return fallback;
        }
    }
}
