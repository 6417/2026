package frc.robot.calibration;

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.time.Instant;
import java.util.Optional;

import com.fasterxml.jackson.databind.DeserializationFeature;
import com.fasterxml.jackson.databind.ObjectMapper;

import edu.wpi.first.wpilibj.Filesystem;
import frc.robot.Constants;

/**
 * JSON-Persistenz für Shooter-Kalibrierdaten.
 *
 * <p>Ladepriorität:
 * 1) Runtime-Datei (z. B. nach Feldkalibrierung geschrieben),
 * 2) Deploy-Datei als Fallback (mit dem Build ausgelieferter Stand).
 */
public final class CalibrationIO {
    private static final ObjectMapper MAPPER = new ObjectMapper()
            .configure(DeserializationFeature.FAIL_ON_UNKNOWN_PROPERTIES, false);

    private CalibrationIO() {
    }

    /** @return Pfad zur beschreibbaren Runtime-Kalibrierdatei. */
    public static Path runtimeConfigPath() {
        return Paths.get(
                Constants.Calibration.runtimeCalibrationFolder,
                Constants.Calibration.shooterCalibrationFileName);
    }

    /** @return Pfad zur optionalen Deploy-Default-Kalibrierdatei. */
    public static Path deployConfigPath() {
        return Filesystem.getDeployDirectory().toPath()
                .resolve(Constants.Calibration.runtimeCalibrationFolder)
                .resolve(Constants.Calibration.shooterCalibrationFileName);
    }

    /**
     * Lädt zuerst Runtime, dann Deploy.
     *
     * @return erstes gültiges Kalibrierobjekt in Prioritätsreihenfolge
     */
    public static Optional<ShooterCalibrationConfig> loadBestAvailable() {
        Optional<ShooterCalibrationConfig> runtime = loadFromPath(runtimeConfigPath());
        if (runtime.isPresent()) {
            return runtime;
        }
        return loadFromPath(deployConfigPath());
    }

    /**
     * Liest und validiert eine Kalibrierdatei.
     *
     * @param path Quelldatei
     * @return Optional mit validierter Konfiguration
     */
    public static Optional<ShooterCalibrationConfig> loadFromPath(Path path) {
        if (!Files.exists(path)) {
            return Optional.empty();
        }
        try {
            ShooterCalibrationConfig config = MAPPER.readValue(path.toFile(), ShooterCalibrationConfig.class);
            return validate(config) ? Optional.of(config) : Optional.empty();
        } catch (IOException e) {
            return Optional.empty();
        }
    }

    /**
     * Schreibt eine validierte Konfiguration in die Runtime-Datei.
     *
     * <p>Vor dem Schreiben wird ein Zeitstempel ({@code generatedAtIsoUtc}) gesetzt.
     */
    public static boolean writeRuntimeConfig(ShooterCalibrationConfig config) {
        if (!validate(config)) {
            return false;
        }
        try {
            Path outputPath = runtimeConfigPath();
            Path parent = outputPath.getParent();
            if (parent != null) {
                Files.createDirectories(parent);
            }
            ShooterCalibrationConfig toWrite = config.copy();
            toWrite.generatedAtIsoUtc = Instant.now().toString();
            MAPPER.writerWithDefaultPrettyPrinter().writeValue(outputPath.toFile(), toWrite);
            return true;
        } catch (IOException e) {
            return false;
        }
    }

    /**
     * Prüft grundlegende Schema-/Wertekonsistenz.
     *
     * <p>Insbesondere müssen Bias-Knoten strikt aufsteigend und vollständig endlich sein.
     */
    public static boolean validate(ShooterCalibrationConfig config) {
        if (config == null) {
            return false;
        }
        if (!Double.isFinite(config.turretZeroOffsetRad)) {
            return false;
        }
        if (config.biasDistancesM == null || config.biasValues == null) {
            return false;
        }
        if (config.biasDistancesM.length != config.biasValues.length) {
            return false;
        }
        if (config.biasDistancesM.length < 2) {
            return false;
        }
        double prev = Double.NEGATIVE_INFINITY;
        for (int i = 0; i < config.biasDistancesM.length; i++) {
            double x = config.biasDistancesM[i];
            double y = config.biasValues[i];
            if (!Double.isFinite(x) || !Double.isFinite(y)) {
                return false;
            }
            if (x <= prev) {
                return false;
            }
            prev = x;
        }
        return true;
    }
}

