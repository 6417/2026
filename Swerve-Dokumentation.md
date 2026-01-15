# Swerve Dokumentation (Rookie Guide)

Ziel:
- Verstehen, wie der Swerve-Code im Projekt arbeitet, Schritt fuer Schritt.
- Rookies sollen nach der Lektuere jede Zeile im Code erklaeren koennen: was, warum, wie.
- Fokus auf `src/main/java/frc/robot/swerve`, nur minimaler Kontext aus anderen Dateien.

Wie du mit dieser Doku arbeitest:
1) Oeffne die Datei im Editor und lies die Line-by-line Sektion hier.
2) Gehe danach im echten Code Zeile fuer Zeile durch und erklaere sie laut.
3) Mache die Mini-Checks und Stop-and-Think Aufgaben (interaktiv).

Scope / Quellen:
- Hauptquelle: `src/main/java/frc/robot/swerve/*`
- Kontext (nur wenn im Swerve-Code verwendet): `Controls.java`, `RobotContainer.java`, `Robot.java`, `Constants.java`, `Utils.java`

Swerve-System in 30 Sekunden:
- Joystick -> `DriveCommand` liest Eingaben.
- `DriveCommand` skaliert und wandelt in `ChassisSpeeds`.
- `SwerveDrive` verteilt auf 4 Module via Kinematik.
- `SwerveModule` setzt Antriebs- und Winkelmotoren.
- Encoder + Gyro -> Odometry/PoseEstimator.
- Optional: Limelight -> Vision Update in PoseEstimator.

Glossar (Kurz):
- `ChassisSpeeds`: Roboter-Geschwindigkeit (vx, vy, omega).
- `SwerveModuleState`: Ziel fuer ein Modul (Radgeschwindigkeit + Winkel).
- `SwerveModulePosition`: Messwert eines Moduls (Weg + Winkel).
- `Pose2d`: Position (x, y) + Rotation.
- `SwerveDriveKinematics`: Rechnet von Chassis zu Modul und zurueck.
- `SwerveDrivePoseEstimator`: Fusion aus Odometry + Vision.

Minimaler Kontext ausserhalb swerve (nur das Nenoetigste):
- `RobotContainer.java`: erstellt `gyro`, `drive`, `controls` als globale Singletons.
- `Robot.java`: ruft beim Start `resetModulesToAbsolute()` auf.
- `Controls.java`: liefert Joystick, Deadband, Speed-Faktoren und Ausrichtung.
- `Constants.java`: hat `SwerveDrive.configs` und Maximalwerte.
- `Utils.java`: `wrap()` fuer Winkel-Optimierung.

---

## Datei: src/main/java/frc/robot/swerve/DriveCommand.java

Zweck:
- Teleop-Command: liest Joystick, wendet Deadband/Skalierung an, setzt `ChassisSpeeds`.

Line-by-line:
- `package frc.robot.swerve;` -> Ordnet die Klasse in das Package ein.
- `(blank line)` -> Lesbarkeit.
- `import edu.wpi.first.math.geometry.Rotation2d;` -> Rotation fuer Field-Relative Umrechnung.
- `import edu.wpi.first.math.kinematics.ChassisSpeeds;` -> Datentyp fuer vx/vy/omega.
- `import edu.wpi.first.wpilibj2.command.Command;` -> Basisklasse fuer Commands.
- `import frc.fridowpi.utils.Vector2;` -> Eigene 2D-Vektor-Klasse.
- `import frc.robot.RobotContainer;` -> Zugriff auf `controls`, `drive`, `gyro`.
- `import frc.robot.Controls.DriveSpeed;` -> Enum fuer Speed-Modi (nur in Kommentar genutzt).
- `import frc.robot.Constants;` -> Maximalgeschwindigkeiten.
- `import frc.robot.Controls;` -> Deadband-Werte und Control-Mode.
- `(blank line)` -> Lesbarkeit.
- `public class DriveCommand extends Command {` -> Definiert Command-Klasse.
- `    public DriveCommand(SwerveDrive drive) {` -> Konstruktor, braucht Subsystem.
- `        addRequirements(drive);` -> Scheduler weiss: dieses Command braucht `drive` exklusiv.
- `    }` -> Ende Konstruktor.
- `(blank line)` -> Lesbarkeit.
- `    public void execute() {` -> Wird alle 20 ms aufgerufen.
- `(blank line)` -> Lesbarkeit.
- `        var joystick = RobotContainer.controls.driveJoystick;` -> Hole Joystick aus Controls.
- `(blank line)` -> Lesbarkeit.
- `        var x = -joystick.getRawAxis(1);` -> Vor/zurueck (Axis 1), invertiert fuer Logik.
- `        var y = -joystick.getRawAxis(0);` -> Links/rechts (Axis 0), invertiert.
- `        ` -> Leere Zeile fuer Trennung.
- `        var rot = -joystick.getRightX();` -> Drehung ueber rechten Stick, invertiert.
- `(blank line)` -> Lesbarkeit.
- `       /*  if (RobotContainer.controls.controlMode == Controls.ControlMode.SEPARATE_ACCELERATION) {` -> Auskommentierter Modus fuer getrennte Beschleunigung.
- `            xy = xy.normalized().scaled(joystick.getRawAxis(4));` -> Wuerde Richtung normieren und Skalar von Axis 4 nehmen.
- `        }*/` -> Kommentarblock Ende.
- `(blank line)` -> Lesbarkeit.
- `        // Apply deadband4` -> Kommentar: Deadband anwenden (Tippfehler "4").
- `        x = applyDeadband(x, Controls.deadBandTurn);` -> Kleine Werte auf 0 setzen.
- `        y = applyDeadband(y, Controls.deadBandTurn);` -> Deadband fuer y.
- `        rot = applyDeadband(rot, Controls.deadBandTurn);` -> Deadband fuer Rotation.
- `        var xy = new Vector2(x, y);` -> Kombiniere in 2D-Vektor.
- `(blank line)` -> Lesbarkeit.
- `       ` -> Leere Zeile (optische Trennung).
- `        // Apply slew rate` -> Kommentar: Beschleunigungs-Limit (aktuell auskommentiert).
- `        /*` -> Start Kommentarblock.
- `         * if (Controls.isSlewRateLimited()) {` -> Wuerde nur aktiv wenn SlewRate an.
- `         * var xLimited = xLimiter.calculate(abs(xy.x)) * signum(xy.x);` -> Beispiel fuer Begrenzung in x.
- `         * var yLimited = yLimiter.calculate(abs(xy.y)) * signum(xy.y);` -> Beispiel fuer Begrenzung in y.
- `         * // rot = rotLimiter.calculate(abs(rot)) * signum(rot);` -> Rotation Limit (auskommentiert).
- `         * if (Double.isNaN(xLimited) || Double.isNaN(yLimited)) {` -> NaN Schutz.
- `         * xLimiter.reset(xy.x);` -> Reset Limiters bei NaN.
- `         * yLimiter.reset(xy.y);` -> Reset Limiters bei NaN.
- `         * } else {` -> Normalfall.
- `         * xy = new Vector2(xLimited, yLimited);` -> Verwende limitierte Werte.
- `         * }` -> Ende else.
- `         * }` -> Ende if.
- `         */` -> Kommentarblock Ende.
- `(blank line)` -> Lesbarkeit.
- `        /*// Convert to velocity` -> Kommentarblock fuer Speed-Faktor per Stick-Taste.
- `        if(joystick.rightStick().getAsBoolean()) {` -> Falls rechter Stick gedrueckt.
- `            RobotContainer.controls.setActiveSpeedFactor(DriveSpeed.SLOW);` -> Wechsel auf SLOW.
- `        } else{` -> Sonst.
- `            RobotContainer.controls.setActiveSpeedFactor(DriveSpeed.FAST);` -> Wechsel auf FAST.
- `        }*/` -> Kommentarblock Ende.
- `(blank line)` -> Lesbarkeit.
- `        xy.scale(Constants.SwerveDrive.maxSpeed * RobotContainer.controls.speedFactors.get(RobotContainer.controls.getActiveSpeedFactor()));` -> Skaliert Geschwindigkeit mit MaxSpeed * aktivem Faktor.
- `        rot *= Constants.SwerveDrive.maxTurnSpeed;` -> Skaliert Drehgeschwindigkeit.
- `        setChassisSpeeds(xy, rot);` -> Uebergabe an SwerveDrive.
- `    }` -> Ende execute().
- `(blank line)` -> Lesbarkeit.
- `    public static Vector2 applyDeadband(Vector2 xy, double deadBand) {` -> Deadband fuer Vektor.
- `        return xy.normalized().scaled(applyDeadband(xy.magnitude(), deadBand));` -> Normiert Richtung, skaliert mit deadbanded Magnitude.
- `    }` -> Ende Methode.
- `(blank line)` -> Lesbarkeit.
- `    public static double applyDeadband(double x, double deadBand) {` -> Deadband fuer Skalar.
- `        return Math.abs(x) < deadBand ? 0 : (Math.abs(x) - deadBand) / (1 - deadBand) * Math.signum(x);` -> Lineare Totzone, danach Reskalierung.
- `    }` -> Ende Methode.
- `(blank line)` -> Lesbarkeit.
- `    private void setChassisSpeeds(Vector2 vxy, double vRot) {` -> Hilfsmethode: setzt Field/Robot oriented.
- `        switch (RobotContainer.controls.driveOrientation) {` -> Auswahl nach Controls-Setting.
- `            case Forwards:` -> Roboter-Orientierung vorne.
- `                RobotContainer.drive.setChassisSpeeds(ChassisSpeeds.fromFieldRelativeSpeeds(vxy.x, vxy.y,` -> Field-relative zu Robot-relative mit 0 Grad.
- `                        vRot, new Rotation2d(0.0)));` -> Rotation=0 bedeutet "vorne".
- `                break;` -> Ende case.
- `            case Backwards:` -> Um 180 Grad gedreht fahren.
- `                RobotContainer.drive.setChassisSpeeds(ChassisSpeeds.fromFieldRelativeSpeeds(vxy.x, vxy.y,` -> Field-relative zu Robot-relative mit 180 Grad.
- `                        vRot, Rotation2d.fromRadians(Math.PI)));` -> Rotation=pi.
- `                break;` -> Ende case.
- `            case FieldOriented:` -> Field-Oriented nach Gyro.
- `                RobotContainer.drive.setChassisSpeeds(ChassisSpeeds.fromFieldRelativeSpeeds(vxy.x, vxy.y,` -> Field-relative zu Robot-relative mit Gyro.
- `                        vRot, RobotContainer.gyro.getRotation2d()));` -> Gyro Rotation als Referenz.
- `                break;` -> Ende case.
- `        }` -> Ende switch.
- `    }` -> Ende setChassisSpeeds().
- `}` -> Ende Klasse.

Mini-Check (interaktiv):
- Warum sind x/y/rot negativ? (Denke an Joystick-Achsen und Vorwaertsrichtung.)
- Was passiert, wenn `driveOrientation = FieldOriented` und der Gyro 90 Grad meldet?
- Suche die Stelle, an der MaxSpeed in reale m/s umgerechnet wird.

---
## Datei: src/main/java/frc/robot/swerve/SwerveDrive.java

Zweck:
- Zentrales Subsystem: verteilt ChassisSpeeds auf Module, berechnet Pose, verarbeitet Vision.

Line-by-line:
- `package frc.robot.swerve;` -> Package.
- `(blank line)` -> Lesbarkeit.
- `import static edu.wpi.first.units.Units.Meter;` -> Statischer Import (in dieser Datei ungenutzt).
- `import static edu.wpi.first.units.Units.Meters;` -> Statischer Import (ungenutzt).
- `import static edu.wpi.first.units.Units.MetersPerSecond;` -> Statischer Import (ungenutzt).
- `import static edu.wpi.first.units.Units.Radians;` -> Statischer Import (ungenutzt).
- `import static edu.wpi.first.units.Units.Volt;` -> Statischer Import (ungenutzt).
- `import static edu.wpi.first.units.Units.Volts;` -> Statischer Import (ungenutzt).
- `(blank line)` -> Lesbarkeit.
- `import edu.wpi.first.math.VecBuilder;` -> Vektor-Builder fuer Noise-Parameter.
- `import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;` -> Pose-Fusion (Odometry + Vision).
- `import edu.wpi.first.math.geometry.Pose2d;` -> Pose Datentyp.
- `import edu.wpi.first.math.geometry.Rotation2d;` -> Rotation Datentyp.
- `import edu.wpi.first.math.kinematics.ChassisSpeeds;` -> Chassis-Velocity.
- `import edu.wpi.first.math.kinematics.SwerveDriveKinematics;` -> Kinematik fuer swerve.
- `import edu.wpi.first.math.kinematics.SwerveDriveOdometry;` -> Odometry (ungenutzt, PoseEstimator verwendet).
- `import edu.wpi.first.math.kinematics.SwerveModulePosition;` -> Modul-Position.
- `import edu.wpi.first.math.kinematics.SwerveModuleState;` -> Modul-Status (Speed+Angle).
- `import edu.wpi.first.math.util.Units;` -> Einheiten-Helfer (Grad/Rad).
- `import edu.wpi.first.units.measure.MutDistance;` -> Units-Messwert (ungenutzt).
- `import edu.wpi.first.units.measure.MutLinearVelocity;` -> Units-Messwert (ungenutzt).
- `import edu.wpi.first.units.measure.MutVoltage;` -> Units-Messwert (ungenutzt).
- `import edu.wpi.first.units.measure.Voltage;` -> Units-Messwert (ungenutzt).
- `import edu.wpi.first.util.sendable.SendableBuilder;` -> Shuffleboard/NT Properties.
- `import edu.wpi.first.wpilibj.RobotController;` -> RobotController (ungenutzt).
- `import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;` -> Shuffleboard UI.
- `import edu.wpi.first.wpilibj2.command.Command;` -> Command (ungenutzt).
- `import edu.wpi.first.wpilibj2.command.SubsystemBase;` -> Subsystem Basis.
- `import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;` -> SysId (ungenutzt).
- `import frc.fridowpi.motors.FridolinsMotor.IdleMode;` -> IdleMode fuer Motoren.
- `import frc.fridowpi.utils.AccelerationLimiter;` -> Beschleunigungs-Limiter (aktuell ungenutzt).
- `import frc.robot.Constants;` -> Konfig/Maxwerte.
- `import frc.robot.LimelightHelpers;` -> Vision Helfer.
- `import frc.robot.RobotContainer;` -> Zugriff auf Gyro.
- `(blank line)` -> Lesbarkeit.
- `public class SwerveDrive extends SubsystemBase {` -> Subsystem Definition.
- `    public SwerveModule[] modules;` -> Array der 4 Module.
- `    private SwerveDriveKinematics kinematics;` -> Kinematik Rechenobjekt.
- `    public SwerveDrivePoseEstimator poseEstimator;` -> Pose Estimator.
- `(blank line)` -> Lesbarkeit.
- `    public LimelightHelpers.PoseEstimate mt2;` -> Feld fuer Vision Pose (aktuell ungenutzt).
- `(blank line)` -> Lesbarkeit.
- `    // I made a mistake by a factor of 10 when I considered the force, the` -> Kommentar zur Physik.
- `    // accelration should` -> Kommentar.
- `    // be roughly 10 m / s^2 and not 75.` -> Kommentar.
- `    private AccelerationLimiter accelLimiter = new AccelerationLimiter(20, 0.267);` -> Limiter Objekt (aktuell nicht verwendet).
- `(blank line)` -> Lesbarkeit.
- `    ChassisSpeeds lastSpeeds = new ChassisSpeeds();` -> Letzte gesetzte Speeds (werden nicht aktualisiert).
- `(blank line)` -> Lesbarkeit.
- `    public static final int LOC_FL = 0;` -> Index Front Left.
- `    public static final int LOC_FR = 1;` -> Index Front Right.
- `    public static final int LOC_RL = 2;` -> Index Rear Left.
- `    public static final int LOC_RR = 3;` -> Index Rear Right.
- `(blank line)` -> Lesbarkeit.
- `    Thread odometryThread;` -> Thread-Feld (derzeit nicht genutzt).
- `(blank line)` -> Lesbarkeit.
- `    public SwerveDrive(ModuleConfig[] configs) {` -> Konstruktor fuer Subsystem.
- `        String[] moduleNames = new String[4];` -> Namen fuer Module (zur Anzeige).
- `        moduleNames[LOC_FR] = "Front Right";` -> Namen setzen (FR).
- `        moduleNames[LOC_FL] = "Front Left";` -> Namen setzen (FL).
- `        moduleNames[LOC_RR] = "Rear Right";` -> Namen setzen (RR).
- `        moduleNames[LOC_RL] = "Rear Left";` -> Namen setzen (RL).
- `(blank line)` -> Lesbarkeit.
- `        modules = new SwerveModule[4];` -> Array erstellen.
- `        for (int i = 0; i < 4; i++) {` -> Schleife fuer Module.
- `            configs[i].name = moduleNames[i];` -> Namen in Config speichern.
- `            modules[i] = new SwerveModule(configs[i]);` -> Modul instanzieren.
- `            Shuffleboard.getTab("Drive").add("SwerveModule " + moduleNames[i], modules[i]);` -> Modul auf Shuffleboard.
- `        }` -> Ende Schleife.
- `        Shuffleboard.getTab("Drive").add("SwerveDrive", this);` -> SwerveDrive auf Shuffleboard.
- `(blank line)` -> Lesbarkeit.
- `        kinematics = new SwerveDriveKinematics(` -> Kinematik mit Modul-Offsets.
- `                configs[0].moduleOffset,` -> Offset Modul 0.
- `                configs[1].moduleOffset,` -> Offset Modul 1.
- `                configs[2].moduleOffset,` -> Offset Modul 2.
- `                configs[3].moduleOffset);` -> Offset Modul 3.
- `(blank line)` -> Lesbarkeit.
- `        poseEstimator = new SwerveDrivePoseEstimator(kinematics,` -> PoseEstimator erstellen.
- `                RobotContainer.gyro.getRotation2d(),` -> Start-Rotation vom Gyro.
- `                getModulePositions(),` -> Start-Modulpositionen.
- `                new Pose2d(),` -> Start-Pose (0,0,0).
- `                VecBuilder.fill(0.02, 0.02, 0.01),` -> Prozessrauschen (odometry).
- `                VecBuilder.fill(0.1, 0.1, 0.01));` -> Visionrauschen (Initialwerte).
- `(blank line)` -> Lesbarkeit.
- `        // posEstimatorThread = new Thread(this::updateOdometry);` -> Auskommentierter Thread.
- `        // posEstimatorThread.start();` -> Wuerde Thread starten.
- `(blank line)` -> Lesbarkeit.
- `        setDefaultCommand(new DriveCommand(this));` -> Default Command fuer Teleop.
- `(blank line)` -> Lesbarkeit.
- `        /*` -> Kommentarblock Start.
- `         * odometryThread = new Thread(this::updateOdometryThread);` -> Thread fuer odometry.
- `         * odometryThread.start();` -> Starten.
- `         */` -> Kommentarblock Ende.
- `    }` -> Ende Konstruktor.
- `(blank line)` -> Lesbarkeit.
- `    public synchronized void updateOdometryThread() {` -> Thread-Funktion, synchronized.
- `        while (true) {` -> Endlosschleife.
- `            updateOdometry();` -> Odometry updaten.
- `            try {` -> Sleep mit Fehlerhandling.
- `                Thread.sleep(10);` -> 10 ms schlafen.
- `            } catch (InterruptedException e) {` -> Interrupt behandeln.
- `                // TODO Auto-generated catch block` -> Kommentar.
- `                e.printStackTrace();` -> Fehler ausgeben.
- `            }` -> Ende catch.
- `        }` -> Ende while.
- `    }` -> Ende Methode.
- `(blank line)` -> Lesbarkeit.
- `    public void setChassisSpeeds(ChassisSpeeds speeds) {` -> Setzt Zielgeschwindigkeit.
- `        // speeds = ChassisSpeeds.discretize(speeds, 0.02); // remove the skew` -> Optional: Diskretisierung (auskommentiert).
- `(blank line)` -> Lesbarkeit.
- `        /*` -> Kommentarblock Start.
- `         * long timeNow = System.currentTimeMillis();` -> Zeitstempel.
- `         * if (lastSetpointTime > 0) {` -> Beschleunigungs-Limit.
- `         * speeds = accelLimiter.constrain(lastMeasuredSpeeds, speeds,` -> AccelLimiter.
- `         * ((double) (timeNow - lastSetpointTime)) / (double) 1000.0);` -> dt in Sekunden.
- `         * }` -> Ende if.
- `         */` -> Kommentarblock Ende.
- `(blank line)` -> Lesbarkeit.
- `        SwerveModuleState[] moduleStates = kinematics.toSwerveModuleStates(speeds);` -> Rechnet in Modul-States.
- `(blank line)` -> Lesbarkeit.
- `        // SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates,` -> Optionale Begrenzung (auskommentiert).
- `        // Constants.SwerveDrive.maxSpeed);` -> MaxSpeed fuer desaturation.
- `(blank line)` -> Lesbarkeit.
- `        for (int i = 0; i < 4; i++) {` -> Alle Module.
- `            modules[i].setDesiredState(moduleStates[i]);` -> Setzt Ziel fuer jedes Modul.
- `        }` -> Ende Schleife.
- `(blank line)` -> Lesbarkeit.
- `        // lastSpeeds = speeds;` -> Letzte Speeds speichern (auskommentiert).
- `        // lastSetpointTime = timeNow;` -> Zeit merken (auskommentiert).
- `        // lastMeasuredSpeeds = getChassisSpeeds();` -> Messung merken (auskommentiert).
- `    }` -> Ende setChassisSpeeds.
- `(blank line)` -> Lesbarkeit.
- `    public void voltageDrive(double voltage) {` -> Characterization: setze Spannung.
- `        for (int i = 0; i < 4; i++) {` -> Alle Module.
- `            modules[i].setDesiredState(voltage);` -> Setzt Drive-Spannung, Winkel auf 0.
- `        }` -> Ende Schleife.
- `    }` -> Ende voltageDrive.
- `(blank line)` -> Lesbarkeit.
- `    public double getcharecterizedVelocity() {` -> Mittelwert der Modul-Geschwindigkeit.
- `        double avareagevelocity = 0;` -> Summe initialisieren.
- `        for (int i = 0; i < 4; i++) {` -> Alle Module.
- `            avareagevelocity += modules[i].getState().speedMetersPerSecond;` -> Geschwindigkeit addieren.
- `        }` -> Ende Schleife.
- `        return avareagevelocity / 4;` -> Durchschnitt.
- `    }` -> Ende Methode.
- `(blank line)` -> Lesbarkeit.
- `    public double getcharecterizedDistance() {` -> Mittelwert der Modul-Position.
- `        double avareageDistance = 0;` -> Summe.
- `        for (int i = 0; i < 4; i++) {` -> Alle Module.
- `            avareageDistance += modules[i].getPosition().distanceMeters;` -> Weg addieren.
- `        }` -> Ende Schleife.
- `        return avareageDistance / 4;` -> Durchschnitt.
- `    }` -> Ende Methode.
- `(blank line)` -> Lesbarkeit.
- `    public double getcharecterizedVoltage() {` -> Mittelwert der Modul-Spannung.
- `        double avareageVoltage = 0;` -> Summe.
- `        for (int i = 0; i < 4; i++) {` -> Alle Module.
- `            avareageVoltage += modules[i].appliedVoltage();` -> Spannung addieren.
- `        }` -> Ende Schleife.
- `        return avareageVoltage / 4;` -> Durchschnitt.
- `    }` -> Ende Methode.
- `(blank line)` -> Lesbarkeit.
- `    public ChassisSpeeds getChassisSpeeds() {` -> Rueckrechnung aus Modul-States.
- `        return kinematics.toChassisSpeeds(modules[0].getState(), modules[1].getState(),` -> Kinematik rueckrechnen.
- `                modules[2].getState(), modules[3].getState());` -> Alle vier Module.
- `    }` -> Ende Methode.
- `(blank line)` -> Lesbarkeit.
- `    public void periodic() {` -> WPILib periodic, alle 20 ms.
- `        updateOdometry();` -> Odometry updaten.
- `        LimelightHelpers.SetRobotOrientation(Constants.Limelight.limelightID,` -> Limelight Orientierung fuer MegaTag2.
- `                poseEstimator.getEstimatedPosition().getRotation().getDegrees(), 0, 0, 0, 0, 0);` -> Yaw setzen, andere 0.
- `    }` -> Ende periodic.
- `(blank line)` -> Lesbarkeit.
- `    public Pose2d getPose() {` -> Getter fuer Pose.
- `        return poseEstimator.getEstimatedPosition();` -> Pose aus Estimator.
- `    }` -> Ende Methode.
- `(blank line)` -> Lesbarkeit.
- `    public synchronized void updateOdometry() {` -> Odometry Update (synchronized).
- `        poseEstimator.update(` -> Update mit Gyro + Modulen.
- `                RobotContainer.gyro.getRotation2d(),` -> Gyro Rotation.
- `                new SwerveModulePosition[] {` -> Array der Modulen.
- `                        modules[LOC_FL].getPosition(),` -> FL.
- `                        modules[LOC_FR].getPosition(),` -> FR.
- `                        modules[LOC_RL].getPosition(),` -> RL.
- `                        modules[LOC_RR].getPosition()` -> RR.
- `                });` -> Array Ende.
- `    }` -> Ende updateOdometry.
- `(blank line)` -> Lesbarkeit.
- `    public void addVisionToOdometry() {` -> Vision Update (wird extern aufgerufen).
- `        // MOVED TO PERIODIC` -> Kommentar.
- `        /*` -> Kommentarblock Start.
- `         * LimelightHelpers.SetRobotOrientation(Constants.Limelight.limelightID,` -> wuerde die Orientierung setzen.
- `         * poseEstimator.getEstimatedPosition().getRotation().getDegrees(), 0, 0, 0, 0,` -> Parameter.
- `         * 0);` -> Parameter.
- `         */` -> Kommentarblock Ende.
- `(blank line)` -> Lesbarkeit.
- `        LimelightHelpers.PoseEstimate lime1 = LimelightHelpers` -> Hole Pose von Limelight.
- `                .getBotPoseEstimate_wpiBlue_MegaTag2(Constants.Limelight.limelightID); // We use MegaTag 1 because 2 has` -> MegaTag2 wird genutzt, Kommentar sagt aber "1".
- `                                                                                       // problems` -> Kommentarfortsetzung.
- `        addLimeLightMeasurementToPoseEstimation(lime1);` -> Pose in Estimator eintragen.
- `(blank line)` -> Lesbarkeit.
- `    }` -> Ende addVisionToOdometry.
- `(blank line)` -> Lesbarkeit.
- `    private void addLimeLightMeasurementToPoseEstimation(LimelightHelpers.PoseEstimate lime) {` -> Hilfsmethode fuer Vision.
- `        if (lime == null)` -> Null-Check.
- `            return;` -> Abbruch wenn null.
- `(blank line)` -> Lesbarkeit.
- `        // TODO: tune these values` -> Parameter muessen getunt werden.
- `        final double farDist = 0.8;` -> Max Distanz in Metern.
- `        final double maxRotationSpeed = 450;` -> Max Rotationsgeschwindigkeit (deg/s) fuer Vision.
- `        final double narrowAngleThreshold = Units.degreesToRadians(5);` -> Winkelgrenze (rad).
- `(blank line)` -> Lesbarkeit.
- `        final boolean isRobotSpinningFast = Math` -> bool: zu hohe Drehgeschwindigkeit?
- `                .abs(RobotContainer.gyro.getAngularVelocityZWorld().getValueAsDouble()) > maxRotationSpeed;` -> Vergleich mit Limit.
- `        final boolean isTagInNarrowAngle = Math` -> bool: Winkel diff klein genug?
- `                .abs(getPose().getRotation().getDegrees()` -> Differenz zwischen Pose und Limelight Pose...
- `                        - lime.pose.getRotation().getDegrees()) <= narrowAngleThreshold;` -> ... und Schwellwert.
- `(blank line)` -> Lesbarkeit.
- `        if (isRobotSpinningFast || !isTagInNarrowAngle) {` -> Wenn zu schnell oder Winkel gross.
- `            // if our angular velocity is greater than 720 degrees` -> Kommentar (720 steht hier, Code nutzt 450).
- `            // per second, ignore vision updates` -> Kommentar.
- `            return;` -> Vision ignorieren.
- `        }` -> Ende if.
- `(blank line)` -> Lesbarkeit.
- `        if (lime.tagCount == 0)` -> Keine Tags?
- `            return;` -> Abbruch.
- `(blank line)` -> Lesbarkeit.
- `        if (lime.avgTagDist > farDist || lime.avgTagDist < 0.0)` -> Distanz zu gross oder ungueltig?
- `            return;` -> Abbruch.
- `(blank line)` -> Lesbarkeit.
- `        poseEstimator.setVisionMeasurementStdDevs(` -> Setze Vision-Rauschen.
- `                VecBuilder.fill(0.7 * (1.0 - lime.avgTagDist / farDist),` -> X noise skaliert mit Distanz.
- `                        0.7 * (1.0 - lime.avgTagDist / farDist),` -> Y noise skaliert.
- `                        Units.degreesToRadians(30)));` -> Rot noise fest 30 deg.
- `        poseEstimator.addVisionMeasurement(` -> Fuege Vision Pose hinzu.
- `                lime.pose,` -> Pose von Limelight.
- `                lime.timestampSeconds);` -> Zeitpunkt der Messung.
- `    }` -> Ende addLimeLightMeasurementToPoseEstimation.
- `(blank line)` -> Lesbarkeit.
- `    public void resetOdoemetry(Pose2d newPose) {` -> Reset Pose (Tippfehler im Namen).
- `        poseEstimator.resetPosition(RobotContainer.gyro.getRotation2d(), getModulePositions(), newPose);` -> Reset mit Gyro + Modulen.
- `    }` -> Ende Methode.
- `(blank line)` -> Lesbarkeit.
- `    public void stopMotors() {` -> Stoppt alle Module.
- `        for (var module : modules) {` -> foreach.
- `            module.stopMotors();` -> Stop.
- `        }` -> Ende Schleife.
- `    }` -> Ende Methode.
- `(blank line)` -> Lesbarkeit.
- `    public void resetModulesToAbsolute() {` -> Winkel-Motoren auf absolute Encoder setzen.
- `        for (SwerveModule module : modules) {` -> foreach.
- `            module.resetToAbsolute();` -> Kalibriert Winkel.
- `        }` -> Ende Schleife.
- `    }` -> Ende Methode.
- `(blank line)` -> Lesbarkeit.
- `    public void setIdleMode(IdleMode mode) {` -> Setzt IdleMode.
- `        for (var module : modules) {` -> foreach.
- `            module.setIdleMode(mode);` -> Nur Drive-Motor IdleMode.
- `        }` -> Ende Schleife.
- `    }` -> Ende Methode.
- `(blank line)` -> Lesbarkeit.
- `    public SwerveModulePosition[] getModulePositions() {` -> Liefert Array von Modul-Positionen.
- `        return new SwerveModulePosition[] {` -> Neues Array.
- `                modules[0].getPosition(),` -> Modul 0 (FL).
- `                modules[1].getPosition(),` -> Modul 1 (FR).
- `                modules[2].getPosition(),` -> Modul 2 (RL).
- `                modules[3].getPosition(),` -> Modul 3 (RR).
- `        };` -> Array Ende.
- `    }` -> Ende Methode.
- `(blank line)` -> Lesbarkeit.
- `    @Override` -> Marker: Override von Sendable.
- `    public void initSendable(SendableBuilder builder) {` -> Shuffleboard Properties.
- `        builder.addDoubleProperty("Chassis speed vx [mps]", () -> lastSpeeds.vxMetersPerSecond, null);` -> Anzeige vx (aktuell bleibt 0).
- `        builder.addDoubleProperty("Chassis speed vy [mps]", () -> lastSpeeds.vyMetersPerSecond, null);` -> Anzeige vy.
- `        builder.addDoubleProperty("Chassis speed omega [rad p s]", () -> lastSpeeds.omegaRadiansPerSecond, null);` -> Anzeige omega.
- `        builder.addDoubleProperty("pose estimator pos x [m]", () -> poseEstimator.getEstimatedPosition().getX(),` -> Zeigt X.
- `                (double posX) -> {` -> Setter fuer X.
- `                    Pose2d currentPos = poseEstimator.getEstimatedPosition();` -> Hole aktuelle Pose.
- `                    currentPos = new Pose2d(posX, currentPos.getY(), currentPos.getRotation());` -> Neue Pose mit neuer X.
- `                    poseEstimator.resetPose(currentPos);` -> Pose im Estimator setzen.
- `                });` -> Ende setter.
- `        builder.addDoubleProperty("pose estimator pos y [m]", () -> poseEstimator.getEstimatedPosition().getY(),` -> Zeigt Y.
- `                (double posY) -> {` -> Setter fuer Y.
- `                    Pose2d currentPos = poseEstimator.getEstimatedPosition();` -> Aktuelle Pose.
- `                    currentPos = new Pose2d(currentPos.getX(), posY, currentPos.getRotation());` -> Neue Pose mit neuer Y.
- `                    poseEstimator.resetPose(currentPos);` -> Pose setzen.
- `                });` -> Ende setter.
- `        builder.addDoubleProperty("pose estimator rot [deg]",` -> Zeigt Rotation.
- `                () -> poseEstimator.getEstimatedPosition().getRotation().getDegrees(), (double rotationDeg) -> {` -> Setter fuer Rotation.
- `                    Pose2d currentPos = poseEstimator.getEstimatedPosition();` -> Aktuelle Pose.
- `                    Rotation2d newRot = new Rotation2d(rotationDeg);` -> Neue Rotation.
- `                    currentPos = new Pose2d(currentPos.getX(), currentPos.getY(), newRot);` -> Neue Pose.
- `                });` -> Hinweis: resetPose fehlt, Rotation wird nicht gesetzt.
- `    }` -> Ende initSendable.
- `}` -> Ende Klasse.

Mini-Check (interaktiv):
- Welche Teile sind aktuell auskommentiert und haben keinen Effekt?
- Warum wird `lastSpeeds` in Shuffleboard angezeigt, aber nie aktualisiert?
- Welche Reihenfolge haben die Module im Odometry-Update?

---
## Datei: src/main/java/frc/robot/swerve/SwerveModule.java

Zweck:
- Abstraktion fuer ein einzelnes Swerve-Modul (Drive + Angle + Encoder).

Line-by-line:
- `package frc.robot.swerve;` -> Package.
- `(blank line)` -> Lesbarkeit.
- `import edu.wpi.first.math.geometry.Rotation2d;` -> Winkeltyp.
- `import edu.wpi.first.math.kinematics.SwerveModulePosition;` -> Positionstyp.
- `import edu.wpi.first.math.kinematics.SwerveModuleState;` -> State (Speed+Angle).
- `import edu.wpi.first.util.sendable.Sendable;` -> Sendable Interface.
- `import edu.wpi.first.util.sendable.SendableBuilder;` -> Builder fuer Shuffleboard.
- `import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;` -> SmartDashboard Zugriff.
- `import frc.fridowpi.sensors.AnalogEncoder;` -> Absolut-Encoder.
- `import frc.robot.Utils;` -> wrap() fuer Winkel.
- `import frc.fridowpi.motors.FridoFalcon500v6;` -> Drive Motor Wrapper.
- `import frc.fridowpi.motors.FridolinsMotor;` -> Motor Interface.
- `import frc.fridowpi.motors.FridolinsMotor.IdleMode;` -> IdleMode enum.
- `(blank line)` -> Lesbarkeit.
- `public class SwerveModule implements Sendable {` -> Modul ist Sendable.
- `    private String moduleName;` -> Modulname (aktuell nie gesetzt).
- `    private FridoFalcon500v6 driveMotor;` -> Drive Motor.
- `    private FridolinsMotor angleMotor;` -> Angle Motor.
- `    private AnalogEncoder absoluteEncoder;` -> Absolut-Encoder.
- `(blank line)` -> Lesbarkeit.
- `    private ModuleConfig config;` -> Konfigurationsdaten.
- `(blank line)` -> Lesbarkeit.
- `    private Rotation2d lastAngle;` -> Letzter Winkel, fuer Stabilitaet.
- `(blank line)` -> Lesbarkeit.
- `    public SwerveModule(ModuleConfig config) {` -> Konstruktor.
- `        this.config = config;` -> Speichert Config.
- `        absoluteEncoder = config.makeAbsoluteEncoder();` -> Encoder bauen.
- `        absoluteEncoder.setPositionOffset(config.absEncoderOffset);` -> Offset setzen.
- `        driveMotor = config.makeDriveMotor();` -> Drive Motor bauen.
- `        angleMotor = config.makeAngleMotor();` -> Angle Motor bauen.
- `(blank line)` -> Lesbarkeit.
- `        lastAngle = getAbsEncoderRotation();` -> Initialwinkel vom Absolutencoder.
- `(blank line)` -> Lesbarkeit.
- `        resetToAbsolute();` -> Angle Motor Encoder auf Absolutwert setzen.
- `    }` -> Ende Konstruktor.
- `(blank line)` -> Lesbarkeit.
- `    public void stopMotors() {` -> Stoppt beide Motoren.
- `        driveMotor.stopMotor();` -> Drive stop.
- `        angleMotor.stopMotor();` -> Angle stop.
- `    }` -> Ende Methode.
- `(blank line)` -> Lesbarkeit.
- `    public void setIdleMode(IdleMode mode) {` -> IdleMode setzen (nur Drive).
- `        driveMotor.setIdleMode(mode);` -> Angle Motor bleibt unveraendert.
- `    }` -> Ende Methode.
- `(blank line)` -> Lesbarkeit.
- `    public void resetToAbsolute() {` -> Kalibriert Angle Motor.
- `        double position = (1 - absoluteEncoder.get()) * config.encoderThicksToRotationNEO * config.angleGearboxRatio;` -> Absolutwert -> Motorposition.
- `        angleMotor.setEncoderPosition(position);` -> Setzt Encoder.
- `    }` -> Ende Methode.
- `(blank line)` -> Lesbarkeit.
- `    public void setDesiredState(SwerveModuleState desiredState) {` -> Kernmethode: Ziel setzen.
- `        desiredState = CTREModuleState.optimize(desiredState, getRotation());` -> Optimierung fuer minimalen Drehweg.
- `(blank line)` -> Lesbarkeit.
- `        double desiredVelocity = (desiredState.speedMetersPerSecond / config.wheelCircumference)` -> MPS -> RPS.
- `                * config.driveGearboxRatio` -> Beruecksichtigt Gearbox.
- `                * config.encoderVelocityToRPSFalcon;` -> Umrechnungsfaktor.
- `        driveMotor.setVelocity(desiredVelocity);` -> Setzt Drive Velocity.
- `(blank line)` -> Lesbarkeit.
- `        Rotation2d angle = (Math.abs(desiredState.speedMetersPerSecond) <= (config.maxSpeed * 0.01))` -> Wenn fast still.
- `                ? lastAngle` -> Dann Winkel halten.
- `                : desiredState.angle; ` -> Sonst Zielwinkel.
- `(blank line)` -> Lesbarkeit.
- `        lastAngle = angle;` -> Letzten Winkel speichern.
- `(blank line)` -> Lesbarkeit.
- `        SmartDashboard.putNumber("Target chassis speed", desiredState.speedMetersPerSecond);` -> Telemetrie.
- `(blank line)` -> Lesbarkeit.
- `        double motorPos = angleMotor.getEncoderTicks() / config.angleGearboxRatio;` -> Motorposition in Rotationen.
- `        double delta = Utils.wrap(lastAngle.getRotations() - motorPos);` -> Kleinster Winkel-Delta (-0.5..0.5).
- `        angleMotor.setPosition((motorPos + delta) * config.angleGearboxRatio);` -> Zielposition fuer Angle Motor.
- `    }` -> Ende setDesiredState.
- `(blank line)` -> Lesbarkeit.
- `    public void setDesiredState(double voltage) {` -> Characterization: Spannung direkt.
- `        ` -> Leere Zeile.
- `        driveMotor.setVoltage(voltage);` -> Drive mit Spannung.
- `(blank line)` -> Lesbarkeit.
- `        angleMotor.setPosition(0);` -> Angle Motor auf 0.
- `    }` -> Ende Methode.
- `(blank line)` -> Lesbarkeit.
- `(blank line)` -> Lesbarkeit.
- `(blank line)` -> Lesbarkeit.
- `    public double getVelocityMPS() {` -> Geschwindigkeit in m/s.
- `        return getVelocityRPS() * config.wheelCircumference;` -> RPS -> MPS.
- `    }` -> Ende Methode.
- `(blank line)` -> Lesbarkeit.
- `    public double getVelocityRPS() {` -> Geschwindigkeit in RPS.
- `        return driveMotor.getEncoderVelocity() / config.encoderVelocityToRPSFalcon / config.driveGearboxRatio;` -> Umrechnen.
- `    }` -> Ende Methode.
- `(blank line)` -> Lesbarkeit.
- `    public double appliedVoltage(){` -> Abgegebene Spannung.
- `        return driveMotor.getAppliedVoltage();` -> Motorwert.
- `    }` -> Ende Methode.
- `(blank line)` -> Lesbarkeit.
- `    public String getName() {` -> Getter Name.
- `        return moduleName;` -> Rueckgabe (aktuell leer).
- `    }` -> Ende Methode.
- `(blank line)` -> Lesbarkeit.
- `    public Rotation2d getAbsEncoderRotation() {` -> Absolutwinkel.
- `        return Rotation2d.fromRotations(absoluteEncoder.get());` -> Encoderwerte in Rotation2d.
- `    }` -> Ende Methode.
- `(blank line)` -> Lesbarkeit.
- `    public Rotation2d getRotation() {` -> Aktueller Winkel.
- `        return getState().angle;` -> Aus State holen.
- `    }` -> Ende Methode.
- `(blank line)` -> Lesbarkeit.
- `    public SwerveModulePosition getPosition() {` -> Position = Weg + Winkel.
- `        double position = (driveMotor.getEncoderTicks() / config.encoderThicksToRotationFalcon` -> Ticks -> Rot.
- `                / config.driveGearboxRatio) * config.wheelCircumference;` -> Rot -> Meter.
- `        Rotation2d angle = Rotation2d.fromRotations(` -> Winkel aus Angle Encoder.
- `                angleMotor.getEncoderTicks() / config.encoderThicksToRotationNEO / config.angleGearboxRatio);` -> Umrechnung.
- `        return new SwerveModulePosition(position, angle);` -> Neues Objekt.
- `    }` -> Ende Methode.
- `(blank line)` -> Lesbarkeit.
- `    public SwerveModuleState getState() {` -> State = Speed + Winkel.
- `        double velocity = (driveMotor.getEncoderVelocity() / config.encoderVelocityToRPSFalcon` -> Ticks/s -> Rot/s.
- `                / config.driveGearboxRatio) * config.wheelCircumference;` -> Rot/s -> m/s.
- `        Rotation2d angle = Rotation2d.fromRotations(` -> Winkel aus Angle Encoder.
- `                angleMotor.getEncoderTicks() / config.encoderThicksToRotationNEO / config.angleGearboxRatio);` -> Umrechnung.
- `        return new SwerveModuleState(velocity, angle);` -> Neues Objekt.
- `    }` -> Ende Methode.
- `(blank line)` -> Lesbarkeit.
- `    @Override` -> Sendable override.
- `    public void initSendable(SendableBuilder builder) {` -> Shuffleboard Properties.
- `        builder.addDoubleProperty("Angle motor ticks", () -> angleMotor.getEncoderTicks(), null);` -> Angle Encoder.
- `        builder.addDoubleProperty("angle motor angle [deg]", () -> getRotation().getDegrees(), null);` -> Winkel in Grad.
- `        builder.addDoubleProperty("drive vel [mps]", () -> getState().speedMetersPerSecond, null);` -> Geschwindigkeit.
- `        builder.addDoubleProperty("abs encoder raw", () -> absoluteEncoder.getRaw(), null);` -> Rohwert Encoder.
- `        builder.addDoubleProperty("abs encoder value", () -> absoluteEncoder.get(), null);` -> Normalisierter Encoder.
- `        builder.addDoubleProperty("last angle [deg]", () -> lastAngle.getDegrees(), null);` -> Letzter Winkel.
- `    }` -> Ende initSendable.
- `}` -> Ende Klasse.

Mini-Check (interaktiv):
- Warum wird `lastAngle` genutzt, wenn die Geschwindigkeit fast 0 ist?
- Welche Umrechnungen brauchen Gearbox-Ratios?
- Was passiert, wenn `moduleName` nie gesetzt wird?

---
## Datei: src/main/java/frc/robot/swerve/ModuleConfig.java

Zweck:
- Konfigurationsdaten fuer ein Modul + Fabrikmethoden fuer Motoren/Encoder.

Line-by-line:
- `package frc.robot.swerve;` -> Package.
- `(blank line)` -> Lesbarkeit.
- `import com.ctre.phoenix6.configs.CurrentLimitsConfigs;` -> CTRE Stromlimits.
- `import com.ctre.phoenix6.configs.Slot0Configs;` -> CTRE PID/FF Slot.
- `import com.revrobotics.spark.SparkBase;` -> REV Spark Base.
- `import com.revrobotics.spark.config.SparkMaxConfig;` -> Spark Max Config.
- `(blank line)` -> Lesbarkeit.
- `import edu.wpi.first.math.geometry.Translation2d;` -> Modul-Offset Position.
- `import frc.fridowpi.motors.FridoFalcon500v6;` -> Drive Motor.
- `import frc.fridowpi.motors.FridoSparkMax;` -> Angle Motor.
- `import frc.fridowpi.motors.FridolinsMotor;` -> Motor Interface.
- `import frc.fridowpi.motors.FridolinsMotor.FridoFeedBackDevice;` -> Encoder Typ.
- `import frc.fridowpi.motors.FridolinsMotor.IdleMode;` -> IdleMode.
- `import frc.fridowpi.motors.utils.FeedForwardValues;` -> FF Werte.
- `import frc.fridowpi.motors.utils.PidValues;` -> PID Werte.
- `import frc.fridowpi.sensors.AnalogEncoder;` -> Absolut Encoder.
- `(blank line)` -> Lesbarkeit.
- `public class ModuleConfig implements Cloneable {` -> Config kann geklont werden.
- `    public String name;` -> Name des Moduls.
- `    public double maxSpeed;` -> Max Speed pro Modul.
- `    public double wheelCircumference;` -> Radumfang in Metern.
- `    public Translation2d moduleOffset;` -> Position im Roboter.
- `    public double absEncoderOffset;` -> Offset fuer Absolut Encoder.
- `(blank line)` -> Lesbarkeit.
- `    public double driveGearboxRatio;` -> Gearbox Ratio fuer Drive.
- `    public int driveMotorID;` -> CAN ID Drive Motor.
- `    public double driveMotorStallCurrentLimit;` -> Stall Current Limit.
- `    public double driveMotorFreeCurrentLimit;` -> Free Current Limit.
- `    public boolean driveMotorInverted;` -> Invertierung Drive.
- `    public PidValues drivePidValues;` -> PID Werte Drive.
- `    public FeedForwardValues driveFFValues;` -> FF Werte Drive.
- `(blank line)` -> Lesbarkeit.
- `    public int angleMotorID;` -> CAN ID Angle Motor.
- `    public double angleGearboxRatio;` -> Gearbox Ratio Angle.
- `    public int angleMotorStallCurrentLimit;` -> Stall Limit.
- `    public int angleMotorFreeCurrentLimit;` -> Free Limit.
- `    public double angleMotorIzone;` -> I-Zone (Spark).
- `    public boolean angleMotorInverted;` -> Invertierung Angle.
- `    public PidValues anglePidValues;` -> PID Werte Angle.
- `(blank line)` -> Lesbarkeit.
- `    public int encoderChannel;` -> Analog Encoder Channel.
- `    public double encoderPositionOffset;` -> Encoder Offset.
- `(blank line)` -> Lesbarkeit.
- `    public double encoderThicksToRotationFalcon;` -> Ticks->Rotation (Drive).
- `    public double encoderVelocityToRPSFalcon;` -> Ticks/s -> RPS (Drive).
- `    public double encoderThicksToRotationNEO;` -> Ticks->Rotation (Angle).
- `    public double encoderVelocityToRPSNEO;` -> Ticks/s -> RPS (Angle).
- `(blank line)` -> Lesbarkeit.
- `    public ModuleConfig() {` -> Default Konstruktor.
- `        name = null;` -> Default Name.
- `        maxSpeed = 0.0;` -> Default MaxSpeed.
- `        wheelCircumference = 0.0;` -> Default Radumfang.
- `        moduleOffset = null;` -> Default Offset.
- `        absEncoderOffset = 0.0;` -> Default Offset.
- `(blank line)` -> Lesbarkeit.
- `        driveGearboxRatio = 0.0;` -> Default.
- `        driveMotorID = -1; // Invalid ID` -> Ungueltige ID.
- `        driveMotorStallCurrentLimit = 70;` -> Default.
- `        driveMotorFreeCurrentLimit = 40;` -> Default.
- `        driveMotorInverted = false;` -> Default.
- `        drivePidValues = null; // Consider creating an "invalid" PidValues object if null isn't suitable` -> PID leer.
- `        driveFFValues = null;` -> FF leer.
- `(blank line)` -> Lesbarkeit.
- `        angleMotorID = -1; // Invalid ID` -> Ungueltige ID.
- `        angleGearboxRatio = 0.0;` -> Default.
- `        angleMotorStallCurrentLimit = 0;` -> Default.
- `        angleMotorFreeCurrentLimit = 0;` -> Default.
- `        angleMotorIzone = 0.0;` -> Default.
- `        angleMotorInverted = true;` -> Default invertiert.
- `        anglePidValues = null;` -> PID leer.
- `(blank line)` -> Lesbarkeit.
- `        encoderChannel = -1; // Invalid channel` -> Ungueltiger Channel.
- `        encoderPositionOffset = 0.0;` -> Default.
- `(blank line)` -> Lesbarkeit.
- `        encoderThicksToRotationFalcon = 1;` -> Default.
- `        encoderVelocityToRPSFalcon = 1;` -> Default.
- `        encoderThicksToRotationNEO = 1;` -> Default.
- `        encoderVelocityToRPSNEO = 1;` -> Default.
- `    }` -> Ende Konstruktor.
- `(blank line)` -> Lesbarkeit.
- `    public FridoFalcon500v6 makeDriveMotor() {` -> Erstellt Drive Motor.
- `        FridoFalcon500v6 driveMotor = new FridoFalcon500v6(driveMotorID);` -> Motor mit ID.
- `        driveMotor.factoryDefault();` -> Reset.
- `        driveMotor.asTalonFX().getConfigurator().apply(new Slot0Configs().withKP(drivePidValues.kP)` -> Set PID/FF.
- `                .withKS(driveFFValues.kS).withKV(driveFFValues.kV).withKA(driveFFValues.kA));` -> FF Werte.
- `        driveMotor.asTalonFX().getConfigurator()` -> Konfigurator.
- `                .apply(new CurrentLimitsConfigs().withStatorCurrentLimit(driveMotorFreeCurrentLimit)` -> Stromlimits.
- `                        .withSupplyCurrentLimit(driveMotorStallCurrentLimit));` -> Supply Limit.
- `        driveMotor.configEncoder(FridoFeedBackDevice.kBuildin, (int) encoderThicksToRotationFalcon);` -> Encoder Konfig.
- `        driveMotor.setInverted(driveMotorInverted);` -> Invertierung.
- `        driveMotor.setPID(drivePidValues, driveFFValues);` -> PID/FF.
- `        driveMotor.setIdleMode(IdleMode.kBrake);` -> Bremsen.
- `        return driveMotor;` -> Rueckgabe.
- `    }` -> Ende Methode.
- `(blank line)` -> Lesbarkeit.
- `    public FridolinsMotor makeAngleMotor() {` -> Erstellt Angle Motor.
- `        FridoSparkMax angleMotor = new FridoSparkMax(angleMotorID);` -> Spark Max mit ID.
- `        // angleMotor.factoryDefault();` -> Reset auskommentiert.
- `        SparkMaxConfig config = new SparkMaxConfig();` -> Neue Config.
- `        config.smartCurrentLimit(angleMotorStallCurrentLimit, angleMotorFreeCurrentLimit);` -> Stromlimits.
- `        angleMotor.asSparkMax().configure(config, SparkBase.ResetMode.kResetSafeParameters,` -> Config anwenden.
- `                SparkBase.PersistMode.kPersistParameters);` -> Persist.
- `        angleMotor.setInverted(angleMotorInverted);` -> Invertierung.
- `        config.closedLoop.iZone(angleMotorIzone);` -> I-Zone.
- `        angleMotor.setPID(anglePidValues);` -> PID setzen.
- `        return angleMotor;` -> Rueckgabe.
- `    }` -> Ende Methode.
- `(blank line)` -> Lesbarkeit.
- `    public AnalogEncoder makeAbsoluteEncoder() {` -> Erstellt Absolut-Encoder.
- `        AnalogEncoder encoder = new AnalogEncoder(encoderChannel);` -> Neuer Encoder.
- `        encoder.setPositionOffset(encoderPositionOffset);` -> Offset setzen.
- `        return encoder;` -> Rueckgabe.
- `    }` -> Ende Methode.
- `(blank line)` -> Lesbarkeit.
- `    @Override` -> Cloneable override.
- `    public ModuleConfig clone() {` -> Clone Methode.
- `        try {` -> Try fuer Clone.
- `            ModuleConfig cloned = (ModuleConfig) super.clone();` -> Shallow clone.
- `(blank line)` -> Lesbarkeit.
- `            // Deep copy of mutable objects` -> Kommentar.
- `            cloned.moduleOffset = moduleOffset != null ? new Translation2d(moduleOffset.getX(), moduleOffset.getY())` -> Kopiert Offset.
- `                    : null;` -> Null wenn kein Offset.
- `            cloned.drivePidValues = drivePidValues != null ? drivePidValues.clone() : null;` -> PID Kopie.
- `            cloned.driveFFValues = driveFFValues != null ? driveFFValues.clone() : null;` -> FF Kopie.
- `            cloned.anglePidValues = anglePidValues != null ? anglePidValues.clone() : null;` -> PID Kopie.
- `(blank line)` -> Lesbarkeit.
- `            return cloned;` -> Rueckgabe.
- `        } catch (CloneNotSupportedException e) {` -> Fehler.
- `            throw new AssertionError("Cloning not supported", e);` -> Runtime Error.
- `        }` -> Ende catch.
- `    }` -> Ende clone.
- `}` -> Ende Klasse.

Mini-Check (interaktiv):
- Wo wird die `moduleOffset` genutzt und warum ist sie wichtig?
- Warum ist `clone()` notwendig fuer vier Module?

---
## Datei: src/main/java/frc/robot/swerve/CTREModuleState.java

Zweck:
- Optimierung: minimiert Winkel-Aenderung, ggf. dreht Radgeschwindigkeit um.

Line-by-line:
- `package frc.robot.swerve;` -> Package.
- `(blank line)` -> Lesbarkeit.
- `// Copyright (c) FIRST and other WPILib contributors.` -> Kommentar.
- `// Open Source Software; you can modify and/or share it under the terms of` -> Kommentar.
- `// the WPILib BSD license file in the root directory of this project.` -> Kommentar.
- `(blank line)` -> Lesbarkeit.
- `import edu.wpi.first.math.geometry.Rotation2d;` -> Rotation Typ.
- `import edu.wpi.first.math.kinematics.SwerveModuleState;` -> State Typ.
- `(blank line)` -> Lesbarkeit.
- `/** Add your docs here. */` -> Kommentar.
- `public class CTREModuleState {` -> Klasse.
- `    /**` -> Javadoc Start.
- `   * Minimize the change in heading the desired swerve module state would require by potentially` -> Doku Zeile.
- `   * reversing the direction the wheel spins. Customized from WPILib's version to include placing` -> Doku Zeile.
- `   * in appropriate scope for CTRE onboard control.` -> Doku Zeile.
- `   *` -> Doku Zeile.
- `   * @param desiredState The desired state.` -> Parameter.
- `   * @param currentAngle The current module angle.` -> Parameter.
- `   */` -> Javadoc Ende.
- `  public static SwerveModuleState optimize(SwerveModuleState desiredState, Rotation2d currentAngle) {` -> Optimierungsmethode.
- `    double targetAngle = placeInAppropriate0To360Scope(currentAngle.getDegrees(), desiredState.angle.getDegrees());` -> Zielwinkel in naechste 0-360 Spanne.
- `    double targetSpeed = desiredState.speedMetersPerSecond;` -> Zielgeschwindigkeit.
- `    double delta = targetAngle - currentAngle.getDegrees();` -> Differenz.
- `    if (Math.abs(delta) > 90){` -> Falls Delta > 90 Grad.
- `        targetSpeed = -targetSpeed;` -> Fahre rueckwaerts.
- `        targetAngle = delta > 90 ? (targetAngle -= 180) : (targetAngle += 180);` -> Winkel um 180 drehen.
- `    }        ` -> Ende if.
- `    return new SwerveModuleState(targetSpeed, Rotation2d.fromDegrees(targetAngle));` -> Neuer optimierter State.
- `  }` -> Ende optimize.
- `(blank line)` -> Lesbarkeit.
- `  /**` -> Javadoc Start.
- `     * @param scopeReference Current Angle` -> Parameter.
- `     * @param newAngle Target Angle` -> Parameter.
- `     * @return Closest angle within scope` -> Return.
- `     */` -> Javadoc Ende.
- `    public static double placeInAppropriate0To360Scope(double scopeReference, double newAngle) {` -> Hilfsmethode.
- `      double lowerBound;` -> Untere Grenze.
- `      double upperBound;` -> Obere Grenze.
- `      double lowerOffset = scopeReference % 360;` -> Offset in 0..360.
- `      if (lowerOffset >= 0) {` -> Positiver Offset.
- `          lowerBound = scopeReference - lowerOffset;` -> Untere Grenze setzen.
- `          upperBound = scopeReference + (360 - lowerOffset);` -> Obere Grenze setzen.
- `      } else {` -> Negativer Offset.
- `          upperBound = scopeReference - lowerOffset;` -> Obere Grenze.
- `          lowerBound = scopeReference - (360 + lowerOffset);` -> Untere Grenze.
- `      }` -> Ende if/else.
- `      while (newAngle < lowerBound) {` -> Angle in Bereich heben.
- `          newAngle += 360;` -> 360 addieren.
- `      }` -> Ende while.
- `      while (newAngle > upperBound) {` -> Angle in Bereich senken.
- `          newAngle -= 360;` -> 360 abziehen.
- `      }` -> Ende while.
- `      if (newAngle - scopeReference > 180) {` -> Ueber 180?
- `          newAngle -= 360;` -> Naeheren Winkel waehlen.
- `      } else if (newAngle - scopeReference < -180) {` -> Unter -180?
- `          newAngle += 360;` -> Naeheren Winkel waehlen.
- `      }` -> Ende if.
- `      return newAngle;` -> Rueckgabe.
- `  }` -> Ende placeInAppropriate0To360Scope.
- `}` -> Ende Klasse.

Mini-Check (interaktiv):
- Warum ist es ok, die Radgeschwindigkeit umzukehren?
- Was passiert, wenn `delta` genau 180 Grad ist?

---

## Datei: src/main/java/frc/robot/swerve/FridoPathplanner.java

Zweck:
- Beispiel-Setup fuer PathPlanner (aktuell komplett auskommentiert).
- Das File hat aktuell keinen Einfluss auf den Code (alle Zeilen beginnen mit `//`).

Lesehinweis:
- Jede Zeile ist Kommentar, d.h. es wird nichts kompiliert.
- Wenn ihr es aktiviert, konfiguriert es einen `AutoBuilder` fuer Swerve.

Interaktiv:
- Finde die Stelle, wo `AutoBuilder.configure` die Callbacks erwartet.
- Ueberlege, welche Methoden aus `SwerveDrive` dort verwendet werden.

---

## Uebung: Erklaere den Datenfluss (interaktiv)

Aufgabe A: Erstelle eine 5-Schritt-Kette
1) Joystick gelesen in `DriveCommand` -> ?
2) `setChassisSpeeds` ruft -> ?
3) `SwerveDriveKinematics` rechnet -> ?
4) `SwerveModule.setDesiredState` setzt -> ?
5) Encoder + Gyro -> ?

Aufgabe B: Partner-Erklaerung
- Einer erklaert `SwerveModule.setDesiredState`, der andere erklaert `CTREModuleState.optimize`.
- Danach tauschen.

Aufgabe C: Suche im Code
- Finde, wo `absEncoderOffset` gesetzt wird.
- Finde, wo `moduleOffset` in die Kinematik eingeht.
- Finde, wo Field-Oriented vs Robot-Oriented entschieden wird.

---

## Hinweise fuer Rookies
- Wenn eine Methode nur Kommentare enthaelt, hat sie aktuell keinen Effekt.
- Achtet auf auskommentierte Blocks: das ist oft "planned" oder "experimentell".
- Wenn eine Variable niemals gesetzt wird (z.B. `moduleName`, `lastSpeeds`), erklaert das auch.

## Offene Fragen fuer das Team (Diskussion)
- Wollen wir den AccelerationLimiter aktivieren?
- Ist die Vision-Filterung (maxRotationSpeed, farDist) richtig fuer unser Setup?
- Soll `setIdleMode` auch den Angle Motor setzen?

Ende.
