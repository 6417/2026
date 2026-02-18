# Simulator-Branch Uebersicht (`simulator`)

## Ziel
Dieser Branch verbindet Runtime-Logik (Shooter/Turret), Ballistik-Simulation und Offline-Tuning so,
dass aus Pose + Robotergeschwindigkeit ein robuster Shot-Command entsteht.

Wichtig: Wir verwenden jetzt einen **analytischen Solver** für den bewegten Schuss. Das alte
"global scale clamp" ist nicht mehr die Hauptlogik, sondern nur noch Hardware-Limit (RPM-Saettigung).

---

## Erklaerung
Die Grundidee im aktuellen Stand ist: Der Roboter schiesst nicht mehr nur aus einer statischen Situation,
sondern waehrend er fährt. Dadurch reicht es nicht, einfach direkt auf den Hub zu zeigen. Der Ball ist eine
Zeit lang in der Luft, waehrend sich der Roboter und der gesamte Bezugspunkt weiterbewegen. Genau deshalb wird
zuerst aus Pose, Heading und Translationsgeschwindigkeit berechnet, welche Muendungsgeschwindigkeit im Feld
noetig ist, damit der Ball nach seiner Flugzeit am Hub ankommt. Erst danach wird diese Feldgroesse in den
Turret-Winkel auf dem Roboter und in die beiden Shooter-RPMs umgesetzt.

Der wichtige Unterschied zu frueher ist, dass wir nicht mehr primaer über eine grobe Scale-Heuristik arbeiten,
die dann stark begrenzt wird, sondern über einen analytischen Löser. Der Solver liefert direkt eine
physikalisch konsistente Loesung mit Status (z. B. gueltig/ungueltig), was die Diagnose deutlich klarer macht.
Wenn es trotzdem Grenzen gibt, kommen diese jetzt vor allem von realen Hardware-Limits wie `maxRpm` und nicht
mehr von einer zentralen mathematischen Krücke. Das führt in der Praxis zu stabileren Kurven und weniger
Sonderfällen beim Tuning.

In der Simulation ist das Ziel nicht nur "Hit oder Miss", sondern vor allem reproduzierbare Auswertung:
Wie verändert sich der Fehler bei Distanz, Fahrtrichtung und Geschwindigkeit? Wo tritt RPM-Sättigung auf?
Wo liefert der Solver keine gueltige Loesung? Dadurch können wir offline systematisch verbessern und später
am echten Roboter meist nur noch wenige Konstanten feinjustieren, statt die Gesamtlogik erneut umzubauen.

---

## Wie der Solver konkret arbeitet
Der Solver bekommt als Eingabe den Shooter-Startpunkt im Feld (inklusive Höhe), den Hub-Zielpunkt
(inklusive Höhe), die aktuelle Robotergeschwindigkeit in X/Y, den festen Launch-Pitch des Shooters
und die physikalischen Grenzen für die Flugzeit. Dann sucht er nicht direkt nach RPM, sondern zuerst
nach einer **gültigen Flugzeit**. Diese Flugzeit ist der Schlüssel, weil damit die nötige
horizontale Ballgeschwindigkeit im Feldkoordinatensystem klar wird: horizontale Strecke geteilt durch Zeit.

Von dieser nötigen Ballgeschwindigkeit im Feld zieht der Solver die aktuelle Robotergeschwindigkeit ab.
Damit bleibt die Geschwindigkeit übrig, die der Shooter relativ zum fahrenden Roboter erzeugen muss.
Aus der Richtung dieses Vektors folgt der Turret-Yaw; aus dem Betrag (plus Pitch-Geometrie) folgt die
benötigte Mündungsgeschwindigkeit. Erst danach wird diese Geschwindigkeit in einen mittleren RPM-Wert
für die Wheels umgerechnet.

Die Suche nach der Flugzeit passiert numerisch stabil: zuerst wird in einem Zeitfenster ein Bereich
gesucht, in dem das Vorzeichen des vertikalen Fehlers wechselt (Bracket). Wenn so ein Bereich gefunden
wird, nutzt der Solver Bisection, bis Toleranzen erreicht sind. Falls kein Vorzeichenwechsel existiert,
wird der beste Naeherungswert geprueft; wenn auch der zu schlecht ist, gibt der Solver sauber
`NO_VALID_FLIGHT_TIME` zurück. Genau das macht das Verhalten debugbar: ihr seht, ob der Schuss
mathematisch loesbar war oder nicht, statt nur ein abgeschnittenes Ergebnis zu bekommen.

Optional gibt es noch eine leichte Drag-Kompensation als Faktor auf die benötigte Geschwindigkeit.
Sie ersetzt keine volle Aerodynamik, hilft aber systematische Unterreichweite auszugleichen, ohne die
Kernkinematik zu verbiegen. Am Ende werden wie gewohnt Hardware-Limits auf RPM angewendet und als
Sättigung markiert, damit klar ist: war das Problem Mathematik oder Motorgrenze.

---

## Änderungen gegenueber letztem Commit (Analyse)

### Neu
- `src/main/java/frc/robot/utils/ShotKinematicSolver.java`
  - analytische Berechnung von `yaw`, `flightTime` und benötigter Mündungsgeschwindigkeit,
  - Statusausgabe (`SOLVED`, `NO_VALID_FLIGHT_TIME`, ...), damit Fehlerursachen sichtbar sind.

- `src/main/java/frc/robot/sim/ShotFocusAutoTuneRunner.java`
  - deterministischer Auto-Tuner mit Fokus auf 2-5 m,
  - staged Suche (coarse -> fine),
  - erzeugt Kandidaten-CSV und Zusammenfassung.

- `src/main/java/frc/robot/sim/ShotFocusMetrics.java`
  - gemeinsame Kennzahlen fuer Trefferquote, Mean Error, RPM-Saettigung und Invalid-Rate,
  - Focus-/Long-Range-Bewertung und Gate-Checks.

### Geaendert
- `src/main/java/frc/robot/subsystems/ShooterSubsystem.java`
  - `computeMovingShotCommand(...)` nutzt jetzt den Solver statt reiner Scale-Heuristik,
  - `ShotCommand` traegt zusaetzlich `solveStatus`, `requiredMuzzleSpeedMps`, `rpmSaturated`,
  - Fire-Gates koennen auf `SHOT_SOLUTION_INVALID` und `RPM_SATURATED` blocken,
  - neue Dashboard-Telemetrie fuer Diagnose.

- `src/main/java/frc/robot/sim/ShotScenarioEvaluator.java`
  - parametrisierte `EvaluationConfig` (aus Constants + Override-Moeglichkeiten),
  - gleiche Kernlogik wie Runtime fuer konsistente Offline-Auswertung.

- `src/main/java/frc/robot/sim/MovingShotAutoTuneRunner.java`
  - Zielfunktion staerker auf robuste Fokus-Performance ausgerichtet,
  - bessere Behandlung von Invalid-/NaN-Faellen.

- `src/main/java/frc/robot/sim/ShotPlotDataExporter.java`
  - zusaetzlicher Export `robustness_focus_summary.csv`.

- `scripts/plot_shot_data.py`
  - liest und zeigt Focus-Summary mit an.

- `build.gradle`
  - neue Tasks:
    - `shotFocusAutoTune`
    - `shotFocusReport`

- `src/main/java/frc/robot/Constants.java`
  - neue/angepasste Tuning-Werte fuer Solver-Drag-Gain und Distance-Bias.

- Bereinigung Legacy-Code:
  - `src/main/java/frc/robot/sim/ShotScenarioSweepRunner.java` entfernt.
  - `src/main/java/frc/robot/sim/ShotAutoTuneRunner.java` entfernt.

---

## Aktueller Stand (nach den letzten Runs)
Quelle: `build/shot_plots/robustness_focus_summary.csv`

- `focus_2_5m`:
  - `hit_rate_pct = 99.049831`
  - `mean_closest_error_m = 0.427524`
  - `rpm_saturation_rate_pct = 1.330236`
  - `solve_invalid_rate_pct = 0.950169`

- `all_2_6m`:
  - `hit_rate_pct = 99.239865`
  - `mean_closest_error_m = 0.389152`
  - `rpm_saturation_rate_pct = 1.064189`
  - `solve_invalid_rate_pct = 0.760135`

Interpretation:
- Trefferquote ist hoch und stabil.
- Saettigung ist niedrig (~1%), also kein dominantes Clamp-Problem mehr.
- Rest-Invalids sind klein, aber noch ein sinnvoller Optimierungspunkt.

---

## Architektur auf einen Blick
1. Planungsebene:
   - aus Pose + Velocity wird ein `ShotCommand` berechnet (`yaw`, `topRPM`, `bottomRPM`).
2. Mechanikebene:
   - Turret und Wheels folgen Sollwerten.
3. Ballistikebene:
   - Flugbahn-Simulation bewertet Hit/Miss und Fehler.

---

## Runtime-Ablauf: Von Roboterdaten zu ShotCommand
Gegeben:
- Roboterpose im Feld,
- Robotergeschwindigkeit im Feld,
- Hubposition.

Ablauf:
1. Shooterposition im Feld berechnen (Pose + Shooter-Offset).
2. Solver-Eingabe bauen (Shooter/Hub als 3D-Punkte, Pitch, g, Velocity, Solver-Grenzen).
3. Solver loesen:
   - finde gueltige Flugzeit,
   - berechne noetigen Muendungsvektor,
   - gib `yawFieldRad`, `requiredMuzzleSpeedMps`, `status` zurück.
4. In Roboterkoordinaten umrechnen:
   - `turretYawRobotRad = yawField - robotHeading - turretZeroOffset`.
5. Aus Muendungsgeschwindigkeit -> `requiredAvgRpm`.
6. Baseline-Split beibehalten:
   - aus Tabellen kommt `splitRpm = top - bottom`,
   - final: `top = avg + split/2`, `bottom = avg - split/2`.
7. Hardware-Limits anwenden (`maxRpm`), Saettigung markieren.
8. Fire-Gates entscheiden (Distance, Turn-Rate, Solver-Status, Saettigung, Ready-Checks).

---

## Wichtige Dateien

### Runtime
- `src/main/java/frc/robot/subsystems/ShooterSubsystem.java`
- `src/main/java/frc/robot/subsystems/TurretSubsystem.java`
- `src/main/java/frc/robot/Constants.java`
- `src/main/java/frc/robot/utils/ShotKinematicSolver.java`

### Simulation
- `src/main/java/frc/robot/sim/ShooterTurretSimulator.java`
- `src/main/java/frc/robot/sim/BallisticShotSimulator.java`
- `src/main/java/frc/robot/sim/ShooterField2dVisualizer.java`
- `src/main/java/frc/robot/sim/ShotScenarioEvaluator.java`

### Analyse / Tuning
- `src/main/java/frc/robot/sim/MovingShotAutoTuneRunner.java`
- `src/main/java/frc/robot/sim/ShotFocusAutoTuneRunner.java`
- `src/main/java/frc/robot/sim/ShotFocusMetrics.java`
- `src/main/java/frc/robot/sim/ShotPlotDataExporter.java`
- `scripts/plot_shot_data.py`

---

## Workflow

### Daten exportieren
```powershell
cd C:\Users\janis.j\Documents\2026Code\2026_MergedShooterTurret
./gradlew shotPlotDataExport
```

### Plots bauen
```powershell
py -3 scripts\plot_shot_data.py
```

### Fokus-Report (ohne Tuning)
```powershell
./gradlew shotFocusReport
```

### Fokus-Auto-Tuning
```powershell
./gradlew shotFocusAutoTune -PfocusTuneArgs="20260225"
```

### Optional: bestehender Moving-Tuner
```powershell
./gradlew movingShotAutoTune -PmovingTuneArgs="6000,20260225"
```

---

## Hinweis zu Clamp/Sättigung
Früher war "clamp rate" ein zentrales Problem wegen globaler Scale-Grenzen.
Jetzt gilt:
- mathematisch loest der Solver direkt die benoetigte Schusskinematik,
- begrenzt wird primaer nur noch durch reale Motorgrenzen (`maxRpm`),
- die relevante Kennzahl ist daher `rpm_saturation_rate_pct`.

Bei den aktuellen Werten ist diese Rate niedrig. Damit ist Clamp aktuell kein Hauptblocker.

---

## Reality Calibrator
Es gibt jetzt einen controller-gesteuerten Feld-Kalibrierer für Shooter/Turret:
- `src/main/java/frc/robot/calibration/ShooterRealityCalibrator.java`
- Session- und Sample-Modelle in `src/main/java/frc/robot/calibration/`
- Persistenz als JSON via `src/main/java/frc/robot/calibration/CalibrationIO.java`

Erklärung:
Der Reality Calibrator ist der praktische Übergang zwischen Simulation und echter Hardware.
Während die Simulation euch stabile Vergleichswerte gibt, erfasst der Calibrator systematisch
reale Schüsse und lernt daraus Korrekturen für Turret-Offset und distanzabhängigen Bias.

Wichtig ist die Trennung in zwei Schritte:
1. Beim Abfeuern wird ein technisches Sample erfasst (Pose, Velocity, Distanz, berechnete RPM/Yaw, Solver-Status).
2. Danach wird das Ergebnis vom Operator gelabelt (Hit, Miss links/rechts, Miss short/long).

Genau diese gelabelten Daten nutzt `CalibrationFitter`, um eine neue Suggestion zu berechnen.
Seitliche Fehler korrigieren den Turret-Nulloffset, Längsfehler korrigieren die Bias-Kurve.
Die Anpassung ist dabei begrenzt und lokal gewichtet, damit einzelne Ausreißer nicht das ganze
Modell verziehen.

Die berechnete Suggestion wird nicht automatisch aktiv. Erst mit Commit wird sie in den Shooter
übernommen und als Runtime-JSON gespeichert. So bleibt der Ablauf kontrollierbar:
messen -> labeln -> fitten -> prüfen -> übernehmen.

Kurzablauf:
1. Calibration Mode aktivieren (Operator: `Back + Start`).
2. Mit `A` Schüsse erfassen.
3. Mit `X` = Hit, `B` = Miss markieren.
4. Optional Miss-Richtung:
   - D-Pad links/rechts für seitlichen Fehler,
   - D-Pad hoch/runter für long/short.
5. Mit `RB` Fit berechnen, mit `LB` übernehmen/speichern.

Persistenz:
- Laufzeitdatei: `calibration/shooter_calibration.json`
- Wird beim Start geladen (falls vorhanden) und als Override auf den Shooter angewendet.
