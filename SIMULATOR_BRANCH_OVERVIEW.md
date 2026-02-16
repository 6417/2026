# Simulator-Branch Übersicht (`simulator`)

## Komponenten:
- Turret- und Shooter-Runtime-Logik,
- Physik-Simulation fuer den Ballflug,
- Offline-Analyse und Auto-Tuning,

sodass wir aus der aktuellen Roboterlage (Pose + Geschwindigkeit) einen Schuss berechnen können, der den Hub trifft, und das Verhalten vor echten Feldtests systematisch auswerten koennen.

---

## Erklärung
Problem: Der Roboter bewegt sich, aber der Ball braucht Zeit bis zum Ziel. Also dürfen wir nicht nur auf den Hub zeigen, sondern müssen vorausberechnen, welche mündungsrichtung und welche Ballgeschwindigkeit jetzt nötig sind, damit der Ball später dort ankommt, wo der Hub ist.

Dafür nimmt der Code zuerst die aktuelle Roboterposition, die Hubposition und die aktuelle Robotergeschwindigkeit. Aus Distanz und Flight-Time-Tabelle wird abgeschaetzt, wie schnell der Ball im Feldkoordinatensystem sein muss. Danach wird die aktuelle Robotergeschwindigkeit herausgerechnet. Das Ergebnis ist die Mündungs- bzw. Abschussgeschwindigkeit relativ zum Shooter. Aus diesem Vektor entsteht direkt der Turret-Winkel. Aus dem Betrag dieses Vektors entsteht der nötige Speed-Scale fuer die Shooter-RPM.

In der Praxis wird dieser Scale begrenzt (clamped), damit das System in sicheren Grenzen bleibt. Genau hier liegt oft der Tradeoff: Wenn die Grenzen zu eng sind, sind viele Fälle am Anschlag und die Kurven werden unruhig oder zu flach. Wenn die Grenzen zu weit sind, kann das Modell zwar flexibler werden, aber die Trefferquote kann leiden. Deshalb haben wir den Simulations- und Tuning-Teil so gebaut, dass wir nicht nur einzelne Schoene-Faelle sehen, sondern robuste Raster über Distanz, Richtung und Geschwindigkeit auswerten koennen.

Der Simulator  integriert den Ballflug schrittweise mit Gravitation und quadratischem Luftwiderstand. Dadurch sehen wir für jeden Schuss nicht nur Hit/Miss, sondern auch den nächsten Abstand zum Hub, die horizontale/vertikale Abweichung und die gesamte Flugspur. So können wir nachvollziehen, warum ein Schuss knapp daneben geht, statt nur zu sehen, dass er daneben ging.

Das Zielbild ist: Der Code im Runtime-Pfad und der Code im Simulationspfad benutzen dieselbe Kernmathematik (insbesondere beim Moving-Shot-Scale). Dann sind die Offline-Plots belastbar und wir koennen Konstanten gezielt anpassen, statt blind am Robot zu raten.

---

## Architektur auf einen Blick
Drei Ebenen arbeiten zusammen:
1. **Planungsebene**: aus Pose + Velocity wird ein ShotCommand berechnet (TurretYaw + TopRPM + BottomRPM).
2. **Mechanikebene**: Turret und Wheels folgen diesen Sollwerten.
3. **Ballistikebene**: simulierte Flugbahn bewertet, ob der Schuss den Hub trifft.

---

## Wichtige Dateien und Aufgaben

### Runtime / Steuerung

- `src/main/java/frc/robot/subsystems/ShooterSubsystem.java`
  - Kernlogik für Moving Shot.
  - Berechnet aus Roboterdaten den Schussbefehl:
    - Turret-Winkel,
    - Top/Bottom-RPM.
  - Regelt Wheels via PID + Feedforward.
  - Schreibt Sim-Telemetrie (Hit, Error, Trace-Infos).

- `src/main/java/frc/robot/subsystems/TurretSubsystem.java`
  - Turret-Sollwinkel und aktueller Winkel.
  - Derzeit Hybridzustand: Hardware-Konfig vorbereitet, Sim-Fallback aktiv.

- `src/main/java/frc/robot/Constants.java`
  - Alle entscheidenden Konstanten:
    - RPM-Tabellen,
    - Flight-Time-Tabelle,
    - Scale-Grenzen,
    - `rpmToMpsFactor`, Drag,
    - `distanceScaleBiasTable`.

### Simulation
- `src/main/java/frc/robot/sim/ShooterTurretSimulator.java`
  - Simuliert Wheel-Dynamik + Turret-Dynamik.
  - Erzeugt Schuss-Events fuer den Ballistik-Simulator.

- `src/main/java/frc/robot/sim/BallisticShotSimulator.java`
  - Integriert Ballflug (g + quadratischer Drag).
  - Liefert Hit/Miss und naechste Annaherung.

- `src/main/java/frc/robot/sim/ShooterField2dVisualizer.java`
  - Zeichnet Robot, Turret-Vektor, aktive Kugel und Flugspuren auf Field2d.

- Datenobjekte:
  - `ShotEvent.java`, `ShotSample.java`, `ShotResult.java`, `CompletedShotTrace.java`.

### Analyse / Tuning
- `src/main/java/frc/robot/sim/ShotScenarioEvaluator.java`
  - Bewertet einzelne Szenarien mit derselben Schussformel wie Runtime.

- `src/main/java/frc/robot/sim/ShotPlotDataExporter.java`
  - Exportiert:
    - `rpm_distance.csv`
    - `trajectories.csv`
    - `direction_sweep.csv`
    - `robustness_grid.csv`
    - `robustness_summary.csv`

- `src/main/java/frc/robot/sim/MovingShotAutoTuneRunner.java`
  - Clamp-bewusster Auto-Tuner (Haupttuner).

- `src/main/java/frc/robot/sim/ShotAutoTuneRunner.java`
  - Legacy-Tuner (Referenz).

- `scripts/plot_shot_data.py`
  - Erzeugt SVG-Plots aus den CSV-Dateien.

---

## Was passiert im Code bei bekannten Roboterdaten?
Gegeben:
- Roboterpose im Feld,
- Robotergeschwindigkeit im Feld,
- Hubposition.

Ablauf:
1. Shooterposition im Feld berechnen (Roboterpose + Shooter-Offset).
2. Vektor Shooter -> Hub und Distanz bestimmen.
3. Baseline-Werte aus Tabellen holen (TopRPM, BottomRPM, FlightTime).
4. Noetige Ball-Feldgeschwindigkeit berechnen (`toHub / flightTime`).
5. Robotergeschwindigkeit abziehen -> noetige Muen dungs-Geschwindigkeit.
6. Richtung dieses Vektors -> Turret-Winkel.
7. Betrag dieses Vektors -> noetiger Speed-Scale.
8. Scale mit `distanceScaleBias` korrigieren und in `[min,max]` clampen.
9. RPMs skalieren und als finalen ShotCommand ausgeben.

Ergebnis:
- Turret richtet sich auf den berechneten Vorhaltewinkel aus,
- Shooter-Wheels laufen auf die berechneten RPM-Sollwerte,
- Schuss kann mit Fire-Gates freigegeben werden.

---

## Clamp-Rate kurz erklaert
`clampRate` = Anteil der Szenarien, bei denen `rawScale` ausserhalb der erlaubten Grenzen liegt und abgeschnitten wird.

Hohe Clamp-Rate bedeutet:
- viele verschiedene Situationen landen auf demselben Grenzwert,
- weniger feine Anpassung,
- oft zackige oder platte Kurven in den Richtungsplots.

Niedrige Clamp-Rate bedeutet:
- mehr nutzbarer Regelbereich,
- feinere Anpassung ueber Richtung/Speed/Distanz.

---

## Workflow: Export, Plot, Tuning

### Daten exportieren
```powershell
cd C:\Users\janis.j\Documents\2026Code\2026_MergedShooterTurret
./gradlew shotPlotDataExport
```

### Plots erzeugen
```powershell
py -3 scripts\plot_shot_data.py
```

### Wichtige Plots
- `build/shot_plots/turret_angle_vs_direction.svg`
- `build/shot_plots/motor_rpm_vs_direction.svg`
- `build/shot_plots/closest_error_vs_direction.svg`
- `build/shot_plots/scale_vs_direction.svg`
- `build/shot_plots/robust_hit_rate_vs_speed.svg`
- `build/shot_plots/robust_mean_error_vs_speed.svg`
- `build/shot_plots/robust_clamp_rate_vs_speed.svg`

### Auto-Tuner starten
```powershell
./gradlew movingShotAutoTune -PmovingTuneArgs="6000,20260225"
```
- arg1: Iterationen
- arg2: Seed

---

## Praktische Hinweise
- Plot-Dateien liegen in `build/` und werden typischerweise nicht von Git getrackt.
- Fuer Team-Reviews wichtige Plots in einen versionierten Ordner kopieren.
- Es gibt aktuell bekannte Deprecation-Warnings zu `schedule()` (nicht Teil der Shot-Mathematik).
