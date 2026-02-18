# Single Scenario Simulator

This tool runs one explicit shooter scenario from robot pose + velocity and visualizes the
full trajectory in a separate interactive white window.

## What It Does
- Blue point + circle = hub center and acceptance radius.
- Click 1 = robot position.
- Click 2 = velocity vector endpoint.
- A trajectory is simulated with the same Java shot engine used by the robot code.
- Trajectory color represents ball height (`z`), with a colorbar on the right.

## Files
- Java runner: `src/main/java/frc/robot/sim/SingleScenarioRunner.java`
- Gradle task: `singleScenarioRun` in `build.gradle`
- Interactive UI: `scripts/interactive_single_scenario.py`
- Output JSON: `build/shot_plots/single_scenario_output.json`

## Run From CLI
```powershell
cd C:\Users\janis.j\Documents\2026Code\2026_MergedShooterTurret
.\gradlew.bat singleScenarioRun -PsingleScenarioArgs="2.0,3.0,0,1.0,0.5"
```

Args format:
- `x_m,y_m,heading_deg,vx_mps,vy_mps[,output_path]`

Example with custom output:
```powershell
.\gradlew.bat singleScenarioRun -PsingleScenarioArgs="2.0,3.0,15,1.2,-0.3,build/shot_plots/case_a.json"
```

## Run Interactive Window
```powershell
cd C:\Users\janis.j\Documents\2026Code\2026_MergedShooterTurret
py -3 scripts\interactive_single_scenario.py
```

Controls:
- Mouse click 1: robot start point
- Mouse click 2: velocity vector endpoint
- `r`: reset points
- `+`: increase speed scale
- `-`: decrease speed scale
- `q`: quit

Notes:
- Heading is aligned with the click-vector direction.
- Velocity magnitude is `vector_length * speed_scale`.
- The script invokes `gradlew singleScenarioRun` internally and then renders the produced JSON.

## Output Fields (JSON)
- `input`: scenario input values
- `hub`: hub center and radius
- `command`: solved yaw, RPMs, solve status, saturation flags, muzzle speeds
- `result`: hit/miss + error metrics
- `trajectory`: sampled points `{t_s,x_m,y_m,z_m}`

