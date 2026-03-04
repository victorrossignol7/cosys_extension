# COSYS-AirSim Heterogeneous Extension (UAV + UGV)

Multi-domain extension of **Cosys-AirSim** enabling the simultaneous simulation of a **UAV (multirotor)** and a **UGV (skid-steer / Husky-like)** in the same Unreal Engine scene, with **separate RPC APIs** and Python test/orchestration scripts.

This repository contains the Python-side validation scripts and demo tooling used to validate the heterogeneous extension described in the final project report (UAV + UGV in one Unreal scene, dual-port API design, heterogeneous orchestration workflow). The report documents the dedicated `Heterogeneous` SimMode, a combined API server wrapper exposing multirotor and car APIs on two ports, and the validation methodology based on `test_drone.py`, `test_ugv.py`, and an orchestrator.

---

## Project goals

The implemented prototype validates the following key idea:

- **Spawn and control heterogeneous robots in the same Unreal Engine environment**
  - 1 UAV (e.g. SimpleFlight)
  - 1 UGV (e.g. CPHusky / skid-steer)
- **Keep compatibility with existing AirSim/Cosys-AirSim clients**
  - `MultirotorClient` for UAV
  - `CarClient` for UGV
- **Use a practical dual-port RPC architecture**
  - UAV API on `41451`
  - UGV API on `41452`
- **Validate with simple Python scripts + orchestrator**

This matches the final report scope and validation strategy.

---

## Repository structure (Python validation / demo side)

```text
AirSim/
├── settings.json                     # Main heterogeneous demo configuration (UAV + UGV)
├── test_drone.py                     # UAV connectivity + control validation
├── test_ugv.py                       # UGV connectivity + control validation
├── materials.csv                     # Project-specific data file (if used by your scripts)
│
├── step1/
│   ├── 1vehicle_test.py              # Early milestone: single vehicle tests
│   └── setting_milestoneA.json
│
├── step2/
│   └── 2same_vehicle_test.py         # Early milestone: homogeneous multi-vehicle tests
│
├── step3/
│   ├── client_uav1.py
│   ├── client_uav2.py
│   ├── orchestrator_server.py        # Orchestrator for homogeneous / earlier integration stage
│   └── settings_milestone_2_and_3.json
│
└── step4/
    ├── client_orch_demo.py
    ├── orchestrator_server_hetero.py # Heterogeneous orchestrator (UAV + UGV)
    └── test/
        ├── diag_ugv.py
        ├── test_hetero_drone_watch_ugv.py
        ├── test_uav_direct.py
        └── test_ugv_direct.py
```

---

## Prerequisites

### 1) Unreal Engine + Cosys-AirSim heterogeneous extension (C++ side)
You need a **Cosys-AirSim build that includes the heterogeneous extension** (the new `Heterogeneous` simulation mode, UGV skid-steer integration, and dual API server behavior). These are the C++ changes described in the final report.

### 2) Python environment (AirSim client scripts)
Use your dedicated virtual environment:

```powershell
C:\Users\victo\venvs\airsim311\Scripts\Activate.ps1
```

Recommended packages (depending on your scripts):

```powershell
pip install msgpack-rpc-python
pip install numpy
```

If your scripts use the AirSim Python API package from source, ensure it is available in the active environment (`airsim` import works).

---

## Launch Unreal Engine with the project settings (Windows / PowerShell)

Use the following PowerShell commands (provided for this project setup):

```powershell
$uproject = "C:\Users\victo\source\repos\Cosys-AirSim\Unreal\Environments\Blocks\Blocks.uproject"
$settings = "C:\Users\victo\Documents\AirSim\settings.json"
$editor   = "C:\Program Files\Epic Games\UE_5.5\Engine\Binaries\Win64\UnrealEditor.exe"

& $editor $uproject -settings="$settings"
```

### Expected behavior
- Unreal opens the **Blocks** environment.
- Cosys-AirSim reads `settings.json`.
- The scene starts in **`SimMode = Heterogeneous`** (if configured in `settings.json`).
- Both robots should be spawned and reachable on their respective APIs, consistent with the report validation criteria (instantiation, separate control, coexistence, readable state feedback).

---

## RPC ports and clients

The heterogeneous mode uses a practical dual-port convention:

- **UAV / multirotor API**: `41451`
- **UGV / car API**: `41452` (commonly `api_port + 1`)

This design is explicitly described in the report and allows direct reuse of existing AirSim Python clients without protocol changes.

### Python clients used
- `airsim.MultirotorClient(host, 41451)` for `Drone1`
- `airsim.CarClient(host, 41452)` for `UGV1`

---

## Quick start (demo workflow)

### 1) Open PowerShell and activate the venv
```powershell
C:\Users\victo\venvs\airsim311\Scripts\Activate.ps1
```

### 2) Go to the AirSim script folder
```powershell
cd C:\Users\victo\Documents\AirSim
```

### 3) Launch Unreal Engine (separate PowerShell)
Use the launch block shown above and wait until the scene is loaded.

### 4) Run the validation scripts

#### UAV validation
```powershell
python .\test_drone.py
```

#### UGV validation
```powershell
python .\test_ugv.py
```

#### Heterogeneous orchestrator demo (project scripts)
Depending on the scenario:
```powershell
python .\step4\orchestrator_server_hetero.py
```

You can also use the step4 test scripts for direct checks:
```powershell
python .\step4\test\test_uav_direct.py
python .\step4\test\test_ugv_direct.py
python .\step4\test\test_hetero_drone_watch_ugv.py
```

The report describes this validation logic as an incremental workflow: validate UAV path, validate UGV path, then run joint orchestration.

---

## Demo terminal UI (colored) + CSV sensor logging

For a visual demo, a terminal dashboard script can be used to:
- launch `test_drone.py`
- monitor UAV + UGV connection status
- display active sensors/cameras (based on `settings.json`)
- show live positions
- log telemetry/sensor summaries to CSV per robot

### Run the demo dashboard (colored + CSV)
```powershell
python .\demo_cosys_dual_terminal_color_csv.py
```

Or with explicit paths:
```powershell
python .\demo_cosys_dual_terminal_color_csv.py .\test_drone.py .\settings.json
```

### Output logs
CSV files are written in:
```text
.\demo_logs\
```

Typical outputs:
- `Drone1_telemetry_*.csv`
- `UGV1_telemetry_*.csv`

### CSV format (high level)
Each row is timestamped and stores robot telemetry and sensor-related values in a flat format suitable for spreadsheet analysis, for example:
- `timestamp`
- pose (`x`, `y`, `z`)
- orientation / kinematics (depending on API availability)
- sensor values or sensor summaries
- units in column headers (e.g. `[m]`, `[m/s]`, `[deg]`)

> Note: For heavy sensors such as LiDAR, the demo logger may record a compact summary (e.g. point count) rather than the full raw point cloud to keep the CSV size manageable.

---

## `settings.json` notes (heterogeneous scene)

The report highlights that the heterogeneous workflow remains **configuration-driven** via `settings.json`, including:
- `SimMode = Heterogeneous`
- one UAV (e.g. `Drone1`, `SimpleFlight`)
- one UGV (e.g. `UGV1`, `CPHusky`)
- per-vehicle sensors (camera, IMU, GPS, etc.)
- base API port (`41451`) implying UGV on `41452` through the wrapper server

This is a key usability point because scenario changes do not require recompiling the whole project.

---

## Typical validation checks (recommended before demo)

### UAV checks (`test_drone.py`)
- connection to multirotor endpoint (`41451`)
- API control enablement
- arm / takeoff / hover / basic move / land (depending on script implementation)

### UGV checks (`test_ugv.py`)
- connection to car endpoint (`41452`)
- API control enablement for `UGV1`
- send `CarControls` (throttle / steering / brake)
- read back state (`speed`, `gear`, `rpm`) and pose

### Joint checks (heterogeneous)
- both clients connect simultaneously
- commands to one robot do not block the other
- pose/state feedback remains readable for both

These are aligned with the functional validation criteria listed in the final report (instantiation, separate control, coexistence, state feedback, repeatability).

---

## Troubleshooting

### `ModuleNotFoundError: No module named '_curses'` on Windows
Windows Python does not include `curses` by default.

Solutions:
- use a Windows-friendly non-curses dashboard script (recommended), or
- install compatibility package:
```powershell
pip install windows-curses
```

### `confirmConnection()` fails / robots not found
Check:
- Unreal scene is fully loaded
- `settings.json` path passed correctly to Unreal
- `SimMode` is set to `Heterogeneous`
- ports are not blocked (`41451`, `41452`)
- robot names in scripts match `settings.json` (`Drone1`, `UGV1`)

### UGV moves in Unreal but no state feedback in script
Check that the script is connecting to the **UGV/car API port (`41452`)**, not the UAV port.

### CSV logs are not created
Check:
- write permissions in `C:\Users\victo\Documents\AirSim`
- script actually started (watch terminal for status/log panel)
- `demo_logs` directory existence (the script should create it)

---

## Project context (academic)

- **Course**: Virtual Reality for Robotics (University of Genoa - DIBRIS, a.y. 2025/2026)
- **Project**: Multi-domain extension of Cosys-AirSim for heterogeneous robot simulation (UAV + UGV)
- **Final deliverable**: functional prototype + validation scripts + final report

The report concludes that a heterogeneous UAV+UGV setup can be integrated into Cosys-AirSim without a full rewrite, by adapting key architectural points (SimMode, API server layer, physics ownership, SimApi selection) and validating with dedicated scripts/orchestrator.

---

## Future improvements (repository-side)

Suggested next steps for this Python-side repo:
- package the orchestrator scripts into a reusable CLI (`python -m ...`)
- add structured logging (JSONL) in addition to CSV
- add a demo config selector (`--scenario blocks / warehouse / custom`)
- add a unified operator dashboard for UAV + UGV command shortcuts
- add ROS 2 bridge scripts for experiment logging / replay

These directions are consistent with the future work themes identified in the report (coordination abstractions, additional robot classes, benchmarking, ROS 2 integration).

---

## License / attribution

This README documents the Python validation/demo layer associated with a Cosys-AirSim heterogeneous extension project. Please adapt the license and attribution section to the licensing terms of:
- your own scripts,
- Cosys-AirSim / AirSim upstream components,
- any course deliverable constraints.

