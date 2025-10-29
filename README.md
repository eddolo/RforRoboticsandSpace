# 🧭 R–SO3 Resetability Framework

**Author:** Paolo Cappuccini — Independent Researcher  
**Project:** Resetability on SO(3) for Robust Control, Balance, and Attitude Recovery

---

### 🧠 Overview
The **R–SO3 Resetability Framework** introduces a new control primitive for systems governed by 3D rotations (SO(3)), such as **robots**, **rockets**, and **spacecraft**.

It defines a scalar metric **R** that measures how easily a recent sequence of rotations can be “reset” — i.e., undone — by replaying the same motion twice, scaled by a computed gain λ.  
Low-R motions can be efficiently reversed without retracing their full trajectory, enabling fast and stable recovery after disturbances.

This repository implements:
- 📚 **Core SO(3) Reset Library** (C++ / Python) — computes λ and R, applies scaled-twice resets  
- 🤖 **Robot Balance Reset Controller (GBRC)** — stabilizes posture after slips or pushes  
- 🛰 **Spacecraft Attitude Reset Controller (ARC)** — restores attitude post-fault using wheels or thrusters  
- 🚀 **Booster Monte Carlo Evaluator** — quantifies recovery time, residual error, and energy vs. baseline control  

Together, these demonstrate that Resetability can serve as a **universal fault-recovery primitive** for nonlinear rotational dynamics — bridging robotics, aerospace, and control theory.

---

### 📂 Contents
| Folder | Description |
|---------|-------------|
| `so3_reset/` | Core library (C++ + Python) |
| `demos/` | Simulation scripts (robot, spacecraft, booster) |
| `results/` | Generated data, plots, and reports |
| `videos/` | Recorded simulation videos |
| `logs/` | Run logs from the PowerShell launcher |

---

### ▶️ Run Everything
Launch all simulations automatically:
```powershell
.\run_all_windows.ps1


## 🧠 Running All Simulations on Windows

To execute the entire R–SO3 Resetability experiment suite on Windows — including the **robot balance**, **spacecraft attitude**, and **booster Monte Carlo** simulations — use the provided PowerShell launcher:

```powershell
.\run_all_windows.ps1
📦 What It Does
Launches all three simulations sequentially:

🤖 Robot Balance Demo (PyBullet visual + video recording)

🛰 Spacecraft Attitude Demo (ODE simulation + 3D or plot mode)

🚀 Booster Monte Carlo (fast + full statistical runs)

Displays colored status messages and progress bars during runs.

Automatically opens the latest MP4 and PDF results when finished.

Saves all terminal output and timestamps to:

bash
Copy code
logs/run_all_log_YYYYMMDD_HHMMSS.txt
⚙️ Requirements
Windows 10 or 11 with PowerShell ≥ 5

Python 3.9+ with the following packages:

bash
Copy code
pip install numpy matplotlib pybullet scipy
Optional: reportlab, pypandoc for high-quality PDF reports.

🧩 Notes
If PowerShell blocks the script, allow local execution temporarily:

powershell
Copy code
Set-ExecutionPolicy -ExecutionPolicy Bypass -Scope Process
This launcher fully automates the workflow for research, testing, and demonstration of the R–SO3 Resetability concept.

---

Cappuccini, P. (2025). Resetability on SO(3): Scaled Reversible Motion for Robotics and Spacecraft Attitude Recovery.
Independent Research Manuscript.