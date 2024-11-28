# Navigation-and-Guidance-Project

This project investigates the stability and control characteristics of the F104-A aircraft model.
Through longitudinal dynamics analysis, stability derivatives are determined and expressed in state
space form. Transfer functions for longitudinal are derived to examine the influence of stability
derivatives on eigenvalues and eigen vectors. An optimal feedback control law is designed using
Linear Quadratic Regulator (LQR) techniques, ensuring closed-loop stability by tuning the gain
vector. A Kalman Filter is then implemented to estimate the true states, and its performance is
analyzed through simulated plots.
Furthermore, the project, focuses on a missile-target engagement scenario, where three distinct
guidance laws are designed and analyzed. Simulations of missile and target trajectories, along
with the corresponding guidance commands, are presented for each law. A comparative analysis
highlights the differences in performance and effectiveness of the guidance strategies. The study
integrates control and estimation techniques, offering insights into both aircraft dynamics and missile guidance systems.

The provided script is divided into two parts:

### **1. `guidancesolver.py`**
This script is focused on simulating missile guidance scenarios using different guidance techniques. 

#### Key Components:

1. **Parameters and Initialization:**
   - `Dist`, `Vm`, `Vt`, etc., define the initial conditions for the simulation, including missile and target velocities, angles, and distances.
   - `GuidanceType` is an enumeration that defines three guidance methods:
     - Pure Pursuit (PP)
     - Deviated Pursuit (DP)
     - True Proportional Navigation (TPN)

2. **`Solver` Class:**
   Handles the core simulation logic, implementing trajectory calculations for each guidance method.
   - **Initialization (`__init__`)**:
     Sets the starting positions of the missile and target.
   - **Reset Function (`reset`)**:
     Resets the simulation state for a specific guidance type.
   - **Guidance Methods**:
     - `PP`: Pure pursuit guidance.
     - `DP`: Deviated pursuit guidance, with an additional deviation angle (`Delta`).
     - `TPN`: True proportional navigation, which uses navigation constants and target-relative velocity vectors.
   - **Trajectory Calculation (`CalculateTrajectory`)**:
     Simulates the missile and target movements over discrete time steps.
   - **Debug Plot (`TpnDebugPlot`)**:
     A debugging function to visualize the TPN method in real-time.
   - **Result Plotting (`Plot`)**:
     Plots missile and target trajectories along with time-variance plots for guidance commands and range.

3. **Simulation Execution:**
   - The script calculates and optionally saves trajectory plots for all three guidance methods (`PP`, `DP`, `TPN`).

#### Output:
Plots showing missile and target trajectories and their dynamics over time. These can be saved or displayed interactively.

---

### **2. MATLAB Code**
This section appears to handle state-space modeling and Linear Quadratic Regulator (LQR) design for a different control problem.

#### Key Components:

1. **State-Space Model:**
   - `A`, `B`, `C`, `D` matrices define a continuous-time state-space system.

2. **Eigenvalues and LQR Design:**
   - The eigenvalues of `A` (`eig_A`) are calculated to analyze system stability.
   - An LQR controller is designed using:
     - `Qbar` (state-weighting matrix) and `R` (control-weighting scalar).
   - The eigenvalues of the closed-loop system (`A - B * K`) are analyzed.

3. **Discretization:**
   - The closed-loop system is discretized using a zero-order hold (`zoh`) method.
   - The discretization step size (`Ts`) is iteratively increased until all eigenvalues of the discrete system (`F`) lie inside the unit circle, ensuring stability.

4. **Noise Generation:**
   - Simulates process and measurement noise for real-world applicability.

---

### **Key Observations:**
- The Python (`guidancesolver.py`) and MATLAB code address different aspects of control systems. While the Python script focuses on missile guidance and trajectory plotting, the MATLAB script delves into state-space analysis and LQR design.
- Both pieces of code highlight practical control system applications, such as navigation and stabilization.


