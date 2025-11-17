# Advanced DC Motor Control Laboratory

## Overview

This comprehensive Python application provides:
1. **Solutions to Problems 13 & 14** - Analytical motor calculations
2. **Advanced Multi-Physics Simulation** - Real-time electromagnetic-thermal-mechanical coupling
3. **Interactive GUI** - Professional Tkinter interface with dynamic controls
4. **Economic Analysis** - Power consumption and cost tracking
5. **Loss Analysis** - Detailed breakdown of all motor losses

## Features

### ✅ Problem Solvers
- **Problem 13**: 440-V shunt motor speed control with series resistance
- **Problem 14**: 460-V series motor with field flux reduction

### ✅ Multi-Physics Simulation
- **Electromagnetic**: Coupled armature and field circuits with back EMF
- **Thermal**: Temperature-dependent resistances, heat transfer equations
- **Mechanical**: Shaft stress analysis, bearing loads, torque transients

### ✅ Advanced Controls
- Real-time ODE solvers (RK45 and Euler methods)
- Adjustable parameters via sliders
- Start/Stop/Reset controls
- Motor type selection (Shunt/Series)

### ✅ Visualization
- Dynamic real-time graphs
- 5 tabs with specialized analysis:
  1. **Motor Control**: Speed, current, torque, power, efficiency, temperature
  2. **Multi-Physics**: Thermal dynamics, mechanical stress, electrical coupling
  3. **Loss Analysis**: Detailed breakdown with pie charts and history
  4. **Economic Analysis**: Power consumption, cost accumulation
  5. **Problem Solvers**: Analytical solutions with verification

### ✅ Loss Breakdown
- Copper losses (armature and field) - temperature dependent
- Iron core losses - speed dependent
- Mechanical friction losses
- Stray load losses
- Real-time pie chart and history graphs

### ✅ Economic Features
- Runtime tracking
- Energy consumption (kWh)
- Operating cost calculation
- Average and peak power monitoring
- Efficiency tracking

## Installation

```bash
# Install required packages
pip install -r requirements_motor_lab.txt

# Or install individually:
pip install numpy matplotlib scipy
```

## Usage

### Running the Application

```bash
python3 dc_motor_advanced_lab.py
```

### Problem Solutions

When you run the application, it first displays the analytical solutions in the console:

**Problem 13 Output:**
```
Supply voltage: 440 V
Initial speed: 1500 rpm
Required series resistance: 2.97 Ω
New armature current: 22.5 A
```

**Problem 14 Output:**
```
Supply voltage: 460 V
Initial current: 25 A
New current: ~26 A
New speed: ~1024 rpm
```

### GUI Operations

1. **Select Motor Type**: Choose between Shunt or Series motor
2. **Choose ODE Solver**: RK45 (accurate) or Euler (fast)
3. **Adjust Parameters**: Use sliders to control:
   - Supply voltage (0-600 V)
   - External resistance (0-10 Ω)
   - Load torque (0-100 N·m)
   - Armature resistance (0.1-5 Ω)
   - Field resistance (50-500 Ω)

4. **Control Simulation**:
   - **START**: Begin real-time simulation
   - **STOP**: Pause simulation
   - **RESET**: Clear all data and restart

5. **Navigate Tabs**:
   - **Motor Control**: Main operating parameters
   - **Multi-Physics**: Advanced coupling analysis
   - **Loss Analysis**: Detailed loss breakdown
   - **Economic Analysis**: Cost and energy tracking
   - **Problem Solvers**: Analytical solutions

## Technical Details

### Differential Equations

The simulator solves coupled ODEs:

```python
# State vector: [Ia, If, omega, theta, T_motor]

# Armature circuit
dIa/dt = (V - Eb - Ia*Ra(T)) / La

# Field circuit
dIf/dt = (V - If*Rf(T)) / Lf

# Mechanical equation
dω/dt = (Te - TL - Bω) / J

# Thermal equation
dT/dt = (P_loss - P_dissipated) / C_thermal
```

### Multi-Physics Coupling

1. **Electromagnetic → Thermal**: I²R losses generate heat
2. **Thermal → Electromagnetic**: Temperature increases resistance (0.393%/°C)
3. **Electromagnetic → Mechanical**: Torque drives rotation
4. **Mechanical → Electromagnetic**: Speed generates back EMF

### Loss Calculation

- **Copper Losses**: I²R with temperature correction
- **Iron Losses**: Proportional to speed²
- **Mechanical Losses**: Proportional to speed
- **Stray Losses**: Proportional to output power

### Mechanical Stress

Shaft shear stress calculation:
```
τ = T·r / J_polar
J_polar = π·d⁴ / 32
Safety Factor = σ_yield / τ
```

## Problem Solutions Explained

### Problem 13: Shunt Motor Speed Control

**Given:**
- 440 V supply, 1500 rpm, 30 A, 15 hp output
- Load torque ∝ speed²

**Method:**
1. Calculate armature resistance from power equation
2. Find back EMF at both speeds (Eb ∝ N)
3. Calculate new current (Ia ∝ N² for T ∝ N²)
4. Determine required series resistance

**Answer:** R = 2.97 Ω, Ia = 22.5 A

### Problem 14: Series Motor Field Flux

**Given:**
- 460 V, Ra = 0.4 Ω, 25 A, 1000 rpm
- 5% flux reduction, T ∝ N², same efficiency

**Method:**
1. Set up torque balance: T₂/T₁ = (N₂/N₁)²
2. Account for flux weakening in torque equation
3. Use voltage equation with back EMF
4. Solve numerically for new current and speed

## Auto-Scaling

The application automatically adjusts to window resizing. All graphs and controls scale appropriately with the window size.

## Performance Notes

- **RK45 Solver**: More accurate, recommended for detailed analysis
- **Euler Solver**: Faster, suitable for quick demonstrations
- **History Limit**: Keeps last 1000 data points for smooth performance
- **Update Rate**: 50ms (20 FPS) for real-time visualization

## Educational Value

This lab demonstrates:
- ✅ DC motor theory and equations
- ✅ Numerical ODE solving techniques
- ✅ Multi-physics system modeling
- ✅ Real-time simulation and visualization
- ✅ Thermal management in electrical machines
- ✅ Economic analysis of motor operation
- ✅ Professional GUI development

## Troubleshooting

**Issue**: GUI doesn't start
- **Solution**: Ensure matplotlib and tkinter are installed

**Issue**: Simulation runs slowly
- **Solution**: Switch to Euler solver or reduce update rate

**Issue**: Plots not updating
- **Solution**: Click RESET and START again

## References

- DC Motor Theory: Chapman, "Electric Machinery Fundamentals"
- Numerical Methods: Scipy documentation
- Multi-Physics: "Multiphysics Modeling Using COMSOL"

## Author

Created for advanced electrical engineering laboratory work.
Combines theoretical analysis with practical simulation.

## License

Educational use permitted. Attribution appreciated.
