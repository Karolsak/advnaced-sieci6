# Quick Start Guide - DC Motor Braking Simulation

## ⚡ 5-Minute Quick Start

### 1. Install Dependencies (if not already installed)
```bash
pip3 install numpy matplotlib scipy --user
```

### 2. Run the Application
```bash
cd /home/user/advnaced-sieci6
python3 dc_motor_braking_simulation.py
```

**Note:** If tkinter is not available, you can still use the standalone model:
```python
from dc_motor_model import DCMotorModel

motor = DCMotorModel(V=400, P=18650, N=45, eta=0.746, Ra=0.2)
result = motor.calculate_braking_resistance()
print(f"Braking Resistance: {result['Rb']:.3f} Ω")
```

### 3. View Example 30.47 Solution
- Launch the application
- The Main Menu tab shows the complete solution automatically
- Braking resistance: **6.103 Ω**

### 4. Run a Simulation
1. Click "Simulation" tab
2. Select solver: **RK45** (recommended)
3. Set simulation time: **5 seconds**
4. Adjust braking resistance slider to **6.103 Ω**
5. Click **"Start Simulation"**
6. Switch to "Results & Graphs" tab to see dynamic plots

### 5. Explore Features
- **Thermal Analysis** tab: View temperature rise and loss breakdown
- **Mechanical Stress** tab: Check shaft stress and bearing loads
- **Economic Analysis** tab: Calculate operating costs (click "Calculate")

---

## 📋 Example 30.47 - Quick Reference

### Given:
- Voltage: 400 V
- Power: 25 h.p. (18.65 kW)
- Speed: 45 rpm
- Efficiency: 74.6%
- Armature Resistance: 0.2 Ω
- Method: Plugging

### Answer:
| Parameter | Value |
|-----------|-------|
| Braking Resistance | **6.103 Ω** |
| Max Braking Current | **125.00 A** |
| Max Braking Torque | **8176.97 N·m** |
| Torque at Zero Speed | **4151.27 N·m** |

---

## 🧪 Run Tests
```bash
python3 test_dc_motor_simulation.py
```

Expected output: **All 7 tests passed successfully!**

---

## 📁 Key Files

| File | Purpose |
|------|---------|
| `dc_motor_braking_simulation.py` | Main GUI application |
| `dc_motor_model.py` | Standalone models (no GUI) |
| `test_dc_motor_simulation.py` | Test suite |
| `README_DC_MOTOR_SIMULATION.md` | Full documentation |
| `PROJECT_SUMMARY.md` | Project overview |

---

## 🎯 Common Use Cases

### Use Case 1: Calculate Braking Resistance
```python
from dc_motor_model import DCMotorModel

motor = DCMotorModel(V=400, P=18650, N=45, eta=0.746, Ra=0.2)
result = motor.calculate_braking_resistance(max_current_factor=2)

print(f"Required braking resistance: {result['Rb']:.3f} Ω")
print(f"Maximum braking torque: {result['T_brake_max']:.2f} N·m")
```

### Use Case 2: Simulate Motor Starting
```python
import numpy as np
from scipy.integrate import solve_ivp
from dc_motor_model import DCMotorModel

motor = DCMotorModel()
y0 = [0, 0, 0, 0]  # Start from rest
t_span = (0, 5.0)
t_eval = np.linspace(0, 5, 1000)

sol = solve_ivp(
    lambda t, y: motor.motor_dynamics_rk45(t, y, V_applied=400, T_load=0, Rb=0),
    t_span, y0, method='RK45', t_eval=t_eval
)

print(f"Final speed: {sol.y[1][-1] * 60 / (2*np.pi):.2f} rpm")
```

### Use Case 3: Calculate Losses
```python
from dc_motor_model import DCMotorModel

motor = DCMotorModel()
losses = motor.calculate_losses(motor.Ia_rated, motor.omega_rated)

print(f"Total losses: {losses['total']:.2f} W")
print(f"Copper losses: {losses['copper_total']:.2f} W")
print(f"Iron losses: {losses['iron']:.2f} W")
```

---

## 🔧 Troubleshooting

### Problem: "ModuleNotFoundError: No module named 'tkinter'"
**Solution:** Install tkinter or use standalone model
```bash
# Ubuntu/Debian
sudo apt-get install python3-tk

# Or use standalone model without GUI
python3 -c "from dc_motor_model import DCMotorModel; print('Standalone model works!')"
```

### Problem: GUI window too small
**Solution:** The application has auto-scaling. Resize the window and it will adjust automatically.

### Problem: Simulation takes too long
**Solution:**
- Use Euler solver instead of RK45 for faster results
- Reduce simulation time
- Reduce number of time points

### Problem: Want to export data
**Solution:**
- Run simulation first
- Go to "Results & Graphs" tab
- Click "Export Data" button
- Choose location to save CSV file

---

## 🎓 Learning Path

### Beginner
1. Run the application
2. View Example 30.47 solution
3. Run basic simulation with default parameters
4. View graphs

### Intermediate
1. Modify motor parameters
2. Try different ODE solvers
3. Adjust control sliders
4. Calculate economics

### Advanced
1. Use standalone model in custom scripts
2. Modify differential equations
3. Add new control algorithms
4. Integrate with other systems

---

## 📞 Support

### Documentation
- `README_DC_MOTOR_SIMULATION.md` - Complete user manual
- `PROJECT_SUMMARY.md` - Project overview
- Inline code comments - Detailed explanations

### Testing
- `test_dc_motor_simulation.py` - Run to verify installation

---

## 🚀 Next Steps

1. ✅ Run the application
2. ✅ Understand Example 30.47 solution
3. ✅ Experiment with different parameters
4. ✅ Explore all tabs
5. ✅ Export and analyze data
6. ✅ Apply to your own motor systems

---

**Enjoy using the DC Motor Braking Simulation System!**

*For detailed information, see README_DC_MOTOR_SIMULATION.md*
