"""
Test script for DC Motor Braking Simulation
Tests basic functionality without GUI
"""

import sys
import numpy as np
from scipy.integrate import solve_ivp

# Test imports
try:
    print("Testing imports...")
    import numpy as np
    import matplotlib
    matplotlib.use('Agg')  # Non-interactive backend for testing
    import matplotlib.pyplot as plt
    from scipy.integrate import solve_ivp, odeint
    print("✓ All required modules imported successfully")
except ImportError as e:
    print(f"✗ Import error: {e}")
    sys.exit(1)

# Import the motor model classes (without GUI)
try:
    print("\nImporting motor model classes...")
    # Import from standalone model (no GUI dependencies)
    from dc_motor_model import DCMotorModel, ControlSystem
    print("✓ Motor model classes imported successfully")
except ImportError as e:
    print(f"✗ Import error: {e}")
    print("Error: Could not import motor model classes.")
    sys.exit(1)

# Test DCMotorModel class
print("\n" + "="*60)
print("TEST 1: DC Motor Model Initialization")
print("="*60)

try:
    motor = DCMotorModel(V=400, P=18650, N=45, eta=0.746, Ra=0.2)
    print(f"✓ Motor model created successfully")
    print(f"  Rated Power: {motor.P} W")
    print(f"  Rated Speed: {motor.N} rpm")
    print(f"  Rated Current: {motor.If_total:.2f} A")
    print(f"  Rated Torque: {motor.T_rated:.2f} N·m")
    print(f"  Torque Constant: {motor.Kt:.4f} N·m/A")
except Exception as e:
    print(f"✗ Motor model initialization failed: {e}")
    sys.exit(1)

# Test braking resistance calculation (Example 30.47)
print("\n" + "="*60)
print("TEST 2: Example 30.47 - Braking Resistance Calculation")
print("="*60)

try:
    result = motor.calculate_braking_resistance(max_current_factor=2)
    print(f"✓ Braking resistance calculated successfully")
    print(f"\n  RESULTS:")
    print(f"  Braking Resistance (Rb):        {result['Rb']:.3f} Ω")
    print(f"  Maximum Braking Current:        {result['Ib_max']:.2f} A")
    print(f"  Maximum Braking Torque:         {result['T_brake_max']:.2f} N·m")
    print(f"  Braking Torque at Zero Speed:   {result['T_brake_zero']:.2f} N·m")

    # Verify results are reasonable
    assert 6.0 < result['Rb'] < 6.2, "Braking resistance out of expected range"
    assert 120 < result['Ib_max'] < 130, "Maximum current out of expected range"
    assert 7800 < result['T_brake_max'] < 8200, "Maximum torque out of expected range"
    print(f"\n✓ All calculated values are within expected ranges")

except Exception as e:
    print(f"✗ Braking resistance calculation failed: {e}")
    import traceback
    traceback.print_exc()
    sys.exit(1)

# Test differential equations
print("\n" + "="*60)
print("TEST 3: Differential Equations (Motor Dynamics)")
print("="*60)

try:
    # Initial conditions: [Ia, omega, theta, T_motor]
    y0 = [motor.Ia_rated, motor.omega_rated, 0, motor.T_rated]
    t = 0
    V_applied = 400
    T_load = motor.T_rated
    Rb = 0

    # Test RK45 format
    dydt = motor.motor_dynamics_rk45(t, y0, V_applied, T_load, Rb)
    print(f"✓ RK45 dynamics function works")
    print(f"  dIa/dt:    {dydt[0]:.4f} A/s")
    print(f"  dω/dt:     {dydt[1]:.4f} rad/s²")
    print(f"  dθ/dt:     {dydt[2]:.4f} rad/s")
    print(f"  dT/dt:     {dydt[3]:.4f} N·m/s")

    # Test Euler format
    dydt_euler = motor.motor_dynamics_euler(y0, t, V_applied, T_load, Rb)
    print(f"\n✓ Euler dynamics function works")

except Exception as e:
    print(f"✗ Differential equations test failed: {e}")
    import traceback
    traceback.print_exc()
    sys.exit(1)

# Test ODE solver integration
print("\n" + "="*60)
print("TEST 4: ODE Solver Integration (Short Simulation)")
print("="*60)

try:
    # Short simulation test
    t_span = (0, 1.0)  # 1 second
    t_eval = np.linspace(0, 1.0, 100)

    # Starting simulation
    y0 = [0, 0, 0, 0]  # Start from rest
    V_applied = 400
    T_load = 0  # No load
    Rb = 0

    # Solve using RK45
    sol = solve_ivp(
        lambda t, y: motor.motor_dynamics_rk45(t, y, V_applied, T_load, Rb),
        t_span, y0, method='RK45', t_eval=t_eval, max_step=0.01
    )

    print(f"✓ ODE solver completed successfully")
    print(f"  Time points: {len(sol.t)}")
    print(f"  Final current: {sol.y[0][-1]:.2f} A")
    print(f"  Final speed: {sol.y[1][-1] * 60 / (2*np.pi):.2f} rpm")
    print(f"  Final torque: {sol.y[3][-1]:.2f} N·m")

    # Verify solution is reasonable
    assert len(sol.t) > 0, "No time points in solution"
    assert not np.any(np.isnan(sol.y)), "NaN values in solution"
    print(f"\n✓ Solution is valid (no NaN values)")

except Exception as e:
    print(f"✗ ODE solver test failed: {e}")
    import traceback
    traceback.print_exc()
    sys.exit(1)

# Test loss calculations
print("\n" + "="*60)
print("TEST 5: Loss Calculations")
print("="*60)

try:
    losses = motor.calculate_losses(motor.Ia_rated, motor.omega_rated)
    print(f"✓ Loss calculations completed")
    print(f"\n  LOSS BREAKDOWN:")
    print(f"  Copper (Armature):  {losses['copper_armature']:.2f} W")
    print(f"  Copper (Field):     {losses['copper_field']:.2f} W")
    print(f"  Iron Losses:        {losses['iron']:.2f} W")
    print(f"  Friction Losses:    {losses['friction']:.2f} W")
    print(f"  Stray Losses:       {losses['stray']:.2f} W")
    print(f"  Total Losses:       {losses['total']:.2f} W")

    # Verify losses are positive and reasonable
    assert losses['total'] > 0, "Total losses must be positive"
    assert losses['total'] < motor.P, "Total losses exceed motor power"
    print(f"\n✓ Loss values are physically reasonable")

except Exception as e:
    print(f"✗ Loss calculation test failed: {e}")
    import traceback
    traceback.print_exc()
    sys.exit(1)

# Test mechanical stress analysis
print("\n" + "="*60)
print("TEST 6: Mechanical Stress Analysis")
print("="*60)

try:
    stress = motor.mechanical_stress_analysis(motor.T_rated)
    print(f"✓ Mechanical stress analysis completed")
    print(f"\n  STRESS ANALYSIS:")
    print(f"  Shaft Stress:       {stress['shaft_stress']:.2f} MPa")
    print(f"  Bearing Load:       {stress['bearing_load']:.2f} kN")
    print(f"  Safety Factor:      {stress['safety_factor']:.2f}")

    # Verify stress values are reasonable
    assert stress['shaft_stress'] > 0, "Shaft stress must be positive"
    assert stress['safety_factor'] > 1, "Safety factor should be > 1"
    print(f"\n✓ Stress analysis values are reasonable")

except Exception as e:
    print(f"✗ Mechanical stress test failed: {e}")
    import traceback
    traceback.print_exc()
    sys.exit(1)

# Test control system
print("\n" + "="*60)
print("TEST 7: Control System")
print("="*60)

try:
    control = ControlSystem(motor)
    print(f"✓ Control system initialized")
    print(f"  Control Mode:       {control.control_mode}")
    print(f"  Current Limit:      {control.current_limit:.2f} A")

    # Test PI controller
    output, integral = control.pi_controller(100, 90, Kp=1.0, Ki=0.1, dt=0.001)
    print(f"\n✓ PI controller works")
    print(f"  Controller Output:  {output:.4f}")

    # Test thermal derating
    derated_current = control.apply_thermal_derating(140, 100)
    print(f"\n✓ Thermal derating works")
    print(f"  Derated Current:    {derated_current:.2f} A")

    # Test current limiter
    limited_current = control.current_limiter(200)
    print(f"\n✓ Current limiter works")
    print(f"  Limited Current:    {limited_current:.2f} A")

except Exception as e:
    print(f"✗ Control system test failed: {e}")
    import traceback
    traceback.print_exc()
    sys.exit(1)

# Summary
print("\n" + "="*60)
print("TEST SUMMARY")
print("="*60)
print("""
✓ All tests passed successfully!

The DC Motor Braking Simulation is ready to use.

To run the full GUI application:
    python3 dc_motor_braking_simulation.py

For headless testing:
    python3 test_dc_motor_simulation.py

Core functionality verified:
  1. Motor model initialization
  2. Example 30.47 solution (braking resistance)
  3. Differential equations (motor dynamics)
  4. ODE solver integration
  5. Loss calculations
  6. Mechanical stress analysis
  7. Control system operations

""")

print("="*60)
print("All systems operational!")
print("="*60)
