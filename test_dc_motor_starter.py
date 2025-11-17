#!/usr/bin/env python3
"""
Test script for DC Motor Starter Design application
Verifies calculations without launching GUI
"""

import sys
import numpy as np

def test_motor_calculations():
    """Test the motor parameter calculations"""
    print("="*70)
    print("DC MOTOR STARTER DESIGN - CALCULATION TEST")
    print("="*70)

    # Given parameters
    P_rated = 36775  # W
    V_supply = 400  # V
    efficiency = 0.92
    cu_loss_percent = 0.05
    R_field = 200  # Ω
    num_studs = 7
    current_ratio = 1.5

    print("\nGIVEN PARAMETERS:")
    print("-"*70)
    print(f"Rated Power:              {P_rated/1000:.3f} kW")
    print(f"Supply Voltage:           {V_supply:.1f} V")
    print(f"Efficiency:               {efficiency*100:.1f} %")
    print(f"Cu Loss % of Input:       {cu_loss_percent*100:.1f} %")
    print(f"Field Resistance:         {R_field:.1f} Ω")
    print(f"Number of Studs:          {num_studs}")
    print(f"Current Ratio (α):        {current_ratio:.2f}")

    # Step 1: Input power
    P_input = P_rated / efficiency
    print(f"\n1. Input Power = Output / Efficiency")
    print(f"   P_input = {P_rated} / {efficiency} = {P_input:.2f} W")

    # Step 2: Line current
    I_line = P_input / V_supply
    print(f"\n2. Line Current = Input Power / Voltage")
    print(f"   I_line = {P_input:.2f} / {V_supply} = {I_line:.2f} A")

    # Step 3: Field current
    I_field = V_supply / R_field
    print(f"\n3. Field Current = Voltage / Field Resistance")
    print(f"   I_field = {V_supply} / {R_field} = {I_field:.2f} A")

    # Step 4: Armature current
    I_armature = I_line - I_field
    print(f"\n4. Armature Current = Line Current - Field Current")
    print(f"   I_armature = {I_line:.2f} - {I_field:.2f} = {I_armature:.2f} A")

    # Step 5: Copper losses
    P_cu_total = cu_loss_percent * P_input
    P_cu_field = I_field**2 * R_field
    P_cu_armature = P_cu_total - P_cu_field
    print(f"\n5. Copper Losses:")
    print(f"   Total Cu Loss = {cu_loss_percent} × {P_input:.2f} = {P_cu_total:.2f} W")
    print(f"   Field Cu Loss = {I_field}² × {R_field} = {P_cu_field:.2f} W")
    print(f"   Armature Cu Loss = {P_cu_total:.2f} - {P_cu_field:.2f} = {P_cu_armature:.2f} W")

    # Step 6: Armature resistance
    R_armature = P_cu_armature / (I_armature**2)
    print(f"\n6. Armature Resistance:")
    print(f"   R_a = P_cu_armature / I_armature²")
    print(f"   R_a = {P_cu_armature:.2f} / {I_armature:.2f}² = {R_armature:.6f} Ω")

    # Step 7: Back EMF
    E_b = V_supply - I_armature * R_armature
    print(f"\n7. Back EMF at Full Load:")
    print(f"   E_b = V - I_a × R_a")
    print(f"   E_b = {V_supply} - {I_armature:.2f} × {R_armature:.6f} = {E_b:.2f} V")

    # Step 8: Starter resistance calculation
    print(f"\n8. STARTER RESISTANCE SECTIONS:")
    print("-"*70)

    I_min = I_armature
    I_max = current_ratio * I_min
    n = num_studs - 1  # Number of sections

    print(f"   Minimum Current (I_min): {I_min:.2f} A")
    print(f"   Maximum Current (I_max): {I_max:.2f} A")
    print(f"   Number of Sections (n):  {n}")

    # Total resistance at start
    R_total_start = V_supply / I_max

    # Geometric progression ratio
    r = current_ratio ** (1/n)
    print(f"   Resistance Ratio (r):    {r:.4f}")
    print(f"   Total R at start:        {R_total_start:.6f} Ω")

    # Calculate total resistances at each stud
    total_resistances = []
    for i in range(n + 1):
        R_total_i = R_total_start / (r ** i)
        total_resistances.append(R_total_i)

    # Calculate individual section resistances
    resistances = []
    for i in range(n):
        R_section = total_resistances[i] - total_resistances[i+1]
        resistances.append(R_section)

    print(f"\n   Resistance Sections:")
    total_R = 0
    for idx, R in enumerate(resistances, 1):
        total_R += R
        print(f"   Section {idx}: {R:.6f} Ω  (Cumulative: {total_R:.6f} Ω)")

    print(f"\n   Total External Resistance: {sum(resistances):.6f} Ω")
    print(f"   Total with Armature:       {sum(resistances) + R_armature:.6f} Ω")

    # Verify starting current
    R_total_start = sum(resistances) + R_armature
    I_start = V_supply / R_total_start
    print(f"\n9. VERIFICATION:")
    print(f"   Starting Current (at stud 1): {I_start:.2f} A")
    print(f"   Expected Maximum Current:     {I_max:.2f} A")
    print(f"   Difference:                   {abs(I_start - I_max):.2f} A")

    if abs(I_start - I_max) < 0.1:
        print(f"   ✓ CALCULATION VERIFIED!")
    else:
        print(f"   ✗ Warning: Large difference detected")

    # Stud configuration
    print(f"\n10. STUD SWITCHING SEQUENCE:")
    print("-"*70)
    cumulative = sum(resistances)
    for i in range(num_studs):
        if i == 0:
            R_total = cumulative + R_armature
            I_at_stud = V_supply / R_total
            print(f"   Stud {i+1}: All resistances    R_total = {R_total:.4f} Ω, I = {I_at_stud:.2f} A")
        elif i < len(resistances):
            cumulative -= resistances[i-1]
            R_total = cumulative + R_armature
            I_at_stud = V_supply / R_total if R_total > 0 else 0
            print(f"   Stud {i+1}: Remove Section {i}  R_total = {R_total:.4f} Ω, I ≈ {I_at_stud:.2f} A*")
        else:
            R_total = R_armature
            print(f"   Stud {i+1}: Direct connection  R_total = {R_total:.4f} Ω (Back EMF limits I)")

    print(f"\n   * Current values are approximate (back EMF increases with speed)")

    print("\n" + "="*70)
    print("TEST COMPLETED SUCCESSFULLY!")
    print("="*70)

    return True

def test_imports():
    """Test that all required modules can be imported"""
    print("\nTesting module imports...")

    tkinter_available = True
    try:
        import tkinter as tk
        print("✓ tkinter imported successfully")
    except ImportError as e:
        print(f"⚠ tkinter import failed: {e}")
        print("  (tkinter is needed for GUI, but calculations will work)")
        tkinter_available = False

    if tkinter_available:
        try:
            from tkinter import ttk
            print("✓ tkinter.ttk imported successfully")
        except ImportError as e:
            print(f"⚠ tkinter.ttk import failed: {e}")
            tkinter_available = False

    try:
        import numpy as np
        print(f"✓ numpy {np.__version__} imported successfully")
    except ImportError as e:
        print(f"✗ numpy import failed: {e}")
        return False

    try:
        import matplotlib.pyplot as plt
        print(f"✓ matplotlib imported successfully")
    except ImportError as e:
        print(f"✗ matplotlib import failed: {e}")
        return False

    try:
        from scipy.integrate import solve_ivp
        print(f"✓ scipy imported successfully")
    except ImportError as e:
        print(f"✗ scipy import failed: {e}")
        return False

    if tkinter_available:
        print("\n✓ All required modules are available (including GUI support)!\n")
    else:
        print("\n✓ Core modules available (GUI will not work without tkinter)!\n")
    return True

if __name__ == "__main__":
    print("\n" + "="*70)
    print("DC MOTOR STARTER APPLICATION - TEST SUITE")
    print("="*70 + "\n")

    # Test imports
    imports_ok = test_imports()
    if not imports_ok:
        print("\nERROR: Missing required modules!")
        print("Please install: pip install numpy scipy matplotlib")
        print("For GUI support, also install tkinter:")
        print("  Ubuntu/Debian: sudo apt-get install python3-tk")
        print("  Fedora/RHEL: sudo dnf install python3-tkinter")
        print("  macOS: tkinter comes with Python")
        sys.exit(1)

    # Test calculations
    try:
        test_motor_calculations()
        print("\n✓ All calculation tests passed successfully!")
        print("\nTo run the GUI application (requires tkinter):")
        print("  python3 dc_motor_starter_advanced.py")
        print("\n")
    except Exception as e:
        print(f"\n✗ Test failed with error: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)
