"""
Verification Script for Train Braking Energy Calculations
This script verifies the analytical solution without GUI
"""

import numpy as np


def calculate_train_braking_energy():
    """
    Calculate train braking energy for the given problem:
    - 400-tonne train
    - Gradient: 1 in 100
    - Time: 20 seconds
    - Speed: 80 km/h → 50 km/h
    - Tractive resistance: 49 N/t
    - Rotational inertia: 7.5%
    - Motor efficiency: 75%
    """

    print("="*70)
    print("TRAIN BRAKING ENERGY RECOVERY CALCULATION")
    print("="*70)
    print()

    # Input parameters
    mass = 400 * 1000  # kg (400 tonnes)
    gradient = 1/100  # 1 in 100
    v1 = 80 / 3.6  # m/s (initial speed)
    v2 = 50 / 3.6  # m/s (final speed)
    time = 20  # seconds
    resistance = 49  # N/t
    rot_inertia = 7.5 / 100  # 7.5%
    efficiency = 75 / 100  # 75%

    print("INPUT PARAMETERS:")
    print(f"  Train mass:              {mass/1000:.1f} tonnes ({mass:.0f} kg)")
    print(f"  Gradient:                1 in {1/gradient:.0f} ({gradient*100:.2f}%)")
    print(f"  Initial speed:           {v1*3.6:.1f} km/h ({v1:.2f} m/s)")
    print(f"  Final speed:             {v2*3.6:.1f} km/h ({v2:.2f} m/s)")
    print(f"  Braking time:            {time:.1f} s")
    print(f"  Tractive resistance:     {resistance:.1f} N/t")
    print(f"  Rotational inertia:      {rot_inertia*100:.1f}%")
    print(f"  Motor efficiency:        {efficiency*100:.1f}%")
    print()

    # Calculate effective mass (including rotational inertia)
    m_eff = mass * (1 + rot_inertia)
    print(f"EFFECTIVE MASS:")
    print(f"  m_eff = m × (1 + k_rot)")
    print(f"  m_eff = {mass:.0f} × (1 + {rot_inertia:.3f})")
    print(f"  m_eff = {m_eff:.0f} kg")
    print()

    # Calculate kinetic energy change
    KE1 = 0.5 * m_eff * v1**2
    KE2 = 0.5 * m_eff * v2**2
    delta_KE = KE1 - KE2

    print("KINETIC ENERGY CHANGE:")
    print(f"  KE₁ = ½ × m_eff × v₁²")
    print(f"  KE₁ = ½ × {m_eff:.0f} × {v1:.2f}²")
    print(f"  KE₁ = {KE1/1e6:.3f} MJ")
    print()
    print(f"  KE₂ = ½ × m_eff × v₂²")
    print(f"  KE₂ = ½ × {m_eff:.0f} × {v2:.2f}²")
    print(f"  KE₂ = {KE2/1e6:.3f} MJ")
    print()
    print(f"  ΔKE = KE₁ - KE₂")
    print(f"  ΔKE = {KE1/1e6:.3f} - {KE2/1e6:.3f}")
    print(f"  ΔKE = {delta_KE/1e6:.3f} MJ")
    print()

    # Calculate distance traveled
    v_avg = (v1 + v2) / 2
    distance = v_avg * time

    print("DISTANCE AND GRADIENT:")
    print(f"  v_avg = (v₁ + v₂) / 2")
    print(f"  v_avg = ({v1:.2f} + {v2:.2f}) / 2")
    print(f"  v_avg = {v_avg:.2f} m/s ({v_avg*3.6:.2f} km/h)")
    print()
    print(f"  distance = v_avg × t")
    print(f"  distance = {v_avg:.2f} × {time}")
    print(f"  distance = {distance:.2f} m")
    print()

    # Calculate potential energy change (going downhill)
    height_change = distance * gradient
    delta_PE = mass * 9.81 * height_change

    print("POTENTIAL ENERGY (from gradient):")
    print(f"  height_change = distance × gradient")
    print(f"  height_change = {distance:.2f} × {gradient:.4f}")
    print(f"  height_change = {height_change:.2f} m (descent)")
    print()
    print(f"  ΔPE = m × g × h")
    print(f"  ΔPE = {mass:.0f} × 9.81 × {height_change:.2f}")
    print(f"  ΔPE = {delta_PE/1e6:.3f} MJ")
    print("  (Positive because descending gives energy)")
    print()

    # Calculate energy lost to resistance
    total_resistance_force = resistance * (mass / 1000)  # N/t × tonnes = N
    energy_resistance = total_resistance_force * distance

    print("ENERGY LOST TO RESISTANCE:")
    print(f"  F_resistance = {resistance} N/t × {mass/1000:.0f} t")
    print(f"  F_resistance = {total_resistance_force:.2f} N")
    print()
    print(f"  E_resistance = F_resistance × distance")
    print(f"  E_resistance = {total_resistance_force:.2f} × {distance:.2f}")
    print(f"  E_resistance = {energy_resistance/1e6:.3f} MJ")
    print()

    # Total mechanical energy available
    total_mech_energy = delta_KE + delta_PE - energy_resistance

    print("TOTAL MECHANICAL ENERGY:")
    print(f"  E_total = ΔKE + ΔPE - E_resistance")
    print(f"  E_total = {delta_KE/1e6:.3f} + {delta_PE/1e6:.3f} - {energy_resistance/1e6:.3f}")
    print(f"  E_total = {total_mech_energy/1e6:.3f} MJ")
    print()

    # Energy returned to lines (with motor efficiency)
    energy_returned = total_mech_energy * efficiency
    energy_returned_kwh = energy_returned / 1e3  # Convert to kWh (MJ to kWh: divide by 3.6, but we have J so divide by 3.6e6, or MJ*1000/3.6)
    energy_returned_kwh = energy_returned / 3.6e6 * 1e6  # Simpler: J / 3.6e6
    energy_returned_kwh = energy_returned / 1e3  # Actually J to kWh is J/3600000, but we work in J

    # Correct conversion: J to kWh = J / 3,600,000
    energy_returned_kwh = energy_returned / 3.6e6

    print("ENERGY RETURNED TO LINES:")
    print(f"  E_returned = E_total × η_motor")
    print(f"  E_returned = {total_mech_energy/1e6:.3f} MJ × {efficiency:.2f}")
    print(f"  E_returned = {energy_returned/1e6:.3f} MJ")
    print(f"  E_returned = {energy_returned_kwh:.2f} kWh")
    print()

    # Calculate average power
    avg_power = energy_returned / time

    print("POWER ANALYSIS:")
    print(f"  P_avg = E_returned / time")
    print(f"  P_avg = {energy_returned:.0f} J / {time} s")
    print(f"  P_avg = {avg_power:.2f} W")
    print(f"  P_avg = {avg_power/1e3:.2f} kW")
    print(f"  P_avg = {avg_power/1e6:.3f} MW")
    print()

    # Additional calculations
    deceleration = (v2 - v1) / time
    braking_force = m_eff * abs(deceleration)

    print("DYNAMICS:")
    print(f"  Deceleration = (v₂ - v₁) / t")
    print(f"  Deceleration = ({v2:.2f} - {v1:.2f}) / {time}")
    print(f"  Deceleration = {deceleration:.3f} m/s²")
    print(f"  |Deceleration| = {abs(deceleration):.3f} m/s²")
    print()
    print(f"  Braking force = m_eff × |deceleration|")
    print(f"  Braking force = {m_eff:.0f} × {abs(deceleration):.3f}")
    print(f"  Braking force = {braking_force:.2f} N")
    print(f"  Braking force = {braking_force/1e3:.2f} kN")
    print()

    # Summary box
    print("="*70)
    print("FINAL ANSWER:")
    print("="*70)
    print(f"  ENERGY RETURNED TO LINES = {energy_returned/1e6:.3f} MJ ({energy_returned_kwh:.2f} kWh)")
    print(f"  AVERAGE POWER           = {avg_power/1e3:.2f} kW")
    print("="*70)
    print()

    # Verification with energy conservation
    print("ENERGY CONSERVATION CHECK:")
    total_input = delta_KE + delta_PE
    total_output = energy_resistance + energy_returned
    balance = total_input - total_output
    print(f"  Input energy:     {total_input/1e6:.3f} MJ (ΔKE + ΔPE)")
    print(f"  Output energy:    {total_output/1e6:.3f} MJ (Resistance + Returned)")
    print(f"  Balance:          {balance/1e6:.6f} MJ")
    print(f"  Balance:          {balance/1e3:.3f} kJ")

    # Account for efficiency loss
    efficiency_loss = total_mech_energy * (1 - efficiency)
    total_accounted = energy_resistance + energy_returned + efficiency_loss
    print()
    print(f"  Including efficiency losses:")
    print(f"    Resistance:     {energy_resistance/1e6:.3f} MJ")
    print(f"    Returned:       {energy_returned/1e6:.3f} MJ")
    print(f"    Motor losses:   {efficiency_loss/1e6:.3f} MJ")
    print(f"    Total:          {total_accounted/1e6:.3f} MJ")
    print(f"    Input:          {total_input/1e6:.3f} MJ")
    print(f"    Balance:        {(total_input - total_accounted)/1e6:.6f} MJ ✓")
    print()

    return energy_returned, energy_returned_kwh, avg_power


if __name__ == "__main__":
    energy_j, energy_kwh, power_w = calculate_train_braking_energy()

    print("\nVERIFICATION COMPLETE!")
    print(f"\nThe train returns {energy_kwh:.2f} kWh to the electrical lines")
    print(f"This corresponds to an average regenerative power of {power_w/1e3:.2f} kW")
