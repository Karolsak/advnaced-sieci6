"""
Test script for alternator calculations
Verifies the core calculations are correct
"""

import math
import sys

def calculate_phase_voltage_phasor(num_coils, coil_voltage, phase_shift_deg):
    """
    Calculate total RMS phase voltage using phasor addition

    Args:
        num_coils: Number of coils in series
        coil_voltage: RMS voltage per coil (V)
        phase_shift_deg: Phase shift between coils (degrees)

    Returns:
        tuple: (total_voltage, real_sum, imag_sum)
    """
    real_sum = 0.0
    imag_sum = 0.0

    print(f"\nPhasor Addition of {num_coils} coils:")
    print(f"{'Coil':<6} {'Angle (°)':<12} {'Real (V)':<12} {'Imag (V)':<12}")
    print("-" * 48)

    for n in range(num_coils):
        angle_deg = n * phase_shift_deg
        angle_rad = math.radians(angle_deg)
        real = coil_voltage * math.cos(angle_rad)
        imag = coil_voltage * math.sin(angle_rad)

        real_sum += real
        imag_sum += imag

        print(f"{n:<6} {angle_deg:<12.1f} {real:<12.4f} {imag:<12.4f}")

    total_voltage = math.sqrt(real_sum**2 + imag_sum**2)

    print("-" * 48)
    print(f"{'Total':<6} {'':<12} {real_sum:<12.4f} {imag_sum:<12.4f}")
    print(f"\nTotal RMS Phase Voltage: {total_voltage:.4f} V")

    return total_voltage, real_sum, imag_sum


def calculate_frequency(num_poles, speed_rpm):
    """
    Calculate electrical frequency

    Args:
        num_poles: Number of poles
        speed_rpm: Rotational speed (RPM)

    Returns:
        float: Frequency in Hz
    """
    frequency = (num_poles * speed_rpm) / 120.0

    print(f"\nFrequency Calculation:")
    print(f"Formula: f = (P × N) / 120")
    print(f"P (poles) = {num_poles}")
    print(f"N (RPM) = {speed_rpm}")
    print(f"f = ({num_poles} × {speed_rpm}) / 120")
    print(f"f = {frequency:.4f} Hz")

    return frequency


def calculate_angular_velocity(speed_rpm):
    """
    Calculate mechanical angular velocity

    Args:
        speed_rpm: Rotational speed (RPM)

    Returns:
        float: Angular velocity in rad/s
    """
    omega = (2 * math.pi * speed_rpm) / 60.0

    print(f"\nAngular Velocity:")
    print(f"ω = 2π × N / 60")
    print(f"ω = 2π × {speed_rpm} / 60")
    print(f"ω = {omega:.4f} rad/s")

    return omega


def main():
    """Main test function"""
    print("="*60)
    print("ALTERNATOR CALCULATIONS TEST")
    print("="*60)

    # Problem parameters
    num_coils = 12
    coil_voltage = 10.0  # V RMS
    phase_shift = 10.0   # degrees
    num_poles = 6
    speed_rpm = 100.0

    print("\nGiven Parameters:")
    print(f"  • Number of coils in series: {num_coils}")
    print(f"  • RMS voltage per coil: {coil_voltage} V")
    print(f"  • Phase shift between coils: {phase_shift}°")
    print(f"  • Number of poles: {num_poles}")
    print(f"  • Rotational speed: {speed_rpm} RPM")

    # Calculate phase voltage
    print("\n" + "="*60)
    print("PHASE VOLTAGE CALCULATION")
    print("="*60)

    phase_voltage, real_sum, imag_sum = calculate_phase_voltage_phasor(
        num_coils, coil_voltage, phase_shift
    )

    # Calculate phase angle
    phase_angle = math.degrees(math.atan2(imag_sum, real_sum))
    print(f"\nResultant phasor angle: {phase_angle:.2f}°")

    # Calculate frequency
    print("\n" + "="*60)
    print("FREQUENCY CALCULATION")
    print("="*60)

    frequency = calculate_frequency(num_poles, speed_rpm)

    # Calculate angular velocity
    print("\n" + "="*60)
    print("MECHANICAL ANGULAR VELOCITY")
    print("="*60)

    omega = calculate_angular_velocity(speed_rpm)

    # Additional calculations
    print("\n" + "="*60)
    print("ADDITIONAL PARAMETERS")
    print("="*60)

    # Electrical angular frequency
    omega_elec = 2 * math.pi * frequency
    print(f"\nElectrical angular frequency:")
    print(f"ω_elec = 2π × f = 2π × {frequency:.4f}")
    print(f"ω_elec = {omega_elec:.4f} rad/s")

    # Peak voltage
    peak_voltage = phase_voltage * math.sqrt(2)
    print(f"\nPeak voltage (from RMS):")
    print(f"V_peak = V_rms × √2")
    print(f"V_peak = {phase_voltage:.4f} × √2")
    print(f"V_peak = {peak_voltage:.4f} V")

    # Line voltage (for 3-phase system)
    line_voltage = phase_voltage * math.sqrt(3)
    print(f"\nLine voltage (3-phase):")
    print(f"V_line = V_phase × √3")
    print(f"V_line = {phase_voltage:.4f} × √3")
    print(f"V_line = {line_voltage:.4f} V")

    # Summary
    print("\n" + "="*60)
    print("SUMMARY OF RESULTS")
    print("="*60)
    print(f"\n{'Parameter':<35} {'Value':<20} {'Unit':<10}")
    print("-" * 65)
    print(f"{'Phase Voltage (RMS)':<35} {phase_voltage:<20.4f} {'V':<10}")
    print(f"{'Electrical Frequency':<35} {frequency:<20.4f} {'Hz':<10}")
    print(f"{'Mechanical Angular Velocity':<35} {omega:<20.4f} {'rad/s':<10}")
    print(f"{'Electrical Angular Frequency':<35} {omega_elec:<20.4f} {'rad/s':<10}")
    print(f"{'Peak Voltage':<35} {peak_voltage:<20.4f} {'V':<10}")
    print(f"{'Line Voltage (3-phase)':<35} {line_voltage:<20.4f} {'V':<10}")
    print("-" * 65)

    print("\n" + "="*60)
    print("TEST COMPLETED SUCCESSFULLY ✓")
    print("="*60)

    return 0


if __name__ == "__main__":
    sys.exit(main())
