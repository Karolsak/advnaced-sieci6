"""
DC Motor Mathematical Models (Standalone - No GUI Dependencies)
Can be imported and used without tkinter
"""

import numpy as np
from scipy.integrate import solve_ivp, odeint

class DCMotorModel:
    """DC Motor Mathematical Model with Differential Equations"""

    def __init__(self, V=400, P=18650, N=45, eta=0.746, Ra=0.2, La=0.05,
                 J=10, B=0.1, Rf=200):
        """
        Initialize DC Motor Parameters
        V: Supply voltage (V)
        P: Rated power (W)
        N: Rated speed (rpm)
        eta: Efficiency
        Ra: Armature resistance (Ω)
        La: Armature inductance (H)
        J: Moment of inertia (kg·m²)
        B: Friction coefficient (N·m·s)
        Rf: Field resistance (Ω)
        """
        self.V = V
        self.P = P
        self.N = N
        self.eta = eta
        self.Ra = Ra
        self.La = La
        self.J = J
        self.B = B
        self.Rf = Rf

        # Calculate derived parameters
        self.omega_rated = (2 * np.pi * N) / 60  # rad/s
        self.If_total = P / (eta * V)  # Full load current
        self.Ish = V / Rf  # Shunt field current
        self.Ia_rated = self.If_total - self.Ish  # Armature current
        self.Eb_rated = V - self.Ia_rated * Ra  # Back EMF
        self.Kt = self.Eb_rated / self.omega_rated  # Torque constant
        self.Ke = self.Kt  # Back EMF constant (same in SI units)
        self.T_rated = P / self.omega_rated  # Rated torque

        # Thermal parameters
        self.thermal_resistance = 2.5  # °C/W
        self.thermal_capacitance = 1000  # J/°C
        self.ambient_temp = 25  # °C
        self.max_temp = 155  # °C (Class F insulation)

        # Mechanical parameters
        self.shaft_diameter = 0.05  # m
        self.shaft_length = 0.3  # m
        self.bearing_friction = 0.05 * self.T_rated

    def calculate_braking_resistance(self, max_current_factor=2):
        """Calculate braking resistance for plugging"""
        Ib_max = max_current_factor * self.If_total
        total_voltage = self.V + self.Eb_rated
        R_total = total_voltage / Ib_max
        Rb = R_total - self.Ra

        # Maximum braking torque
        T_brake_max = (Ib_max / self.Ia_rated) * self.T_rated

        # Braking torque at zero speed
        Ib_zero = self.V / (self.Ra + Rb)
        T_brake_zero = (Ib_zero / self.Ia_rated) * self.T_rated

        return {
            'Rb': Rb,
            'Ib_max': Ib_max,
            'T_brake_max': T_brake_max,
            'T_brake_zero': T_brake_zero,
            'R_total': R_total
        }

    def motor_dynamics_rk45(self, t, y, V_applied, T_load, Rb=0):
        """
        Differential equations for DC motor (for RK45 solver)
        y = [Ia, omega, theta, T_motor]
        """
        Ia, omega, theta, T_motor = y

        # Back EMF
        Eb = self.Ke * omega

        # Armature current derivative
        dIa_dt = (V_applied - Eb - Ia * (self.Ra + Rb)) / self.La

        # Motor torque
        T_motor = self.Kt * Ia

        # Angular acceleration
        domega_dt = (T_motor - T_load - self.B * omega) / self.J

        # Angular position
        dtheta_dt = omega

        # Torque derivative (for tracking)
        dT_dt = self.Kt * dIa_dt

        return [dIa_dt, domega_dt, dtheta_dt, dT_dt]

    def motor_dynamics_euler(self, y, t, V_applied, T_load, Rb=0):
        """
        Differential equations for DC motor (for Euler/odeint solver)
        y = [Ia, omega, theta, T_motor]
        """
        Ia, omega, theta, T_motor = y

        # Back EMF
        Eb = self.Ke * omega

        # Armature current derivative
        dIa_dt = (V_applied - Eb - Ia * (self.Ra + Rb)) / self.La

        # Motor torque
        T_motor_new = self.Kt * Ia

        # Angular acceleration
        domega_dt = (T_motor_new - T_load - self.B * omega) / self.J

        # Angular position
        dtheta_dt = omega

        # Torque derivative
        dT_dt = self.Kt * dIa_dt

        return [dIa_dt, domega_dt, dtheta_dt, dT_dt]

    def thermal_dynamics(self, t, y, P_losses):
        """
        Thermal model differential equation
        y = [T_temp] (temperature)
        """
        T_temp = y[0]

        # Heat transfer equation
        dT_dt = (P_losses - (T_temp - self.ambient_temp) / self.thermal_resistance) / self.thermal_capacitance

        return [dT_dt]

    def calculate_losses(self, Ia, omega):
        """Calculate detailed loss breakdown"""
        # Copper losses (I²R)
        P_copper_armature = Ia**2 * self.Ra
        P_copper_field = self.Ish**2 * self.Rf
        P_copper_total = P_copper_armature + P_copper_field

        # Iron losses (hysteresis + eddy current)
        # Simplified model: proportional to speed squared
        P_iron = 0.02 * self.P * (omega / self.omega_rated)**2

        # Mechanical friction losses
        P_friction = self.bearing_friction * abs(omega)

        # Stray load losses (approximately 1% of rated power)
        P_stray = 0.01 * self.P * (Ia / self.Ia_rated)

        P_total = P_copper_total + P_iron + P_friction + P_stray

        return {
            'copper_armature': P_copper_armature,
            'copper_field': P_copper_field,
            'copper_total': P_copper_total,
            'iron': P_iron,
            'friction': P_friction,
            'stray': P_stray,
            'total': P_total
        }

    def mechanical_stress_analysis(self, T_current):
        """Analyze mechanical stress on shaft and bearings"""
        # Shaft shear stress (τ = 16T / πd³)
        tau_shaft = (16 * abs(T_current)) / (np.pi * self.shaft_diameter**3)

        # Bearing load (simplified)
        bearing_load = abs(T_current) / (self.shaft_diameter / 2)

        # Safety factor (assuming yield strength of steel = 250 MPa)
        yield_strength = 250e6  # Pa
        safety_factor = yield_strength / (tau_shaft + 1e-10)

        return {
            'shaft_stress': tau_shaft / 1e6,  # MPa
            'bearing_load': bearing_load / 1000,  # kN
            'safety_factor': safety_factor
        }


class ControlSystem:
    """Advanced Control System for DC Motor"""

    def __init__(self, motor_model):
        self.motor = motor_model
        self.control_mode = 'voltage'  # 'voltage', 'current', 'speed', 'torque'
        self.thermal_derating = True
        self.current_limit = 2 * motor_model.If_total

    def pi_controller(self, setpoint, current_value, Kp=1.0, Ki=0.1, dt=0.001, integral_sum=0):
        """PI Controller"""
        error = setpoint - current_value
        integral_sum += error * dt
        output = Kp * error + Ki * integral_sum
        return output, integral_sum

    def apply_thermal_derating(self, temperature, nominal_current):
        """Apply thermal derating based on temperature"""
        if not self.thermal_derating:
            return nominal_current

        if temperature > self.motor.max_temp:
            return 0  # Emergency shutdown
        elif temperature > 0.9 * self.motor.max_temp:
            # Linear derating
            derating_factor = (self.motor.max_temp - temperature) / (0.1 * self.motor.max_temp)
            return nominal_current * max(0, derating_factor)
        else:
            return nominal_current

    def current_limiter(self, requested_current):
        """Limit current to safe values"""
        return np.clip(requested_current, -self.current_limit, self.current_limit)
