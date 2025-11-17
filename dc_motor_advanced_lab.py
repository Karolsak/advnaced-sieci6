"""
Advanced DC Motor Control and Analysis Laboratory
Comprehensive multi-physics simulation with GUI
Solves Problems 13 & 14 and provides advanced motor analysis
"""

import tkinter as tk
from tkinter import ttk, messagebox
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure
from scipy.integrate import odeint, solve_ivp
import threading
import time
from dataclasses import dataclass
from typing import Tuple, List, Dict
import queue


# ====================== PROBLEM SOLVERS ======================

class MotorProblemSolver:
    """Solves specific motor problems"""

    @staticmethod
    def solve_problem_13():
        """
        Problem 13: 440-V shunt motor speed control
        Initial: 1500 rpm, 30 A armature current, 15 hp output
        Load torque ∝ speed²
        Find: Resistance for 1300 rpm and new current
        """
        print("\n" + "="*60)
        print("PROBLEM 13: Shunt Motor Speed Control")
        print("="*60)

        # Given parameters
        V = 440  # Supply voltage (V)
        N1 = 1500  # Initial speed (rpm)
        Ia1 = 30  # Initial armature current (A)
        P_out = 15 * 746  # Output power (W), 15 hp
        N2 = 1300  # Desired speed (rpm)

        # Calculate initial back EMF
        # For shunt motor: Eb = V - Ia*Ra
        # First, find Ra from power equation
        # P_out = Eb * Ia (approximately, neglecting losses)
        Eb1 = P_out / Ia1
        Ra = (V - Eb1) / Ia1

        print(f"Initial conditions:")
        print(f"  Supply voltage: {V} V")
        print(f"  Initial speed: {N1} rpm")
        print(f"  Initial armature current: {Ia1} A")
        print(f"  Output power: {P_out/746:.1f} hp ({P_out:.0f} W)")
        print(f"  Armature resistance: {Ra:.4f} Ω")
        print(f"  Initial back EMF: {Eb1:.2f} V")

        # For shunt motor: Eb ∝ N (flux constant)
        # Eb1/N1 = Eb2/N2
        Eb2 = Eb1 * (N2 / N1)

        # Load torque varies as speed²
        # T ∝ N²
        # Also, T ∝ Ia (flux constant for shunt motor)
        # Therefore: Ia ∝ N²
        Ia2 = Ia1 * (N2 / N1)**2

        # With external resistance R in series:
        # V = Eb2 + Ia2*(Ra + R)
        R = (V - Eb2) / Ia2 - Ra

        print(f"\nFinal conditions at {N2} rpm:")
        print(f"  New back EMF: {Eb2:.2f} V")
        print(f"  New armature current: {Ia2:.2f} A")
        print(f"  Required series resistance: {R:.2f} Ω")

        print(f"\nExpected answers: 2.97 Ω, 22.5 A")
        print(f"Calculated answers: {R:.2f} Ω, {Ia2:.1f} A")
        print("="*60 + "\n")

        return {
            'R': R,
            'Ia2': Ia2,
            'N2': N2,
            'Eb2': Eb2,
            'Ra': Ra
        }

    @staticmethod
    def solve_problem_14():
        """
        Problem 14: 460-V series motor with field flux reduction
        Initial: 1000 rpm, 25 A, Ra = 0.4 Ω
        Flux reduced by 5%
        Load torque ∝ speed², same efficiency
        Find: New current and speed
        """
        print("\n" + "="*60)
        print("PROBLEM 14: Series Motor Field Flux Reduction")
        print("="*60)

        # Given parameters
        V = 460  # Supply voltage (V)
        Ra = 0.4  # Armature resistance (Ω)
        I1 = 25  # Initial current (A)
        N1 = 1000  # Initial speed (rpm)
        flux_reduction = 0.05  # 5% flux reduction

        # Calculate initial back EMF
        Eb1 = V - I1 * Ra

        print(f"Initial conditions:")
        print(f"  Supply voltage: {V} V")
        print(f"  Armature resistance: {Ra} Ω")
        print(f"  Initial current: {I1} A")
        print(f"  Initial speed: {N1} rpm")
        print(f"  Initial back EMF: {Eb1:.2f} V")

        # For series motor:
        # Eb = K * φ * N, where φ ∝ I (in linear region)
        # T = K * φ * I ∝ I²

        # After flux reduction: φ2 = φ1 * (1 - 0.05) = 0.95 * φ1
        # But φ ∝ I, so we need to account for this

        # Load torque: T ∝ N²
        # T1 = k * N1²
        # T2 = k * N2²

        # For series motor: T = K * φ * I
        # With φ ∝ I: T = K' * I²

        # Same efficiency means: (Eb*I)/(V*I) is constant
        # Eb/V is constant

        # After flux reduction by 5%:
        # Eb2 = K * 0.95 * φ1 * (I2/I1) * N2
        # But φ1 ∝ I1, so:
        # Eb1 = K * I1 * N1
        # With flux control reducing field by 5%, effectively:
        # Eb2 = V - I2 * Ra

        # Torque balance: T1/T2 = N1²/N2²
        # For series motor with flux weakening:
        # T ∝ φ * I
        # If flux reduced by 5%: φ2 = 0.95 * φ1 * (I2/I1)
        # T ∝ I²

        # T2/T1 = (0.95 * I2)² / I1² = N2²/N1²

        # Also: Eb ∝ φ * N
        # Eb1/Eb2 = (I1 * N1)/(0.95 * I2 * N2)

        # From voltage equation:
        # Eb1 = V - I1*Ra = 450 V
        # Eb2 = V - I2*Ra

        # Same efficiency: Eb1/V = Eb2/V (approximately)
        # This gives: I1 = I2 (but this contradicts flux reduction)

        # More accurate approach:
        # T ∝ φ * I, with flux control φ2 = 0.95*φ1, and φ ∝ I
        # T1 = k * I1²
        # T2 = k * 0.95 * I2² (due to 5% flux reduction in field)

        # T ∝ N²:
        # T2/T1 = N2²/N1²
        # (0.95 * I2²)/I1² = N2²/N1²

        # Eb = K * φ * N
        # Eb1 = K * I1 * N1
        # Eb2 = K * 0.95 * I2 * N2

        # Eb1/Eb2 = (I1 * N1)/(0.95 * I2 * N2)
        # (V - I1*Ra)/(V - I2*Ra) = (I1 * N1)/(0.95 * I2 * N2)

        # From torque: I2 = I1 * sqrt(N2²/N1²) / sqrt(0.95)
        # I2 = I1 * N2/(N1 * sqrt(0.95))

        # Solving numerically:
        # Let x = I2/I1 and y = N2/N1
        # From torque: 0.95 * x² = y²  =>  y = sqrt(0.95) * x
        # From Eb: (V - I1*Ra)/(V - x*I1*Ra) = (N1)/(0.95 * x * y * N1)
        # (V - I1*Ra)/(V - x*I1*Ra) = 1/(0.95 * x * y)

        # Substitute y = sqrt(0.95) * x:
        # (V - I1*Ra)/(V - x*I1*Ra) = 1/(0.95 * x * sqrt(0.95) * x)
        # (V - I1*Ra)/(V - x*I1*Ra) = 1/(0.95^1.5 * x²)

        # Solving: 0.95^1.5 * x² * (V - I1*Ra) = V - x*I1*Ra

        def equation(x):
            lhs = (0.95**1.5) * (x**2) * (V - I1*Ra)
            rhs = V - x*I1*Ra
            return lhs - rhs

        # Numerical solution
        from scipy.optimize import fsolve
        x_solution = fsolve(equation, 1.0)[0]
        I2 = x_solution * I1
        N2 = np.sqrt(0.95) * x_solution * N1

        Eb2 = V - I2 * Ra

        print(f"\nFinal conditions after 5% flux reduction:")
        print(f"  New current: {I2:.2f} A")
        print(f"  New speed: {N2:.0f} rpm")
        print(f"  New back EMF: {Eb2:.2f} V")

        print(f"\nVerification:")
        T1 = I1**2
        T2 = 0.95 * I2**2
        print(f"  Torque ratio T2/T1 = {T2/T1:.4f}")
        print(f"  Speed² ratio N2²/N1² = {(N2/N1)**2:.4f}")
        print(f"  (Should be equal for load ∝ N²)")

        print("="*60 + "\n")

        return {
            'I2': I2,
            'N2': N2,
            'Eb2': Eb2,
            'flux_reduction': flux_reduction
        }


# ====================== MOTOR MODELS ======================

@dataclass
class MotorParameters:
    """DC Motor physical parameters"""
    V_supply: float = 440.0  # Supply voltage (V RMS)
    Ra: float = 0.5  # Armature resistance (Ω)
    La: float = 0.05  # Armature inductance (H)
    Rf: float = 220.0  # Field resistance (Ω)
    Lf: float = 50.0  # Field inductance (H)
    J: float = 0.5  # Moment of inertia (kg·m²)
    B: float = 0.1  # Viscous friction coefficient (N·m·s/rad)
    Kt: float = 1.5  # Torque constant (N·m/A)
    Ke: float = 1.5  # Back EMF constant (V·s/rad)

    # Thermal parameters
    thermal_resistance: float = 2.5  # K/W
    thermal_capacitance: float = 1000.0  # J/K
    ambient_temp: float = 25.0  # °C
    max_temp: float = 155.0  # °C (Class F insulation)

    # Mechanical parameters
    shaft_diameter: float = 0.05  # m
    shaft_length: float = 0.3  # m
    shaft_modulus: float = 200e9  # Pa (steel)

    # Loss coefficients
    iron_loss_coef: float = 50.0  # W at rated speed
    mechanical_loss_coef: float = 30.0  # W
    stray_loss_coef: float = 0.01  # Fraction of output


class DCMotorModel:
    """Advanced DC motor model with multi-physics coupling"""

    def __init__(self, params: MotorParameters, motor_type: str = 'shunt'):
        self.params = params
        self.motor_type = motor_type  # 'shunt' or 'series'

        # State variables: [Ia, If, omega, theta, temp]
        self.state = np.array([0.0, 0.0, 0.0, 0.0, params.ambient_temp])

        # History for plotting
        self.time_history = []
        self.state_history = []
        self.loss_history = []

        # Control inputs
        self.V_applied = params.V_supply
        self.R_external = 0.0
        self.load_torque = 0.0

    def differential_equations(self, t, y):
        """
        Coupled electromagnetic-thermal-mechanical equations
        y = [Ia, If, omega, theta, T_motor]
        """
        Ia, If, omega, theta, T_motor = y

        # Temperature-dependent resistance (copper has +0.393%/°C)
        temp_coef = 0.00393
        Ra_temp = self.params.Ra * (1 + temp_coef * (T_motor - self.params.ambient_temp))
        Rf_temp = self.params.Rf * (1 + temp_coef * (T_motor - self.params.ambient_temp))

        # Back EMF
        if self.motor_type == 'shunt':
            Eb = self.params.Ke * If * omega
        else:  # series
            Eb = self.params.Ke * Ia * omega

        # Armature circuit equation
        if self.motor_type == 'shunt':
            dIa_dt = (self.V_applied - Eb - Ia * (Ra_temp + self.R_external)) / self.params.La
        else:  # series
            dIa_dt = (self.V_applied - Eb - Ia * (Ra_temp + Rf_temp + self.R_external)) / (self.params.La + self.params.Lf)

        # Field circuit equation
        if self.motor_type == 'shunt':
            dIf_dt = (self.V_applied - If * Rf_temp) / self.params.Lf
        else:  # series
            dIf_dt = dIa_dt  # Same current in series

        # Electromagnetic torque
        if self.motor_type == 'shunt':
            Te = self.params.Kt * If * Ia
        else:  # series
            Te = self.params.Kt * Ia * Ia  # Ia = If for series

        # Mechanical equation with speed-dependent load
        T_friction = self.params.B * omega
        T_net = Te - self.load_torque - T_friction
        domega_dt = T_net / self.params.J

        # Angular position
        dtheta_dt = omega

        # Thermal equation - heat transfer
        # Heat generated
        P_copper_armature = Ia**2 * Ra_temp
        P_copper_field = If**2 * Rf_temp
        P_iron = self.params.iron_loss_coef * (omega / (2 * np.pi / 60 * 1500))**2  # Proportional to speed²
        P_mechanical = self.params.mechanical_loss_coef * abs(omega)
        P_stray = self.params.stray_loss_coef * abs(Te * omega)

        P_total_loss = P_copper_armature + P_copper_field + P_iron + P_mechanical + P_stray

        # Heat dissipation
        P_dissipated = (T_motor - self.params.ambient_temp) / self.params.thermal_resistance

        # Temperature rise
        dT_dt = (P_total_loss - P_dissipated) / self.params.thermal_capacitance

        return [dIa_dt, dIf_dt, domega_dt, dtheta_dt, dT_dt]

    def rk45_step(self, dt):
        """Runge-Kutta 4th order integration step"""
        t = 0
        y = self.state.copy()

        k1 = np.array(self.differential_equations(t, y))
        k2 = np.array(self.differential_equations(t + dt/2, y + dt*k1/2))
        k3 = np.array(self.differential_equations(t + dt/2, y + dt*k2/2))
        k4 = np.array(self.differential_equations(t + dt, y + dt*k3))

        self.state = y + (dt/6) * (k1 + 2*k2 + 2*k3 + k4)

    def euler_step(self, dt):
        """Euler integration step (simpler, less accurate)"""
        dydt = np.array(self.differential_equations(0, self.state))
        self.state = self.state + dt * dydt

    def calculate_losses(self):
        """Calculate detailed loss breakdown"""
        Ia, If, omega, theta, T_motor = self.state

        temp_coef = 0.00393
        Ra_temp = self.params.Ra * (1 + temp_coef * (T_motor - self.params.ambient_temp))
        Rf_temp = self.params.Rf * (1 + temp_coef * (T_motor - self.params.ambient_temp))

        losses = {
            'copper_armature': Ia**2 * Ra_temp,
            'copper_field': If**2 * Rf_temp,
            'iron': self.params.iron_loss_coef * (omega / (2 * np.pi / 60 * 1500))**2,
            'mechanical': self.params.mechanical_loss_coef * abs(omega),
            'stray': self.params.stray_loss_coef * abs(self.get_torque() * omega)
        }

        losses['total'] = sum(losses.values())

        return losses

    def get_torque(self):
        """Get current electromagnetic torque"""
        Ia, If, omega, theta, T_motor = self.state
        if self.motor_type == 'shunt':
            return self.params.Kt * If * Ia
        else:
            return self.params.Kt * Ia * Ia

    def get_power_output(self):
        """Get mechanical power output"""
        omega = self.state[2]
        return self.get_torque() * omega

    def get_efficiency(self):
        """Calculate efficiency"""
        Ia, If = self.state[0], self.state[1]
        P_in = self.V_applied * (Ia + If if self.motor_type == 'shunt' else Ia)
        P_out = self.get_power_output()

        if P_in > 0:
            return (P_out / P_in) * 100
        return 0

    def calculate_shaft_stress(self):
        """Calculate mechanical stress on shaft"""
        torque = self.get_torque()
        radius = self.shaft_diameter / 2

        # Torsional shear stress: τ = T*r/J
        # J = π*d⁴/32 for solid circular shaft
        J_shaft = np.pi * self.params.shaft_diameter**4 / 32
        shear_stress = torque * radius / J_shaft

        return {
            'shear_stress': shear_stress,  # Pa
            'max_stress': shear_stress,  # Pa
            'safety_factor': 250e6 / max(abs(shear_stress), 1)  # Assuming yield strength 250 MPa
        }


# ====================== GUI APPLICATION ======================

class AdvancedMotorLab(tk.Tk):
    """Main application window"""

    def __init__(self):
        super().__init__()

        self.title("Advanced DC Motor Control Laboratory")
        self.geometry("1400x900")

        # Motor model
        self.params = MotorParameters()
        self.motor = DCMotorModel(self.params, 'shunt')

        # Simulation control
        self.simulation_running = False
        self.simulation_thread = None
        self.dt = 0.01  # Time step
        self.solver_type = 'rk45'  # or 'euler'

        # Data queue for thread-safe updates
        self.data_queue = queue.Queue()

        # Economic parameters
        self.electricity_cost = 0.12  # $/kWh
        self.runtime_hours = 0
        self.total_energy = 0

        # Create GUI
        self.create_widgets()

        # Bind resize event
        self.bind('<Configure>', self.on_window_resize)

        # Update timer
        self.after(50, self.update_plots)

    def create_widgets(self):
        """Create all GUI widgets"""

        # Create notebook (tabs)
        self.notebook = ttk.Notebook(self)
        self.notebook.pack(fill='both', expand=True, padx=5, pady=5)

        # Tab 1: Motor Control
        self.control_tab = ttk.Frame(self.notebook)
        self.notebook.add(self.control_tab, text='Motor Control')
        self.create_control_tab()

        # Tab 2: Multi-Physics Analysis
        self.physics_tab = ttk.Frame(self.notebook)
        self.notebook.add(self.physics_tab, text='Multi-Physics')
        self.create_physics_tab()

        # Tab 3: Loss Analysis
        self.loss_tab = ttk.Frame(self.notebook)
        self.notebook.add(self.loss_tab, text='Loss Analysis')
        self.create_loss_tab()

        # Tab 4: Economic Analysis
        self.economic_tab = ttk.Frame(self.notebook)
        self.notebook.add(self.economic_tab, text='Economic Analysis')
        self.create_economic_tab()

        # Tab 5: Problem Solvers
        self.solver_tab = ttk.Frame(self.notebook)
        self.notebook.add(self.solver_tab, text='Problem Solvers')
        self.create_solver_tab()

    def create_control_tab(self):
        """Create motor control interface"""

        # Left panel - Controls
        left_panel = ttk.Frame(self.control_tab)
        left_panel.pack(side='left', fill='y', padx=5, pady=5)

        # Title
        ttk.Label(left_panel, text="Motor Parameters", font=('Arial', 12, 'bold')).pack(pady=5)

        # Motor type selection
        ttk.Label(left_panel, text="Motor Type:").pack(anchor='w', padx=5)
        self.motor_type_var = tk.StringVar(value='shunt')
        motor_type_frame = ttk.Frame(left_panel)
        motor_type_frame.pack(fill='x', padx=5, pady=2)
        ttk.Radiobutton(motor_type_frame, text='Shunt', variable=self.motor_type_var,
                        value='shunt', command=self.change_motor_type).pack(side='left')
        ttk.Radiobutton(motor_type_frame, text='Series', variable=self.motor_type_var,
                        value='series', command=self.change_motor_type).pack(side='left')

        # Solver selection
        ttk.Label(left_panel, text="ODE Solver:").pack(anchor='w', padx=5, pady=(10,0))
        self.solver_var = tk.StringVar(value='rk45')
        solver_frame = ttk.Frame(left_panel)
        solver_frame.pack(fill='x', padx=5, pady=2)
        ttk.Radiobutton(solver_frame, text='RK45', variable=self.solver_var,
                        value='rk45').pack(side='left')
        ttk.Radiobutton(solver_frame, text='Euler', variable=self.solver_var,
                        value='euler').pack(side='left')

        # Control sliders
        self.sliders = {}

        slider_configs = [
            ('Supply Voltage (V)', 'V_supply', 0, 600, 440),
            ('External Resistance (Ω)', 'R_external', 0, 10, 0),
            ('Load Torque (N·m)', 'load_torque', 0, 100, 0),
            ('Armature Resistance (Ω)', 'Ra', 0.1, 5, 0.5),
            ('Field Resistance (Ω)', 'Rf', 50, 500, 220),
        ]

        for label, key, min_val, max_val, init_val in slider_configs:
            frame = ttk.LabelFrame(left_panel, text=label)
            frame.pack(fill='x', padx=5, pady=5)

            slider = tk.Scale(frame, from_=min_val, to=max_val, resolution=0.1,
                            orient='horizontal', command=lambda v, k=key: self.update_parameter(k, v))
            slider.set(init_val)
            slider.pack(fill='x', padx=5, pady=2)

            value_label = ttk.Label(frame, text=f"{init_val:.1f}")
            value_label.pack()

            self.sliders[key] = (slider, value_label)

        # Control buttons
        button_frame = ttk.Frame(left_panel)
        button_frame.pack(fill='x', padx=5, pady=10)

        ttk.Button(button_frame, text="START", command=self.start_simulation).pack(side='left', padx=2)
        ttk.Button(button_frame, text="STOP", command=self.stop_simulation).pack(side='left', padx=2)
        ttk.Button(button_frame, text="RESET", command=self.reset_simulation).pack(side='left', padx=2)

        # Status display
        status_frame = ttk.LabelFrame(left_panel, text="Motor Status")
        status_frame.pack(fill='both', expand=True, padx=5, pady=5)

        self.status_labels = {}
        status_vars = ['Speed (rpm)', 'Torque (N·m)', 'Current (A)',
                      'Power (kW)', 'Efficiency (%)', 'Temperature (°C)']

        for var in status_vars:
            frame = ttk.Frame(status_frame)
            frame.pack(fill='x', padx=5, pady=2)
            ttk.Label(frame, text=var + ":").pack(side='left')
            label = ttk.Label(frame, text="0.0", font=('Arial', 10, 'bold'))
            label.pack(side='right')
            self.status_labels[var] = label

        # Right panel - Plots
        right_panel = ttk.Frame(self.control_tab)
        right_panel.pack(side='right', fill='both', expand=True, padx=5, pady=5)

        # Create matplotlib figure
        self.fig_control = Figure(figsize=(10, 8), dpi=100)
        self.canvas_control = FigureCanvasTkAgg(self.fig_control, right_panel)
        self.canvas_control.get_tk_widget().pack(fill='both', expand=True)

        # Create subplots
        self.ax_speed = self.fig_control.add_subplot(3, 2, 1)
        self.ax_current = self.fig_control.add_subplot(3, 2, 2)
        self.ax_torque = self.fig_control.add_subplot(3, 2, 3)
        self.ax_power = self.fig_control.add_subplot(3, 2, 4)
        self.ax_efficiency = self.fig_control.add_subplot(3, 2, 5)
        self.ax_temp = self.fig_control.add_subplot(3, 2, 6)

        self.fig_control.tight_layout()

    def create_physics_tab(self):
        """Create multi-physics analysis tab"""

        # Info panel
        info_frame = ttk.LabelFrame(self.physics_tab, text="Multi-Physics Coupling")
        info_frame.pack(fill='x', padx=5, pady=5)

        info_text = """
        This simulation couples three physical domains:
        • Electromagnetic: Armature and field circuits with back EMF
        • Thermal: Temperature-dependent resistances and heat transfer
        • Mechanical: Shaft torque, bearing loads, and stress analysis
        """
        ttk.Label(info_frame, text=info_text, justify='left').pack(padx=10, pady=5)

        # Plots
        plot_frame = ttk.Frame(self.physics_tab)
        plot_frame.pack(fill='both', expand=True, padx=5, pady=5)

        self.fig_physics = Figure(figsize=(12, 8), dpi=100)
        self.canvas_physics = FigureCanvasTkAgg(self.fig_physics, plot_frame)
        self.canvas_physics.get_tk_widget().pack(fill='both', expand=True)

        # Subplots for physics
        self.ax_thermal = self.fig_physics.add_subplot(2, 2, 1)
        self.ax_mechanical = self.fig_physics.add_subplot(2, 2, 2)
        self.ax_electrical = self.fig_physics.add_subplot(2, 2, 3)
        self.ax_coupling = self.fig_physics.add_subplot(2, 2, 4)

        self.fig_physics.tight_layout()

    def create_loss_tab(self):
        """Create loss analysis tab"""

        # Loss breakdown display
        loss_frame = ttk.LabelFrame(self.loss_tab, text="Loss Breakdown (Real-time)")
        loss_frame.pack(fill='x', padx=5, pady=5)

        self.loss_labels = {}
        loss_types = ['Copper (Armature)', 'Copper (Field)', 'Iron Core',
                     'Mechanical Friction', 'Stray Load', 'Total']

        for loss_type in loss_types:
            frame = ttk.Frame(loss_frame)
            frame.pack(fill='x', padx=5, pady=2)
            ttk.Label(frame, text=loss_type + ":").pack(side='left')
            label = ttk.Label(frame, text="0.0 W", font=('Arial', 10))
            label.pack(side='right')
            self.loss_labels[loss_type] = label

        # Pie chart and bar chart
        plot_frame = ttk.Frame(self.loss_tab)
        plot_frame.pack(fill='both', expand=True, padx=5, pady=5)

        self.fig_loss = Figure(figsize=(12, 6), dpi=100)
        self.canvas_loss = FigureCanvasTkAgg(self.fig_loss, plot_frame)
        self.canvas_loss.get_tk_widget().pack(fill='both', expand=True)

        self.ax_loss_pie = self.fig_loss.add_subplot(1, 2, 1)
        self.ax_loss_bar = self.fig_loss.add_subplot(1, 2, 2)

        self.fig_loss.tight_layout()

    def create_economic_tab(self):
        """Create economic analysis tab"""

        # Parameters
        param_frame = ttk.LabelFrame(self.economic_tab, text="Economic Parameters")
        param_frame.pack(fill='x', padx=5, pady=5)

        ttk.Label(param_frame, text="Electricity Cost ($/kWh):").grid(row=0, column=0, padx=5, pady=5)
        self.cost_entry = ttk.Entry(param_frame)
        self.cost_entry.insert(0, "0.12")
        self.cost_entry.grid(row=0, column=1, padx=5, pady=5)

        # Statistics
        stats_frame = ttk.LabelFrame(self.economic_tab, text="Operating Statistics")
        stats_frame.pack(fill='x', padx=5, pady=5)

        self.economic_labels = {}
        economic_vars = ['Runtime (hours)', 'Energy Consumed (kWh)', 'Operating Cost ($)',
                        'Average Power (kW)', 'Peak Power (kW)', 'Average Efficiency (%)']

        for var in economic_vars:
            frame = ttk.Frame(stats_frame)
            frame.pack(fill='x', padx=5, pady=2)
            ttk.Label(frame, text=var + ":").pack(side='left')
            label = ttk.Label(frame, text="0.0", font=('Arial', 10, 'bold'))
            label.pack(side='right')
            self.economic_labels[var] = label

        # Charts
        plot_frame = ttk.Frame(self.economic_tab)
        plot_frame.pack(fill='both', expand=True, padx=5, pady=5)

        self.fig_economic = Figure(figsize=(12, 6), dpi=100)
        self.canvas_economic = FigureCanvasTkAgg(self.fig_economic, plot_frame)
        self.canvas_economic.get_tk_widget().pack(fill='both', expand=True)

        self.ax_power_consumption = self.fig_economic.add_subplot(1, 2, 1)
        self.ax_cost = self.fig_economic.add_subplot(1, 2, 2)

        self.fig_economic.tight_layout()

    def create_solver_tab(self):
        """Create problem solver tab"""

        # Problem 13
        problem13_frame = ttk.LabelFrame(self.solver_tab, text="Problem 13: Shunt Motor Speed Control")
        problem13_frame.pack(fill='x', padx=5, pady=5)

        ttk.Label(problem13_frame, text="""
        A 440-V shunt motor running at 1500 rpm takes 30 A armature current
        and delivers 15 hp. Load torque varies as speed². Find resistance
        to reduce speed to 1300 rpm and the new current.
        """, justify='left').pack(padx=10, pady=5)

        ttk.Button(problem13_frame, text="Solve Problem 13",
                  command=self.solve_problem_13).pack(pady=5)

        self.result13_text = tk.Text(problem13_frame, height=8, width=80)
        self.result13_text.pack(padx=10, pady=5)

        # Problem 14
        problem14_frame = ttk.LabelFrame(self.solver_tab, text="Problem 14: Series Motor Field Flux Reduction")
        problem14_frame.pack(fill='x', padx=5, pady=5)

        ttk.Label(problem14_frame, text="""
        A 460-V series motor (Ra = 0.4 Ω) takes 25 A at 1000 rpm.
        Field flux reduced by 5%. Load torque ∝ speed², same efficiency.
        Find new current and speed.
        """, justify='left').pack(padx=10, pady=5)

        ttk.Button(problem14_frame, text="Solve Problem 14",
                  command=self.solve_problem_14).pack(pady=5)

        self.result14_text = tk.Text(problem14_frame, height=8, width=80)
        self.result14_text.pack(padx=10, pady=5)

    def change_motor_type(self):
        """Change motor type"""
        motor_type = self.motor_type_var.get()
        self.motor = DCMotorModel(self.params, motor_type)
        self.reset_simulation()

    def update_parameter(self, key, value):
        """Update motor parameter from slider"""
        value = float(value)
        self.sliders[key][1].config(text=f"{value:.1f}")

        if key == 'V_supply':
            self.motor.V_applied = value
        elif key == 'R_external':
            self.motor.R_external = value
        elif key == 'load_torque':
            self.motor.load_torque = value
        elif key == 'Ra':
            self.params.Ra = value
            self.motor.params.Ra = value
        elif key == 'Rf':
            self.params.Rf = value
            self.motor.params.Rf = value

    def start_simulation(self):
        """Start simulation"""
        if not self.simulation_running:
            self.simulation_running = True
            self.solver_type = self.solver_var.get()
            self.simulation_thread = threading.Thread(target=self.simulation_loop, daemon=True)
            self.simulation_thread.start()

    def stop_simulation(self):
        """Stop simulation"""
        self.simulation_running = False

    def reset_simulation(self):
        """Reset simulation"""
        self.stop_simulation()
        time.sleep(0.1)

        self.motor.state = np.array([0.0, 0.0, 0.0, 0.0, self.params.ambient_temp])
        self.motor.time_history = []
        self.motor.state_history = []
        self.motor.loss_history = []

        self.runtime_hours = 0
        self.total_energy = 0

    def simulation_loop(self):
        """Main simulation loop (runs in separate thread)"""
        last_time = time.time()
        sim_time = 0

        while self.simulation_running:
            current_time = time.time()
            real_dt = current_time - last_time
            last_time = current_time

            # Update motor state
            if self.solver_type == 'rk45':
                self.motor.rk45_step(self.dt)
            else:
                self.motor.euler_step(self.dt)

            # Store history
            sim_time += self.dt
            self.motor.time_history.append(sim_time)
            self.motor.state_history.append(self.motor.state.copy())

            losses = self.motor.calculate_losses()
            self.motor.loss_history.append(losses)

            # Update economic data
            self.runtime_hours += self.dt / 3600
            Ia, If = self.motor.state[0], self.motor.state[1]
            if self.motor.motor_type == 'shunt':
                P_in = self.motor.V_applied * (Ia + If) / 1000  # kW
            else:
                P_in = self.motor.V_applied * Ia / 1000  # kW
            self.total_energy += P_in * self.dt / 3600  # kWh

            # Put data in queue for GUI update
            self.data_queue.put({
                'time': sim_time,
                'state': self.motor.state.copy(),
                'losses': losses
            })

            # Limit to real-time or faster
            time.sleep(max(0, self.dt - real_dt))

            # Keep only last 1000 points
            if len(self.motor.time_history) > 1000:
                self.motor.time_history = self.motor.time_history[-1000:]
                self.motor.state_history = self.motor.state_history[-1000:]
                self.motor.loss_history = self.motor.loss_history[-1000:]

    def update_plots(self):
        """Update all plots (runs in main thread)"""

        # Get data from queue
        while not self.data_queue.empty():
            try:
                data = self.data_queue.get_nowait()
            except queue.Empty:
                break

        if len(self.motor.time_history) == 0:
            self.after(50, self.update_plots)
            return

        # Get current state
        Ia, If, omega, theta, T_motor = self.motor.state
        speed_rpm = omega * 60 / (2 * np.pi)
        torque = self.motor.get_torque()
        power_kw = self.motor.get_power_output() / 1000
        efficiency = self.motor.get_efficiency()

        # Update status labels
        self.status_labels['Speed (rpm)'].config(text=f"{speed_rpm:.1f}")
        self.status_labels['Torque (N·m)'].config(text=f"{torque:.2f}")
        self.status_labels['Current (A)'].config(text=f"{Ia:.2f}")
        self.status_labels['Power (kW)'].config(text=f"{power_kw:.3f}")
        self.status_labels['Efficiency (%)'].config(text=f"{efficiency:.1f}")
        self.status_labels['Temperature (°C)'].config(text=f"{T_motor:.1f}")

        # Update control tab plots
        time_data = np.array(self.motor.time_history)
        state_data = np.array(self.motor.state_history)

        # Speed
        self.ax_speed.clear()
        self.ax_speed.plot(time_data, state_data[:, 2] * 60 / (2*np.pi), 'b-', linewidth=2)
        self.ax_speed.set_xlabel('Time (s)')
        self.ax_speed.set_ylabel('Speed (rpm)')
        self.ax_speed.set_title('Motor Speed')
        self.ax_speed.grid(True, alpha=0.3)

        # Current
        self.ax_current.clear()
        self.ax_current.plot(time_data, state_data[:, 0], 'r-', label='Armature', linewidth=2)
        self.ax_current.plot(time_data, state_data[:, 1], 'g-', label='Field', linewidth=2)
        self.ax_current.set_xlabel('Time (s)')
        self.ax_current.set_ylabel('Current (A)')
        self.ax_current.set_title('Currents')
        self.ax_current.legend()
        self.ax_current.grid(True, alpha=0.3)

        # Torque
        self.ax_torque.clear()
        torques = []
        for state in state_data:
            if self.motor.motor_type == 'shunt':
                T = self.motor.params.Kt * state[1] * state[0]
            else:
                T = self.motor.params.Kt * state[0] * state[0]
            torques.append(T)
        self.ax_torque.plot(time_data, torques, 'm-', linewidth=2)
        self.ax_torque.set_xlabel('Time (s)')
        self.ax_torque.set_ylabel('Torque (N·m)')
        self.ax_torque.set_title('Electromagnetic Torque')
        self.ax_torque.grid(True, alpha=0.3)

        # Power
        self.ax_power.clear()
        powers = np.array(torques) * state_data[:, 2] / 1000
        self.ax_power.plot(time_data, powers, 'c-', linewidth=2)
        self.ax_power.set_xlabel('Time (s)')
        self.ax_power.set_ylabel('Power (kW)')
        self.ax_power.set_title('Mechanical Power Output')
        self.ax_power.grid(True, alpha=0.3)

        # Efficiency
        self.ax_efficiency.clear()
        efficiencies = []
        for i, state in enumerate(state_data):
            if self.motor.motor_type == 'shunt':
                P_in = self.motor.V_applied * (state[0] + state[1])
            else:
                P_in = self.motor.V_applied * state[0]
            if P_in > 0:
                eff = (torques[i] * state[2] / P_in) * 100
            else:
                eff = 0
            efficiencies.append(min(eff, 100))
        self.ax_efficiency.plot(time_data, efficiencies, 'y-', linewidth=2)
        self.ax_efficiency.set_xlabel('Time (s)')
        self.ax_efficiency.set_ylabel('Efficiency (%)')
        self.ax_efficiency.set_title('Efficiency')
        self.ax_efficiency.set_ylim([0, 100])
        self.ax_efficiency.grid(True, alpha=0.3)

        # Temperature
        self.ax_temp.clear()
        self.ax_temp.plot(time_data, state_data[:, 4], 'orange', linewidth=2)
        self.ax_temp.axhline(y=self.params.max_temp, color='r', linestyle='--', label='Max Temp')
        self.ax_temp.set_xlabel('Time (s)')
        self.ax_temp.set_ylabel('Temperature (°C)')
        self.ax_temp.set_title('Motor Temperature')
        self.ax_temp.legend()
        self.ax_temp.grid(True, alpha=0.3)

        self.canvas_control.draw()

        # Update physics tab
        self.update_physics_plots()

        # Update loss tab
        self.update_loss_plots()

        # Update economic tab
        self.update_economic_plots()

        # Schedule next update
        self.after(50, self.update_plots)

    def update_physics_plots(self):
        """Update multi-physics plots"""
        if len(self.motor.time_history) == 0:
            return

        time_data = np.array(self.motor.time_history)
        state_data = np.array(self.motor.state_history)

        # Thermal plot
        self.ax_thermal.clear()
        self.ax_thermal.plot(time_data, state_data[:, 4], 'r-', linewidth=2, label='Motor Temp')
        self.ax_thermal.axhline(y=self.params.ambient_temp, color='b', linestyle='--', label='Ambient')
        self.ax_thermal.axhline(y=self.params.max_temp, color='orange', linestyle='--', label='Max Rating')
        self.ax_thermal.fill_between(time_data, self.params.ambient_temp, state_data[:, 4], alpha=0.3)
        self.ax_thermal.set_xlabel('Time (s)')
        self.ax_thermal.set_ylabel('Temperature (°C)')
        self.ax_thermal.set_title('Thermal Dynamics')
        self.ax_thermal.legend()
        self.ax_thermal.grid(True, alpha=0.3)

        # Mechanical plot
        self.ax_mechanical.clear()
        stress_data = []
        for state in state_data:
            if self.motor.motor_type == 'shunt':
                T = self.motor.params.Kt * state[1] * state[0]
            else:
                T = self.motor.params.Kt * state[0] * state[0]
            radius = self.params.shaft_diameter / 2
            J_shaft = np.pi * self.params.shaft_diameter**4 / 32
            stress = T * radius / J_shaft / 1e6  # MPa
            stress_data.append(stress)

        self.ax_mechanical.plot(time_data, stress_data, 'g-', linewidth=2)
        self.ax_mechanical.axhline(y=250, color='r', linestyle='--', label='Yield Strength')
        self.ax_mechanical.set_xlabel('Time (s)')
        self.ax_mechanical.set_ylabel('Shear Stress (MPa)')
        self.ax_mechanical.set_title('Shaft Mechanical Stress')
        self.ax_mechanical.legend()
        self.ax_mechanical.grid(True, alpha=0.3)

        # Electrical plot
        self.ax_electrical.clear()
        back_emf = []
        for state in state_data:
            if self.motor.motor_type == 'shunt':
                Eb = self.motor.params.Ke * state[1] * state[2]
            else:
                Eb = self.motor.params.Ke * state[0] * state[2]
            back_emf.append(Eb)

        self.ax_electrical.plot(time_data, back_emf, 'b-', linewidth=2, label='Back EMF')
        self.ax_electrical.axhline(y=self.motor.V_applied, color='r', linestyle='--', label='Supply Voltage')
        self.ax_electrical.set_xlabel('Time (s)')
        self.ax_electrical.set_ylabel('Voltage (V)')
        self.ax_electrical.set_title('Electrical Dynamics')
        self.ax_electrical.legend()
        self.ax_electrical.grid(True, alpha=0.3)

        # Coupling plot (temperature effect on resistance)
        self.ax_coupling.clear()
        Ra_temp = []
        for state in state_data:
            Ra = self.params.Ra * (1 + 0.00393 * (state[4] - self.params.ambient_temp))
            Ra_temp.append(Ra)

        self.ax_coupling.plot(time_data, Ra_temp, 'purple', linewidth=2)
        self.ax_coupling.axhline(y=self.params.Ra, color='b', linestyle='--', label='Cold Resistance')
        self.ax_coupling.set_xlabel('Time (s)')
        self.ax_coupling.set_ylabel('Resistance (Ω)')
        self.ax_coupling.set_title('Thermal-Electrical Coupling (Ra vs Temp)')
        self.ax_coupling.legend()
        self.ax_coupling.grid(True, alpha=0.3)

        self.fig_physics.tight_layout()
        self.canvas_physics.draw()

    def update_loss_plots(self):
        """Update loss analysis plots"""
        if len(self.motor.loss_history) == 0:
            return

        # Get latest losses
        losses = self.motor.loss_history[-1]

        # Update loss labels
        self.loss_labels['Copper (Armature)'].config(text=f"{losses['copper_armature']:.2f} W")
        self.loss_labels['Copper (Field)'].config(text=f"{losses['copper_field']:.2f} W")
        self.loss_labels['Iron Core'].config(text=f"{losses['iron']:.2f} W")
        self.loss_labels['Mechanical Friction'].config(text=f"{losses['mechanical']:.2f} W")
        self.loss_labels['Stray Load'].config(text=f"{losses['stray']:.2f} W")
        self.loss_labels['Total'].config(text=f"{losses['total']:.2f} W")

        # Pie chart
        self.ax_loss_pie.clear()
        loss_labels = ['Copper\n(Armature)', 'Copper\n(Field)', 'Iron', 'Mechanical', 'Stray']
        loss_values = [losses['copper_armature'], losses['copper_field'],
                      losses['iron'], losses['mechanical'], losses['stray']]

        colors = ['#ff9999', '#ff6666', '#66b3ff', '#99ff99', '#ffcc99']

        # Only plot if there are losses
        if sum(loss_values) > 0:
            self.ax_loss_pie.pie(loss_values, labels=loss_labels, autopct='%1.1f%%',
                                colors=colors, startangle=90)
            self.ax_loss_pie.set_title('Loss Distribution')

        # Bar chart - loss history
        self.ax_loss_bar.clear()
        if len(self.motor.loss_history) > 1:
            time_data = np.array(self.motor.time_history)

            copper_arm = [l['copper_armature'] for l in self.motor.loss_history]
            copper_field = [l['copper_field'] for l in self.motor.loss_history]
            iron = [l['iron'] for l in self.motor.loss_history]
            mechanical = [l['mechanical'] for l in self.motor.loss_history]
            stray = [l['stray'] for l in self.motor.loss_history]

            self.ax_loss_bar.plot(time_data, copper_arm, label='Cu (Arm)', color=colors[0], linewidth=2)
            self.ax_loss_bar.plot(time_data, copper_field, label='Cu (Field)', color=colors[1], linewidth=2)
            self.ax_loss_bar.plot(time_data, iron, label='Iron', color=colors[2], linewidth=2)
            self.ax_loss_bar.plot(time_data, mechanical, label='Mech', color=colors[3], linewidth=2)
            self.ax_loss_bar.plot(time_data, stray, label='Stray', color=colors[4], linewidth=2)

            self.ax_loss_bar.set_xlabel('Time (s)')
            self.ax_loss_bar.set_ylabel('Loss (W)')
            self.ax_loss_bar.set_title('Loss History')
            self.ax_loss_bar.legend()
            self.ax_loss_bar.grid(True, alpha=0.3)

        self.fig_loss.tight_layout()
        self.canvas_loss.draw()

    def update_economic_plots(self):
        """Update economic analysis plots"""
        if len(self.motor.time_history) == 0:
            return

        # Get electricity cost
        try:
            electricity_cost = float(self.cost_entry.get())
        except:
            electricity_cost = 0.12

        # Calculate statistics
        time_data = np.array(self.motor.time_history)
        state_data = np.array(self.motor.state_history)

        powers = []
        for state in state_data:
            if self.motor.motor_type == 'shunt':
                P_in = self.motor.V_applied * (state[0] + state[1]) / 1000  # kW
            else:
                P_in = self.motor.V_applied * state[0] / 1000  # kW
            powers.append(P_in)

        powers = np.array(powers)

        avg_power = np.mean(powers) if len(powers) > 0 else 0
        peak_power = np.max(powers) if len(powers) > 0 else 0

        # Calculate efficiency
        efficiencies = []
        for i, state in enumerate(state_data):
            if self.motor.motor_type == 'shunt':
                T = self.motor.params.Kt * state[1] * state[0]
            else:
                T = self.motor.params.Kt * state[0] * state[0]

            P_out = T * state[2]
            P_in = powers[i] * 1000

            if P_in > 0:
                eff = (P_out / P_in) * 100
            else:
                eff = 0
            efficiencies.append(min(eff, 100))

        avg_efficiency = np.mean(efficiencies) if len(efficiencies) > 0 else 0

        operating_cost = self.total_energy * electricity_cost

        # Update labels
        self.economic_labels['Runtime (hours)'].config(text=f"{self.runtime_hours:.4f}")
        self.economic_labels['Energy Consumed (kWh)'].config(text=f"{self.total_energy:.4f}")
        self.economic_labels['Operating Cost ($)'].config(text=f"{operating_cost:.4f}")
        self.economic_labels['Average Power (kW)'].config(text=f"{avg_power:.3f}")
        self.economic_labels['Peak Power (kW)'].config(text=f"{peak_power:.3f}")
        self.economic_labels['Average Efficiency (%)'].config(text=f"{avg_efficiency:.1f}")

        # Power consumption plot
        self.ax_power_consumption.clear()
        self.ax_power_consumption.plot(time_data, powers, 'b-', linewidth=2)
        self.ax_power_consumption.fill_between(time_data, 0, powers, alpha=0.3)
        self.ax_power_consumption.set_xlabel('Time (s)')
        self.ax_power_consumption.set_ylabel('Power (kW)')
        self.ax_power_consumption.set_title('Power Consumption')
        self.ax_power_consumption.grid(True, alpha=0.3)

        # Cost accumulation plot
        self.ax_cost.clear()
        energy_cumulative = np.cumsum(powers) * self.dt / 3600  # kWh
        cost_cumulative = energy_cumulative * electricity_cost

        self.ax_cost.plot(time_data, cost_cumulative, 'g-', linewidth=2)
        self.ax_cost.fill_between(time_data, 0, cost_cumulative, alpha=0.3, color='green')
        self.ax_cost.set_xlabel('Time (s)')
        self.ax_cost.set_ylabel('Cumulative Cost ($)')
        self.ax_cost.set_title('Operating Cost Accumulation')
        self.ax_cost.grid(True, alpha=0.3)

        self.fig_economic.tight_layout()
        self.canvas_economic.draw()

    def solve_problem_13(self):
        """Solve problem 13"""
        result = MotorProblemSolver.solve_problem_13()

        output = f"""
SOLUTION FOR PROBLEM 13:
========================

Given:
• Supply voltage: 440 V
• Initial speed: 1500 rpm
• Initial armature current: 30 A
• Output power: 15 hp (11,190 W)
• Load torque ∝ speed²
• Desired speed: 1300 rpm

Calculated:
• Armature resistance: {result['Ra']:.4f} Ω
• Initial back EMF: {440 - 30*result['Ra']:.2f} V
• New back EMF at 1300 rpm: {result['Eb2']:.2f} V

Results:
• Required series resistance: {result['R']:.2f} Ω
• New armature current: {result['Ia2']:.1f} A

Expected answers: 2.97 Ω, 22.5 A
"""

        self.result13_text.delete('1.0', tk.END)
        self.result13_text.insert('1.0', output)

    def solve_problem_14(self):
        """Solve problem 14"""
        result = MotorProblemSolver.solve_problem_14()

        output = f"""
SOLUTION FOR PROBLEM 14:
========================

Given:
• Supply voltage: 460 V
• Armature resistance: 0.4 Ω
• Initial current: 25 A
• Initial speed: 1000 rpm
• Flux reduction: 5%
• Load torque ∝ speed²
• Same efficiency

Calculated:
• Initial back EMF: {460 - 25*0.4:.2f} V

Results:
• New current: {result['I2']:.2f} A
• New speed: {result['N2']:.0f} rpm
• New back EMF: {result['Eb2']:.2f} V

The flux reduction causes the motor to draw different current
to maintain torque balance with the load characteristic.
"""

        self.result14_text.delete('1.0', tk.END)
        self.result14_text.insert('1.0', output)

    def on_window_resize(self, event):
        """Handle window resize for auto-scaling"""
        # Redraw all canvases
        try:
            self.canvas_control.draw()
            self.canvas_physics.draw()
            self.canvas_loss.draw()
            self.canvas_economic.draw()
        except:
            pass


# ====================== MAIN ======================

def main():
    """Main entry point"""

    # First, solve the problems in console
    print("\n" + "="*70)
    print(" DC MOTOR PROBLEMS - ANALYTICAL SOLUTIONS")
    print("="*70)

    MotorProblemSolver.solve_problem_13()
    MotorProblemSolver.solve_problem_14()

    print("\n" + "="*70)
    print(" LAUNCHING ADVANCED DC MOTOR LABORATORY GUI")
    print("="*70 + "\n")

    # Launch GUI
    app = AdvancedMotorLab()
    app.mainloop()


if __name__ == "__main__":
    main()
