#!/usr/bin/env python3
"""
Advanced Universal Motor AC/DC Analysis Laboratory
Multi-Physics Simulation with Electromagnetic-Thermal-Mechanical Coupling
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
from datetime import datetime

class UniversalMotorSimulator:
    """
    Advanced Universal Motor Multi-Physics Simulator
    Includes electromagnetic, thermal, and mechanical analysis
    """

    def __init__(self):
        # Default motor parameters from problem
        self.P_rated = 250  # W
        self.f = 50  # Hz
        self.V_rated = 220  # V
        self.N_dc = 2000  # rpm on DC
        self.I_dc = 1.0  # A on DC
        self.I_ac = 1.0  # A rms on AC
        self.Ra = 20  # Ohm - armature resistance
        self.La = 0.4  # H - armature inductance

        # Physical constants
        self.omega = 2 * np.pi * self.f
        self.rho_copper = 1.68e-8  # Ohm-m
        self.rho_iron = 7800  # kg/m³
        self.cp_copper = 385  # J/(kg·K)
        self.cp_iron = 450  # J/(kg·K)
        self.h_conv = 25  # W/(m²·K) - convection coefficient
        self.T_ambient = 25  # °C

        # Motor construction parameters
        self.l_conductor = 10  # m - total conductor length
        self.A_conductor = 1e-6  # m² - conductor cross-section
        self.mass_copper = 0.5  # kg
        self.mass_iron = 1.5  # kg
        self.A_surface = 0.05  # m² - cooling surface area
        self.J = 0.001  # kg·m² - moment of inertia
        self.B_friction = 0.0005  # N·m·s - friction coefficient

        # Calculate motor constants
        self.calculate_motor_constants()

        # Simulation state
        self.running = False
        self.time_data = []
        self.speed_data = []
        self.torque_data = []
        self.current_data = []
        self.voltage_data = []
        self.temp_data = []
        self.power_data = []
        self.efficiency_data = []

    def calculate_motor_constants(self):
        """Calculate motor constants from DC operation"""
        # DC operation analysis
        E_dc = self.V_rated - self.I_dc * self.Ra  # Back EMF on DC
        self.omega_dc = self.N_dc * 2 * np.pi / 60  # rad/s

        # Motor constant (E = K_e * omega)
        self.K_e = E_dc / self.omega_dc  # V/(rad/s)
        self.K_t = self.K_e  # N·m/A (for series motor)

        # DC torque
        self.T_dc = (E_dc * self.I_dc) / self.omega_dc

        # Iron loss coefficient (estimated)
        self.K_iron = 0.01  # W/(rad/s)²

        # Windage loss coefficient
        self.K_wind = 0.00001  # W/(rad/s)³

    def solve_ac_operation(self):
        """
        Solve AC operation parameters
        Returns: speed (rpm), torque (Nm), power_factor, temperature (°C)
        """
        # AC impedance
        X_L = self.omega * self.La
        Z = np.sqrt(self.Ra**2 + X_L**2)

        # Phase angle
        phi_z = np.arctan(X_L / self.Ra)

        # For AC operation with same current (1.0 A rms)
        V_drop = self.I_ac * Z

        # Back EMF (phasor)
        # V = I*Z + E, so E = V - I*Z
        # But this is complex, we need to consider the phase

        # Active power input
        P_input = self.V_rated * self.I_ac * np.cos(phi_z)

        # Copper loss
        P_copper = self.I_ac**2 * self.Ra

        # Estimated back EMF magnitude
        E_ac = np.sqrt(self.V_rated**2 + (self.I_ac * Z)**2 -
                       2 * self.V_rated * self.I_ac * Z * np.cos(phi_z))

        # For better approximation, use power balance
        # P_input = P_copper + P_iron + P_mech
        # E_ac * I_ac * cos(delta) ≈ P_mech

        # Simplified: assume E_ac ≈ V - I*Ra (resistive approximation)
        E_ac_approx = self.V_rated - self.I_ac * self.Ra

        # Speed from back EMF
        omega_ac = E_ac_approx / self.K_e
        N_ac = omega_ac * 60 / (2 * np.pi)

        # Torque (average)
        # For AC operation, torque pulsates. Average torque:
        T_ac = self.K_t * self.I_ac * (self.I_ac / np.sqrt(2))  # RMS consideration

        # Power factor
        # Approximation based on impedance angle
        pf = np.cos(phi_z)

        # Calculate losses and temperature
        P_iron = self.K_iron * omega_ac**2
        P_wind = self.K_wind * omega_ac**3
        P_friction = self.B_friction * omega_ac**2

        P_total_loss = P_copper + P_iron + P_wind + P_friction

        # Steady-state temperature rise
        delta_T = P_total_loss / (self.h_conv * self.A_surface)
        T_steady = self.T_ambient + delta_T

        results = {
            'speed_rpm': N_ac,
            'torque_nm': T_ac,
            'power_factor': pf,
            'temperature_c': T_steady,
            'P_input': P_input,
            'P_copper': P_copper,
            'P_iron': P_iron,
            'P_mech': P_input - P_total_loss,
            'efficiency': (P_input - P_total_loss) / P_input * 100 if P_input > 0 else 0,
            'Z': Z,
            'X_L': X_L,
            'phi_z': np.degrees(phi_z)
        }

        return results

    def motor_dynamics_dc(self, state, t, V_applied, T_load):
        """
        DC motor differential equations
        state = [omega, i, T_temp]
        """
        omega, i, T_temp = state

        # Electrical equation: V = Ra*i + La*di/dt + K_e*omega
        di_dt = (V_applied - self.Ra * i - self.K_e * omega) / self.La

        # Mechanical equation: J*domega/dt = T_motor - T_load - B*omega
        T_motor = self.K_t * i
        T_friction = self.B_friction * omega
        domega_dt = (T_motor - T_load - T_friction) / self.J

        # Thermal equation: m*c*dT/dt = P_loss - h*A*(T - T_amb)
        P_copper_loss = i**2 * self.Ra
        P_iron_loss = self.K_iron * omega**2
        P_mech_loss = self.B_friction * omega**2
        P_total_loss = P_copper_loss + P_iron_loss + P_mech_loss

        thermal_mass = self.mass_copper * self.cp_copper + self.mass_iron * self.cp_iron
        dT_dt = (P_total_loss - self.h_conv * self.A_surface *
                (T_temp - self.T_ambient)) / thermal_mass

        return [domega_dt, di_dt, dT_dt]

    def motor_dynamics_ac(self, t, state, V_rms, T_load):
        """
        AC motor differential equations with instantaneous voltage
        state = [omega, i, T_temp]
        """
        omega, i, T_temp = state

        # Instantaneous voltage
        V_inst = V_rms * np.sqrt(2) * np.sin(self.omega * t)

        # Electrical equation
        E_back = self.K_e * omega
        di_dt = (V_inst - self.Ra * i - E_back) / self.La

        # Mechanical equation (torque varies with current squared for series motor)
        # For AC, we use instantaneous current
        T_motor = self.K_t * i * abs(i)  # Series motor characteristic
        T_friction = self.B_friction * omega
        domega_dt = (T_motor - T_load - T_friction) / self.J

        # Thermal equation
        P_copper_loss = i**2 * self.Ra
        P_iron_loss = self.K_iron * omega**2
        P_mech_loss = self.B_friction * omega**2
        P_total_loss = P_copper_loss + P_iron_loss + P_mech_loss

        thermal_mass = self.mass_copper * self.cp_copper + self.mass_iron * self.cp_iron
        dT_dt = (P_total_loss - self.h_conv * self.A_surface *
                (T_temp - self.T_ambient)) / thermal_mass

        return [domega_dt, di_dt, dT_dt]

    def simulate_transient(self, t_span, V_applied, T_load, mode='DC', method='RK45'):
        """
        Simulate motor transient response
        mode: 'DC' or 'AC'
        method: 'RK45', 'Euler', 'RK23'
        """
        # Initial conditions: [omega, i, T_temp]
        y0 = [0, 0, self.T_ambient]

        if method == 'Euler':
            # Manual Euler method
            dt = 0.0001  # Time step
            t_eval = np.arange(t_span[0], t_span[1], dt)
            solution = {'t': t_eval, 'y': np.zeros((3, len(t_eval)))}
            solution['y'][:, 0] = y0

            for idx in range(1, len(t_eval)):
                t = t_eval[idx-1]
                state = solution['y'][:, idx-1]

                if mode == 'DC':
                    derivatives = self.motor_dynamics_dc(state, t, V_applied, T_load)
                else:
                    derivatives = self.motor_dynamics_ac(t, state, V_applied, T_load)

                solution['y'][:, idx] = state + np.array(derivatives) * dt
        else:
            # Use scipy ODE solvers
            t_eval = np.linspace(t_span[0], t_span[1], 1000)

            if mode == 'DC':
                solution = solve_ivp(
                    lambda t, y: self.motor_dynamics_dc(y, t, V_applied, T_load),
                    t_span, y0, method=method, t_eval=t_eval, max_step=0.001
                )
            else:
                solution = solve_ivp(
                    lambda t, y: self.motor_dynamics_ac(t, y, V_applied, T_load),
                    t_span, y0, method=method, t_eval=t_eval, max_step=0.0001
                )

        return solution

    def calculate_detailed_losses(self, omega, i, T_temp):
        """Calculate detailed loss breakdown"""
        # Copper losses (I²R)
        P_copper_armature = i**2 * self.Ra

        # Iron losses (hysteresis + eddy current)
        f_actual = omega / (2 * np.pi)
        P_hysteresis = 0.005 * (f_actual / self.f)**1.6 * omega  # Proportional to f^1.6
        P_eddy = 0.003 * (f_actual / self.f)**2 * omega  # Proportional to f²
        P_iron_total = P_hysteresis + P_eddy

        # Mechanical losses
        P_friction = self.B_friction * omega**2
        P_windage = self.K_wind * omega**3

        # Stray load losses (approximately 1% of rated power)
        P_stray = 0.01 * self.P_rated * (i / self.I_rated)**2 if hasattr(self, 'I_rated') else 0

        # Temperature-dependent resistance
        alpha_cu = 0.00393  # Temperature coefficient of copper
        Ra_temp = self.Ra * (1 + alpha_cu * (T_temp - 20))
        P_copper_corrected = i**2 * Ra_temp

        losses = {
            'P_copper': P_copper_corrected,
            'P_hysteresis': P_hysteresis,
            'P_eddy': P_eddy,
            'P_iron_total': P_iron_total,
            'P_friction': P_friction,
            'P_windage': P_windage,
            'P_stray': P_stray,
            'P_total': P_copper_corrected + P_iron_total + P_friction + P_windage + P_stray
        }

        return losses

    def mechanical_stress_analysis(self, T_motor, omega, T_transient):
        """Analyze mechanical stress on shaft and bearings"""
        # Shaft stress (torsional)
        # Assume shaft diameter
        d_shaft = 0.015  # m (15 mm)
        J_shaft = np.pi * d_shaft**4 / 32  # Polar moment of inertia

        # Torsional shear stress: tau = T*r/J
        tau_shaft = T_motor * (d_shaft/2) / J_shaft  # Pa

        # Bearing loads
        # Radial load from rotor weight (assumed)
        m_rotor = self.mass_iron + self.mass_copper
        F_radial = m_rotor * 9.81  # N

        # Transient torque causes additional load
        dT_dt = T_transient
        F_transient = abs(dT_dt) / (d_shaft/2)  # Approximate

        # Total bearing load
        F_bearing = np.sqrt(F_radial**2 + F_transient**2)

        # Centrifugal stress in rotor
        # Assume rotor outer radius
        r_rotor = 0.03  # m
        rho_rotor = self.mass_iron / (np.pi * r_rotor**2 * 0.05)  # Approximate density
        sigma_centrifugal = rho_rotor * omega**2 * r_rotor**2 / 2  # Pa

        stress = {
            'tau_shaft_MPa': tau_shaft / 1e6,
            'F_bearing_N': F_bearing,
            'sigma_centrifugal_MPa': sigma_centrifugal / 1e6,
            'safety_factor_shaft': 400 / (tau_shaft / 1e6) if tau_shaft > 0 else np.inf,  # Assuming 400 MPa yield
        }

        return stress


class UniversalMotorGUI:
    """Advanced GUI for Universal Motor Simulation"""

    def __init__(self, root):
        self.root = root
        self.root.title("Advanced Universal Motor AC/DC Analysis Laboratory")
        self.root.geometry("1400x900")

        # Create simulator instance
        self.simulator = UniversalMotorSimulator()

        # Simulation control
        self.simulation_running = False
        self.current_mode = 'DC'
        self.ode_method = 'RK45'

        # Setup GUI
        self.setup_gui()

        # Bind window resize
        self.root.bind('<Configure>', self.on_window_resize)

    def setup_gui(self):
        """Setup the complete GUI"""
        # Create main container with grid for auto-resizing
        self.main_container = ttk.PanedWindow(self.root, orient=tk.HORIZONTAL)
        self.main_container.pack(fill=tk.BOTH, expand=True)

        # Left panel for controls
        self.left_panel = ttk.Frame(self.main_container, width=400)
        self.main_container.add(self.left_panel, weight=1)

        # Right panel for visualization
        self.right_panel = ttk.Frame(self.main_container)
        self.main_container.add(self.right_panel, weight=3)

        # Setup left panel
        self.setup_control_panel()

        # Setup right panel with tabs
        self.setup_visualization_panel()

    def setup_control_panel(self):
        """Setup control panel with parameters and controls"""
        # Title
        title = ttk.Label(self.left_panel, text="Universal Motor Laboratory",
                         font=('Arial', 14, 'bold'))
        title.pack(pady=10)

        # Create notebook for organized controls
        control_notebook = ttk.Notebook(self.left_panel)
        control_notebook.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)

        # Tab 1: Motor Parameters
        params_frame = ttk.Frame(control_notebook)
        control_notebook.add(params_frame, text="Motor Parameters")
        self.setup_parameters_tab(params_frame)

        # Tab 2: Operation Mode
        operation_frame = ttk.Frame(control_notebook)
        control_notebook.add(operation_frame, text="Operation Mode")
        self.setup_operation_tab(operation_frame)

        # Tab 3: Control Settings
        control_frame = ttk.Frame(control_notebook)
        control_notebook.add(control_frame, text="Advanced Controls")
        self.setup_control_tab(control_frame)

        # Tab 4: Thermal Settings
        thermal_frame = ttk.Frame(control_notebook)
        control_notebook.add(thermal_frame, text="Thermal Analysis")
        self.setup_thermal_tab(thermal_frame)

        # Control Buttons at bottom
        self.setup_control_buttons()

    def setup_parameters_tab(self, parent):
        """Setup motor parameters input"""
        canvas = tk.Canvas(parent)
        scrollbar = ttk.Scrollbar(parent, orient="vertical", command=canvas.yview)
        scrollable_frame = ttk.Frame(canvas)

        scrollable_frame.bind(
            "<Configure>",
            lambda e: canvas.configure(scrollregion=canvas.bbox("all"))
        )

        canvas.create_window((0, 0), window=scrollable_frame, anchor="nw")
        canvas.configure(yscrollcommand=scrollbar.set)

        # Parameters dictionary for easy access
        self.param_vars = {}

        parameters = [
            ("Rated Power (W)", "P_rated", 250, 1, 10000),
            ("Frequency (Hz)", "f", 50, 25, 400),
            ("Rated Voltage (V)", "V_rated", 220, 50, 500),
            ("DC Speed (rpm)", "N_dc", 2000, 100, 20000),
            ("DC Current (A)", "I_dc", 1.0, 0.1, 20),
            ("AC Current RMS (A)", "I_ac", 1.0, 0.1, 20),
            ("Armature Resistance (Ω)", "Ra", 20, 0.1, 100),
            ("Armature Inductance (H)", "La", 0.4, 0.01, 5),
            ("Moment of Inertia (kg·m²)", "J", 0.001, 0.0001, 0.1),
            ("Friction Coefficient (N·m·s)", "B_friction", 0.0005, 0.00001, 0.01),
        ]

        for i, (label, var_name, default, min_val, max_val) in enumerate(parameters):
            frame = ttk.Frame(scrollable_frame)
            frame.pack(fill=tk.X, padx=5, pady=2)

            ttk.Label(frame, text=label, width=25).pack(side=tk.LEFT)

            var = tk.DoubleVar(value=default)
            self.param_vars[var_name] = var

            entry = ttk.Entry(frame, textvariable=var, width=10)
            entry.pack(side=tk.LEFT, padx=5)

            # Slider
            slider = ttk.Scale(frame, from_=min_val, to=max_val,
                             variable=var, orient=tk.HORIZONTAL)
            slider.pack(side=tk.LEFT, fill=tk.X, expand=True)

        canvas.pack(side="left", fill="both", expand=True)
        scrollbar.pack(side="right", fill="y")

        # Update button
        ttk.Button(parent, text="Update Parameters",
                  command=self.update_parameters).pack(pady=10)

    def setup_operation_tab(self, parent):
        """Setup operation mode selection"""
        ttk.Label(parent, text="Select Operation Mode",
                 font=('Arial', 12, 'bold')).pack(pady=10)

        self.mode_var = tk.StringVar(value='DC')

        modes_frame = ttk.LabelFrame(parent, text="Supply Type", padding=10)
        modes_frame.pack(fill=tk.X, padx=10, pady=5)

        ttk.Radiobutton(modes_frame, text="DC Supply", variable=self.mode_var,
                       value='DC').pack(anchor=tk.W)
        ttk.Radiobutton(modes_frame, text="AC Supply (RMS)", variable=self.mode_var,
                       value='AC').pack(anchor=tk.W)

        # Voltage control
        voltage_frame = ttk.LabelFrame(parent, text="Applied Voltage", padding=10)
        voltage_frame.pack(fill=tk.X, padx=10, pady=5)

        self.voltage_var = tk.DoubleVar(value=220)
        ttk.Label(voltage_frame, text="Voltage (V):").pack()
        ttk.Scale(voltage_frame, from_=0, to=500, variable=self.voltage_var,
                 orient=tk.HORIZONTAL).pack(fill=tk.X)
        ttk.Entry(voltage_frame, textvariable=self.voltage_var, width=10).pack()

        # Load torque control
        load_frame = ttk.LabelFrame(parent, text="Load Torque", padding=10)
        load_frame.pack(fill=tk.X, padx=10, pady=5)

        self.load_var = tk.DoubleVar(value=0.5)
        ttk.Label(load_frame, text="Torque (N·m):").pack()
        ttk.Scale(load_frame, from_=0, to=5, variable=self.load_var,
                 orient=tk.HORIZONTAL).pack(fill=tk.X)
        ttk.Entry(load_frame, textvariable=self.load_var, width=10).pack()

        # ODE Solver selection
        solver_frame = ttk.LabelFrame(parent, text="ODE Solver Method", padding=10)
        solver_frame.pack(fill=tk.X, padx=10, pady=5)

        self.solver_var = tk.StringVar(value='RK45')
        solvers = ['RK45', 'RK23', 'Euler', 'DOP853', 'BDF']

        for solver in solvers:
            ttk.Radiobutton(solver_frame, text=solver, variable=self.solver_var,
                           value=solver).pack(anchor=tk.W)

        # Quick calculation button
        ttk.Button(parent, text="Calculate AC Operation (Steady State)",
                  command=self.calculate_ac_steady_state,
                  style='Accent.TButton').pack(pady=10)

    def setup_control_tab(self, parent):
        """Setup advanced control options"""
        ttk.Label(parent, text="Advanced Control Features",
                 font=('Arial', 12, 'bold')).pack(pady=10)

        # Speed control
        speed_frame = ttk.LabelFrame(parent, text="Speed Control", padding=10)
        speed_frame.pack(fill=tk.X, padx=10, pady=5)

        self.speed_control_var = tk.BooleanVar(value=False)
        ttk.Checkbutton(speed_frame, text="Enable Speed Control",
                       variable=self.speed_control_var).pack(anchor=tk.W)

        self.target_speed_var = tk.DoubleVar(value=2000)
        ttk.Label(speed_frame, text="Target Speed (rpm):").pack()
        ttk.Scale(speed_frame, from_=0, to=5000, variable=self.target_speed_var,
                 orient=tk.HORIZONTAL).pack(fill=tk.X)

        # PWM control
        pwm_frame = ttk.LabelFrame(parent, text="PWM Control", padding=10)
        pwm_frame.pack(fill=tk.X, padx=10, pady=5)

        self.pwm_enabled_var = tk.BooleanVar(value=False)
        ttk.Checkbutton(pwm_frame, text="Enable PWM",
                       variable=self.pwm_enabled_var).pack(anchor=tk.W)

        self.duty_cycle_var = tk.DoubleVar(value=100)
        ttk.Label(pwm_frame, text="Duty Cycle (%):").pack()
        ttk.Scale(pwm_frame, from_=0, to=100, variable=self.duty_cycle_var,
                 orient=tk.HORIZONTAL).pack(fill=tk.X)

        # Soft start
        soft_frame = ttk.LabelFrame(parent, text="Soft Start", padding=10)
        soft_frame.pack(fill=tk.X, padx=10, pady=5)

        self.soft_start_var = tk.BooleanVar(value=False)
        ttk.Checkbutton(soft_frame, text="Enable Soft Start",
                       variable=self.soft_start_var).pack(anchor=tk.W)

        self.ramp_time_var = tk.DoubleVar(value=2.0)
        ttk.Label(soft_frame, text="Ramp Time (s):").pack()
        ttk.Entry(soft_frame, textvariable=self.ramp_time_var, width=10).pack()

    def setup_thermal_tab(self, parent):
        """Setup thermal analysis parameters"""
        ttk.Label(parent, text="Thermal & Derating Analysis",
                 font=('Arial', 12, 'bold')).pack(pady=10)

        # Ambient conditions
        ambient_frame = ttk.LabelFrame(parent, text="Ambient Conditions", padding=10)
        ambient_frame.pack(fill=tk.X, padx=10, pady=5)

        self.t_ambient_var = tk.DoubleVar(value=25)
        ttk.Label(ambient_frame, text="Ambient Temp (°C):").pack()
        ttk.Scale(ambient_frame, from_=-20, to=60, variable=self.t_ambient_var,
                 orient=tk.HORIZONTAL).pack(fill=tk.X)
        ttk.Entry(ambient_frame, textvariable=self.t_ambient_var, width=10).pack()

        # Cooling
        cooling_frame = ttk.LabelFrame(parent, text="Cooling System", padding=10)
        cooling_frame.pack(fill=tk.X, padx=10, pady=5)

        self.h_conv_var = tk.DoubleVar(value=25)
        ttk.Label(cooling_frame, text="Convection Coeff (W/m²·K):").pack()
        ttk.Scale(cooling_frame, from_=5, to=100, variable=self.h_conv_var,
                 orient=tk.HORIZONTAL).pack(fill=tk.X)

        self.forced_cooling_var = tk.BooleanVar(value=False)
        ttk.Checkbutton(cooling_frame, text="Forced Cooling (Fan)",
                       variable=self.forced_cooling_var,
                       command=self.toggle_forced_cooling).pack(anchor=tk.W)

        # Temperature limits
        limits_frame = ttk.LabelFrame(parent, text="Temperature Limits", padding=10)
        limits_frame.pack(fill=tk.X, padx=10, pady=5)

        self.t_max_var = tk.DoubleVar(value=120)
        ttk.Label(limits_frame, text="Max Temperature (°C):").pack()
        ttk.Entry(limits_frame, textvariable=self.t_max_var, width=10).pack()

        self.t_warning_var = tk.DoubleVar(value=100)
        ttk.Label(limits_frame, text="Warning Temperature (°C):").pack()
        ttk.Entry(limits_frame, textvariable=self.t_warning_var, width=10).pack()

        # Derating info
        ttk.Label(parent, text="Auto derating at high temperatures",
                 font=('Arial', 9, 'italic')).pack(pady=5)

    def setup_control_buttons(self):
        """Setup main control buttons"""
        button_frame = ttk.Frame(self.left_panel)
        button_frame.pack(side=tk.BOTTOM, fill=tk.X, padx=5, pady=10)

        # Start button
        self.start_button = ttk.Button(button_frame, text="▶ Start Simulation",
                                       command=self.start_simulation,
                                       style='Accent.TButton')
        self.start_button.pack(fill=tk.X, pady=2)

        # Stop button
        self.stop_button = ttk.Button(button_frame, text="⏸ Stop Simulation",
                                      command=self.stop_simulation,
                                      state=tk.DISABLED)
        self.stop_button.pack(fill=tk.X, pady=2)

        # Reset button
        ttk.Button(button_frame, text="↺ Reset",
                  command=self.reset_simulation).pack(fill=tk.X, pady=2)

        # Export button
        ttk.Button(button_frame, text="💾 Export Results",
                  command=self.export_results).pack(fill=tk.X, pady=2)

    def setup_visualization_panel(self):
        """Setup visualization panel with tabs"""
        self.viz_notebook = ttk.Notebook(self.right_panel)
        self.viz_notebook.pack(fill=tk.BOTH, expand=True)

        # Tab 1: Main Results
        self.results_tab = ttk.Frame(self.viz_notebook)
        self.viz_notebook.add(self.results_tab, text="📊 Dynamic Results")
        self.setup_results_tab()

        # Tab 2: Multi-Physics
        self.physics_tab = ttk.Frame(self.viz_notebook)
        self.viz_notebook.add(self.physics_tab, text="🔬 Multi-Physics")
        self.setup_physics_tab()

        # Tab 3: Loss Analysis
        self.loss_tab = ttk.Frame(self.viz_notebook)
        self.viz_notebook.add(self.loss_tab, text="📉 Loss Analysis")
        self.setup_loss_tab()

        # Tab 4: Economic Analysis
        self.economic_tab = ttk.Frame(self.viz_notebook)
        self.viz_notebook.add(self.economic_tab, text="💰 Economic Analysis")
        self.setup_economic_tab()

        # Tab 5: Steady State Results
        self.steady_tab = ttk.Frame(self.viz_notebook)
        self.viz_notebook.add(self.steady_tab, text="📋 Steady State")
        self.setup_steady_state_tab()

    def setup_results_tab(self):
        """Setup main results visualization"""
        # Create figure with subplots
        self.fig_results = Figure(figsize=(10, 8), dpi=100)

        self.ax_speed = self.fig_results.add_subplot(3, 2, 1)
        self.ax_current = self.fig_results.add_subplot(3, 2, 2)
        self.ax_torque = self.fig_results.add_subplot(3, 2, 3)
        self.ax_power = self.fig_results.add_subplot(3, 2, 4)
        self.ax_temp = self.fig_results.add_subplot(3, 2, 5)
        self.ax_efficiency = self.fig_results.add_subplot(3, 2, 6)

        self.fig_results.tight_layout(pad=3.0)

        # Create canvas
        self.canvas_results = FigureCanvasTkAgg(self.fig_results, self.results_tab)
        self.canvas_results.get_tk_widget().pack(fill=tk.BOTH, expand=True)

    def setup_physics_tab(self):
        """Setup multi-physics visualization"""
        self.fig_physics = Figure(figsize=(10, 8), dpi=100)

        self.ax_electromagnetic = self.fig_physics.add_subplot(2, 2, 1)
        self.ax_thermal = self.fig_physics.add_subplot(2, 2, 2)
        self.ax_mechanical = self.fig_physics.add_subplot(2, 2, 3)
        self.ax_coupling = self.fig_physics.add_subplot(2, 2, 4)

        self.fig_physics.tight_layout(pad=3.0)

        self.canvas_physics = FigureCanvasTkAgg(self.fig_physics, self.physics_tab)
        self.canvas_physics.get_tk_widget().pack(fill=tk.BOTH, expand=True)

    def setup_loss_tab(self):
        """Setup loss analysis visualization"""
        self.fig_loss = Figure(figsize=(10, 8), dpi=100)

        self.ax_loss_pie = self.fig_loss.add_subplot(2, 2, 1)
        self.ax_loss_breakdown = self.fig_loss.add_subplot(2, 2, 2)
        self.ax_loss_vs_speed = self.fig_loss.add_subplot(2, 2, 3)
        self.ax_efficiency_curve = self.fig_loss.add_subplot(2, 2, 4)

        self.fig_loss.tight_layout(pad=3.0)

        self.canvas_loss = FigureCanvasTkAgg(self.fig_loss, self.loss_tab)
        self.canvas_loss.get_tk_widget().pack(fill=tk.BOTH, expand=True)

    def setup_economic_tab(self):
        """Setup economic analysis"""
        # Create frame with scrollbar
        canvas = tk.Canvas(self.economic_tab)
        scrollbar = ttk.Scrollbar(self.economic_tab, orient="vertical", command=canvas.yview)
        scrollable_frame = ttk.Frame(canvas)

        scrollable_frame.bind(
            "<Configure>",
            lambda e: canvas.configure(scrollregion=canvas.bbox("all"))
        )

        canvas.create_window((0, 0), window=scrollable_frame, anchor="nw")
        canvas.configure(yscrollcommand=scrollbar.set)

        # Economic parameters
        ttk.Label(scrollable_frame, text="Economic Analysis Parameters",
                 font=('Arial', 14, 'bold')).pack(pady=10)

        param_frame = ttk.LabelFrame(scrollable_frame, text="Cost Parameters", padding=10)
        param_frame.pack(fill=tk.X, padx=10, pady=5)

        self.elec_cost_var = tk.DoubleVar(value=0.12)  # $/kWh
        self.operating_hours_var = tk.DoubleVar(value=4000)  # hours/year
        self.maintenance_cost_var = tk.DoubleVar(value=50)  # $/year
        self.motor_cost_var = tk.DoubleVar(value=500)  # $

        ttk.Label(param_frame, text="Electricity Cost ($/kWh):").grid(row=0, column=0, sticky=tk.W)
        ttk.Entry(param_frame, textvariable=self.elec_cost_var, width=15).grid(row=0, column=1)

        ttk.Label(param_frame, text="Operating Hours (h/year):").grid(row=1, column=0, sticky=tk.W)
        ttk.Entry(param_frame, textvariable=self.operating_hours_var, width=15).grid(row=1, column=1)

        ttk.Label(param_frame, text="Maintenance Cost ($/year):").grid(row=2, column=0, sticky=tk.W)
        ttk.Entry(param_frame, textvariable=self.maintenance_cost_var, width=15).grid(row=2, column=1)

        ttk.Label(param_frame, text="Motor Initial Cost ($):").grid(row=3, column=0, sticky=tk.W)
        ttk.Entry(param_frame, textvariable=self.motor_cost_var, width=15).grid(row=3, column=1)

        ttk.Button(scrollable_frame, text="Calculate Economic Analysis",
                  command=self.calculate_economics).pack(pady=10)

        # Results display
        self.economic_results_text = tk.Text(scrollable_frame, height=20, width=80)
        self.economic_results_text.pack(padx=10, pady=10, fill=tk.BOTH, expand=True)

        canvas.pack(side="left", fill="both", expand=True)
        scrollbar.pack(side="right", fill="y")

    def setup_steady_state_tab(self):
        """Setup steady state results display"""
        # Create text widget with scrollbar
        text_frame = ttk.Frame(self.steady_tab)
        text_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)

        scrollbar = ttk.Scrollbar(text_frame)
        scrollbar.pack(side=tk.RIGHT, fill=tk.Y)

        self.steady_state_text = tk.Text(text_frame, wrap=tk.WORD,
                                         yscrollcommand=scrollbar.set,
                                         font=('Courier', 10))
        self.steady_state_text.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        scrollbar.config(command=self.steady_state_text.yview)

        # Add initial problem statement
        self.display_problem_statement()

    def display_problem_statement(self):
        """Display the original problem statement"""
        problem_text = """
═══════════════════════════════════════════════════════════════════
        UNIVERSAL MOTOR AC/DC ANALYSIS - PROBLEM STATEMENT
═══════════════════════════════════════════════════════════════════

Given:
------
Motor Rating: 250 W, Single-Phase, 50 Hz, 220 V Universal Motor

DC Operation:
  • Supply Voltage: 220 V DC
  • Current: 1.0 A
  • Speed: 2000 rpm

AC Operation:
  • Supply Voltage: 220 V AC (RMS)
  • Current: 1.0 A (RMS)
  • Frequency: 50 Hz

Motor Parameters:
  • Armature Resistance (Ra): 20 Ω
  • Armature Inductance (La): 0.4 H

Required:
---------
For AC operation, calculate:
  1. Speed (rpm)
  2. Torque (N·m)
  3. Power Factor

Click "Calculate AC Operation (Steady State)" to solve.
═══════════════════════════════════════════════════════════════════
        """
        self.steady_state_text.insert('1.0', problem_text)
        self.steady_state_text.config(state=tk.DISABLED)

    def update_parameters(self):
        """Update simulator parameters from GUI"""
        try:
            for param, var in self.param_vars.items():
                setattr(self.simulator, param, var.get())

            # Update thermal parameters
            self.simulator.T_ambient = self.t_ambient_var.get()
            self.simulator.h_conv = self.h_conv_var.get()

            # Recalculate motor constants
            self.simulator.calculate_motor_constants()

            messagebox.showinfo("Success", "Parameters updated successfully!")
        except Exception as e:
            messagebox.showerror("Error", f"Error updating parameters: {str(e)}")

    def toggle_forced_cooling(self):
        """Toggle forced cooling"""
        if self.forced_cooling_var.get():
            self.h_conv_var.set(100)  # Higher convection with fan
        else:
            self.h_conv_var.set(25)  # Natural convection

    def calculate_ac_steady_state(self):
        """Calculate and display AC steady state operation"""
        self.update_parameters()

        results = self.simulator.solve_ac_operation()

        # Display results
        self.steady_state_text.config(state=tk.NORMAL)
        self.steady_state_text.delete('1.0', tk.END)

        result_text = f"""
═══════════════════════════════════════════════════════════════════
                    AC OPERATION STEADY STATE RESULTS
═══════════════════════════════════════════════════════════════════

OPERATING CONDITIONS:
─────────────────────
Supply Voltage (RMS):        {self.simulator.V_rated:.2f} V
Supply Frequency:            {self.simulator.f:.2f} Hz
Current (RMS):               {self.simulator.I_ac:.2f} A

MOTOR PARAMETERS:
─────────────────
Armature Resistance (Ra):   {self.simulator.Ra:.2f} Ω
Armature Inductance (La):   {self.simulator.La:.4f} H
Reactance (XL = ωLa):        {results['X_L']:.2f} Ω
Impedance (Z):               {results['Z']:.2f} Ω
Impedance Angle:             {results['phi_z']:.2f}°

PERFORMANCE RESULTS:
────────────────────
✓ Speed:                     {results['speed_rpm']:.2f} rpm
✓ Torque:                    {results['torque_nm']:.4f} N·m
✓ Power Factor:              {results['power_factor']:.4f}

POWER ANALYSIS:
───────────────
Input Power:                 {results['P_input']:.2f} W
Copper Loss:                 {results['P_copper']:.2f} W
Iron Loss:                   {results['P_iron']:.2f} W
Mechanical Output:           {results['P_mech']:.2f} W
Efficiency:                  {results['efficiency']:.2f}%

THERMAL ANALYSIS:
─────────────────
Steady-State Temperature:    {results['temperature_c']:.2f}°C
Ambient Temperature:         {self.simulator.T_ambient:.2f}°C
Temperature Rise:            {results['temperature_c'] - self.simulator.T_ambient:.2f}°C

COMPARISON WITH DC OPERATION:
─────────────────────────────
DC Speed:                    {self.simulator.N_dc:.2f} rpm
AC Speed:                    {results['speed_rpm']:.2f} rpm
Speed Reduction:             {(1 - results['speed_rpm']/self.simulator.N_dc)*100:.2f}%

DC Torque:                   {self.simulator.T_dc:.4f} N·m
AC Torque:                   {results['torque_nm']:.4f} N·m

NOTES:
──────
• AC operation results in lower speed due to reactive voltage drop
• Torque pulsates at twice the supply frequency in AC operation
• Power factor is lagging due to inductance
• Higher losses in AC mode due to iron losses and reactive power

═══════════════════════════════════════════════════════════════════
                        Analysis completed at {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}
═══════════════════════════════════════════════════════════════════
        """

        self.steady_state_text.insert('1.0', result_text)
        self.steady_state_text.config(state=tk.DISABLED)

        messagebox.showinfo("Calculation Complete",
                          f"AC Operation Results:\n\n"
                          f"Speed: {results['speed_rpm']:.2f} rpm\n"
                          f"Torque: {results['torque_nm']:.4f} N·m\n"
                          f"Power Factor: {results['power_factor']:.4f}")

    def start_simulation(self):
        """Start dynamic simulation"""
        self.simulation_running = True
        self.start_button.config(state=tk.DISABLED)
        self.stop_button.config(state=tk.NORMAL)

        # Update parameters
        self.update_parameters()

        # Run simulation in separate thread
        sim_thread = threading.Thread(target=self.run_simulation)
        sim_thread.daemon = True
        sim_thread.start()

    def run_simulation(self):
        """Run the dynamic simulation"""
        try:
            # Get parameters
            V_applied = self.voltage_var.get()
            T_load = self.load_var.get()
            mode = self.mode_var.get()
            method = self.solver_var.get()

            # Apply PWM if enabled
            if self.pwm_enabled_var.get():
                V_applied = V_applied * self.duty_cycle_var.get() / 100

            # Simulate
            t_span = [0, 2.0]  # 2 seconds simulation

            solution = self.simulator.simulate_transient(t_span, V_applied, T_load, mode, method)

            # Extract results
            t = solution['t']
            omega = solution['y'][0, :]
            i = solution['y'][1, :]
            T_temp = solution['y'][2, :]

            # Convert to rpm
            N = omega * 60 / (2 * np.pi)

            # Calculate torque
            T = self.simulator.K_t * i * np.abs(i) if mode == 'AC' else self.simulator.K_t * i

            # Calculate power
            P_mech = T * omega
            P_input = V_applied * i
            efficiency = np.where(P_input > 0, P_mech / P_input * 100, 0)

            # Update plots
            self.root.after(0, self.update_plots, t, N, i, T, P_mech, T_temp, efficiency)

        except Exception as e:
            self.root.after(0, messagebox.showerror, "Simulation Error", str(e))
        finally:
            self.simulation_running = False
            self.root.after(0, self.stop_button.config, {'state': tk.DISABLED})
            self.root.after(0, self.start_button.config, {'state': tk.NORMAL})

    def update_plots(self, t, N, i, T, P, T_temp, efficiency):
        """Update all plots with simulation results"""
        # Clear all axes
        for ax in [self.ax_speed, self.ax_current, self.ax_torque,
                   self.ax_power, self.ax_temp, self.ax_efficiency]:
            ax.clear()

        # Speed plot
        self.ax_speed.plot(t, N, 'b-', linewidth=2)
        self.ax_speed.set_xlabel('Time (s)')
        self.ax_speed.set_ylabel('Speed (rpm)')
        self.ax_speed.set_title('Motor Speed')
        self.ax_speed.grid(True, alpha=0.3)

        # Current plot
        self.ax_current.plot(t, i, 'r-', linewidth=2)
        self.ax_current.set_xlabel('Time (s)')
        self.ax_current.set_ylabel('Current (A)')
        self.ax_current.set_title('Armature Current')
        self.ax_current.grid(True, alpha=0.3)

        # Torque plot
        self.ax_torque.plot(t, T, 'g-', linewidth=2)
        self.ax_torque.set_xlabel('Time (s)')
        self.ax_torque.set_ylabel('Torque (N·m)')
        self.ax_torque.set_title('Motor Torque')
        self.ax_torque.grid(True, alpha=0.3)

        # Power plot
        self.ax_power.plot(t, P, 'm-', linewidth=2)
        self.ax_power.set_xlabel('Time (s)')
        self.ax_power.set_ylabel('Power (W)')
        self.ax_power.set_title('Mechanical Power')
        self.ax_power.grid(True, alpha=0.3)

        # Temperature plot
        self.ax_temp.plot(t, T_temp, 'orange', linewidth=2)
        self.ax_temp.axhline(y=self.t_warning_var.get(), color='yellow',
                            linestyle='--', label='Warning')
        self.ax_temp.axhline(y=self.t_max_var.get(), color='red',
                            linestyle='--', label='Max')
        self.ax_temp.set_xlabel('Time (s)')
        self.ax_temp.set_ylabel('Temperature (°C)')
        self.ax_temp.set_title('Motor Temperature')
        self.ax_temp.legend()
        self.ax_temp.grid(True, alpha=0.3)

        # Efficiency plot
        self.ax_efficiency.plot(t, efficiency, 'cyan', linewidth=2)
        self.ax_efficiency.set_xlabel('Time (s)')
        self.ax_efficiency.set_ylabel('Efficiency (%)')
        self.ax_efficiency.set_title('Motor Efficiency')
        self.ax_efficiency.set_ylim([0, 100])
        self.ax_efficiency.grid(True, alpha=0.3)

        self.fig_results.tight_layout()
        self.canvas_results.draw()

        # Update multi-physics plots
        self.update_physics_plots(t, N, i, T, T_temp)

        # Update loss plots
        self.update_loss_plots(N[-1] * 2*np.pi/60, i[-1], T_temp[-1])

    def update_physics_plots(self, t, N, i, T, T_temp):
        """Update multi-physics plots"""
        for ax in [self.ax_electromagnetic, self.ax_thermal,
                   self.ax_mechanical, self.ax_coupling]:
            ax.clear()

        omega = N * 2 * np.pi / 60

        # Electromagnetic: Back EMF
        E_back = self.simulator.K_e * omega
        self.ax_electromagnetic.plot(t, E_back, 'b-', linewidth=2, label='Back EMF')
        V_applied = self.voltage_var.get()
        self.ax_electromagnetic.axhline(y=V_applied, color='r', linestyle='--', label='Applied V')
        self.ax_electromagnetic.set_xlabel('Time (s)')
        self.ax_electromagnetic.set_ylabel('Voltage (V)')
        self.ax_electromagnetic.set_title('Electromagnetic: Back EMF')
        self.ax_electromagnetic.legend()
        self.ax_electromagnetic.grid(True, alpha=0.3)

        # Thermal: Heat generation and dissipation
        P_loss = i**2 * self.simulator.Ra + self.simulator.K_iron * omega**2
        P_dissipated = self.simulator.h_conv * self.simulator.A_surface * (T_temp - self.simulator.T_ambient)
        self.ax_thermal.plot(t, P_loss, 'r-', linewidth=2, label='Heat Generated')
        self.ax_thermal.plot(t, P_dissipated, 'b-', linewidth=2, label='Heat Dissipated')
        self.ax_thermal.set_xlabel('Time (s)')
        self.ax_thermal.set_ylabel('Power (W)')
        self.ax_thermal.set_title('Thermal: Heat Balance')
        self.ax_thermal.legend()
        self.ax_thermal.grid(True, alpha=0.3)

        # Mechanical: Speed vs Torque
        self.ax_mechanical.plot(N, T, 'g-', linewidth=2)
        self.ax_mechanical.set_xlabel('Speed (rpm)')
        self.ax_mechanical.set_ylabel('Torque (N·m)')
        self.ax_mechanical.set_title('Mechanical: Speed-Torque Curve')
        self.ax_mechanical.grid(True, alpha=0.3)

        # Coupling: Temperature effect on resistance
        alpha = 0.00393
        Ra_temp = self.simulator.Ra * (1 + alpha * (T_temp - 20))
        P_copper = i**2 * Ra_temp
        self.ax_coupling.plot(T_temp, P_copper, 'purple', linewidth=2)
        self.ax_coupling.set_xlabel('Temperature (°C)')
        self.ax_coupling.set_ylabel('Copper Loss (W)')
        self.ax_coupling.set_title('Thermal-Electrical Coupling')
        self.ax_coupling.grid(True, alpha=0.3)

        self.fig_physics.tight_layout()
        self.canvas_physics.draw()

    def update_loss_plots(self, omega, i, T_temp):
        """Update loss analysis plots"""
        for ax in [self.ax_loss_pie, self.ax_loss_breakdown,
                   self.ax_loss_vs_speed, self.ax_efficiency_curve]:
            ax.clear()

        # Calculate losses
        losses = self.simulator.calculate_detailed_losses(omega, i, T_temp)

        # Pie chart of losses
        loss_labels = ['Copper', 'Iron', 'Friction', 'Windage', 'Stray']
        loss_values = [losses['P_copper'], losses['P_iron_total'],
                      losses['P_friction'], losses['P_windage'], losses['P_stray']]

        colors = ['#ff6b6b', '#4ecdc4', '#45b7d1', '#96ceb4', '#ffeaa7']
        self.ax_loss_pie.pie(loss_values, labels=loss_labels, autopct='%1.1f%%',
                            colors=colors, startangle=90)
        self.ax_loss_pie.set_title('Loss Distribution')

        # Bar chart of loss breakdown
        self.ax_loss_breakdown.bar(loss_labels, loss_values, color=colors)
        self.ax_loss_breakdown.set_ylabel('Power Loss (W)')
        self.ax_loss_breakdown.set_title('Detailed Loss Breakdown')
        self.ax_loss_breakdown.tick_params(axis='x', rotation=45)
        self.ax_loss_breakdown.grid(True, alpha=0.3, axis='y')

        # Loss vs speed curve
        speed_range = np.linspace(100, 5000, 50) * 2*np.pi/60  # rad/s
        total_loss = []
        for w in speed_range:
            loss = self.simulator.calculate_detailed_losses(w, i, T_temp)
            total_loss.append(loss['P_total'])

        self.ax_loss_vs_speed.plot(speed_range * 60/(2*np.pi), total_loss, 'b-', linewidth=2)
        self.ax_loss_vs_speed.set_xlabel('Speed (rpm)')
        self.ax_loss_vs_speed.set_ylabel('Total Loss (W)')
        self.ax_loss_vs_speed.set_title('Loss vs Speed')
        self.ax_loss_vs_speed.grid(True, alpha=0.3)

        # Efficiency curve
        P_input = self.voltage_var.get() * i
        P_out = P_input - np.array(total_loss)
        efficiency = np.where(P_input > 0, P_out / P_input * 100, 0)

        self.ax_efficiency_curve.plot(speed_range * 60/(2*np.pi), efficiency, 'g-', linewidth=2)
        self.ax_efficiency_curve.set_xlabel('Speed (rpm)')
        self.ax_efficiency_curve.set_ylabel('Efficiency (%)')
        self.ax_efficiency_curve.set_title('Efficiency vs Speed')
        self.ax_efficiency_curve.set_ylim([0, 100])
        self.ax_efficiency_curve.grid(True, alpha=0.3)

        self.fig_loss.tight_layout()
        self.canvas_loss.draw()

    def calculate_economics(self):
        """Calculate and display economic analysis"""
        try:
            # Get parameters
            elec_cost = self.elec_cost_var.get()  # $/kWh
            hours_per_year = self.operating_hours_var.get()
            maint_cost = self.maintenance_cost_var.get()
            motor_cost = self.motor_cost_var.get()

            # Get motor performance
            results = self.simulator.solve_ac_operation()

            # Calculate annual energy consumption
            P_avg_kW = results['P_input'] / 1000
            annual_energy_kWh = P_avg_kW * hours_per_year
            annual_energy_cost = annual_energy_kWh * elec_cost

            # Total annual cost
            annual_operating_cost = annual_energy_cost + maint_cost

            # Calculate for different load scenarios
            load_factors = [0.25, 0.50, 0.75, 1.0]

            economic_text = f"""
═══════════════════════════════════════════════════════════════════
                        ECONOMIC ANALYSIS REPORT
═══════════════════════════════════════════════════════════════════

COST PARAMETERS:
────────────────
Electricity Rate:            ${elec_cost:.4f} /kWh
Operating Hours:             {hours_per_year:.0f} hours/year
Maintenance Cost:            ${maint_cost:.2f} /year
Motor Initial Cost:          ${motor_cost:.2f}

MOTOR PERFORMANCE:
──────────────────
Average Input Power:         {P_avg_kW:.3f} kW
Power Factor:                {results['power_factor']:.3f}
Efficiency:                  {results['efficiency']:.2f}%

ANNUAL ENERGY ANALYSIS:
───────────────────────
Energy Consumption:          {annual_energy_kWh:.2f} kWh/year
Energy Cost:                 ${annual_energy_cost:.2f} /year
Maintenance Cost:            ${maint_cost:.2f} /year
──────────────────────────────────────────────────────────────────
Total Operating Cost:        ${annual_operating_cost:.2f} /year

LIFECYCLE COST ANALYSIS (10 years):
────────────────────────────────────
Initial Investment:          ${motor_cost:.2f}
10-Year Operating Cost:      ${annual_operating_cost * 10:.2f}
10-Year Maintenance:         ${maint_cost * 10:.2f}
──────────────────────────────────────────────────────────────────
Total 10-Year Cost:          ${motor_cost + annual_operating_cost * 10:.2f}

LOAD FACTOR ANALYSIS:
─────────────────────
Load Factor    Power(kW)    Energy(kWh/yr)    Cost($/yr)
──────────────────────────────────────────────────────────────────
"""

            for lf in load_factors:
                P_lf = P_avg_kW * lf
                E_lf = P_lf * hours_per_year
                C_lf = E_lf * elec_cost + maint_cost
                economic_text += f"{lf*100:5.0f}%         {P_lf:6.3f}       {E_lf:8.2f}        ${C_lf:8.2f}\n"

            # ROI analysis for efficiency improvement
            efficiency_baseline = results['efficiency']
            efficiency_improved = min(efficiency_baseline + 5, 95)  # 5% improvement

            P_input_improved = results['P_mech'] / (efficiency_improved/100)
            energy_saved_kWh = (results['P_input'] - P_input_improved) * hours_per_year / 1000
            cost_saved = energy_saved_kWh * elec_cost

            economic_text += f"""
EFFICIENCY IMPROVEMENT ANALYSIS:
─────────────────────────────────
Current Efficiency:          {efficiency_baseline:.2f}%
Improved Efficiency:         {efficiency_improved:.2f}%
Annual Energy Saved:         {energy_saved_kWh:.2f} kWh
Annual Cost Saved:           ${cost_saved:.2f}
Payback Period:              {motor_cost * 0.2 / cost_saved:.2f} years
                            (assuming 20% higher cost for efficient motor)

POWER QUALITY IMPACT:
─────────────────────
Power Factor:                {results['power_factor']:.3f}
Reactive Power:              {P_avg_kW * np.tan(np.arccos(results['power_factor'])):.3f} kVAR
Power Factor Penalty:        {annual_energy_cost * 0.05 if results['power_factor'] < 0.85 else 0:.2f} $/year
                            (assumed 5% penalty for PF < 0.85)

ENVIRONMENTAL IMPACT:
─────────────────────
CO2 Emissions:               {annual_energy_kWh * 0.5:.2f} kg/year
                            (assuming 0.5 kg CO2/kWh)
Equivalent Trees:            {annual_energy_kWh * 0.5 / 20:.0f} trees needed for offset

RECOMMENDATIONS:
────────────────
• {'Consider power factor correction' if results['power_factor'] < 0.9 else 'Power factor is acceptable'}
• {'Motor is operating efficiently' if results['efficiency'] > 80 else 'Consider motor upgrade for better efficiency'}
• Annual operating cost is {annual_operating_cost/motor_cost*100:.1f}% of initial investment
• Energy cost dominates operating cost ({annual_energy_cost/annual_operating_cost*100:.1f}% of total)

═══════════════════════════════════════════════════════════════════
                Report generated at {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}
═══════════════════════════════════════════════════════════════════
            """

            self.economic_results_text.delete('1.0', tk.END)
            self.economic_results_text.insert('1.0', economic_text)

            messagebox.showinfo("Economic Analysis Complete",
                              f"Annual Operating Cost: ${annual_operating_cost:.2f}\n"
                              f"10-Year Total Cost: ${motor_cost + annual_operating_cost * 10:.2f}")

        except Exception as e:
            messagebox.showerror("Error", f"Error in economic analysis: {str(e)}")

    def stop_simulation(self):
        """Stop the simulation"""
        self.simulation_running = False
        self.stop_button.config(state=tk.DISABLED)
        self.start_button.config(state=tk.NORMAL)

    def reset_simulation(self):
        """Reset all simulation data"""
        # Clear all plots
        for fig in [self.fig_results, self.fig_physics, self.fig_loss]:
            fig.clear()

        # Recreate axes
        self.ax_speed = self.fig_results.add_subplot(3, 2, 1)
        self.ax_current = self.fig_results.add_subplot(3, 2, 2)
        self.ax_torque = self.fig_results.add_subplot(3, 2, 3)
        self.ax_power = self.fig_results.add_subplot(3, 2, 4)
        self.ax_temp = self.fig_results.add_subplot(3, 2, 5)
        self.ax_efficiency = self.fig_results.add_subplot(3, 2, 6)

        self.canvas_results.draw()

        messagebox.showinfo("Reset", "Simulation reset successfully")

    def export_results(self):
        """Export results to file"""
        try:
            timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
            filename = f'universal_motor_results_{timestamp}.txt'

            with open(filename, 'w') as f:
                f.write("Universal Motor AC/DC Analysis Results\n")
                f.write(f"Generated: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n\n")

                # Motor parameters
                f.write("Motor Parameters:\n")
                f.write(f"Rated Power: {self.simulator.P_rated} W\n")
                f.write(f"Frequency: {self.simulator.f} Hz\n")
                f.write(f"Voltage: {self.simulator.V_rated} V\n")
                f.write(f"Armature Resistance: {self.simulator.Ra} Ω\n")
                f.write(f"Armature Inductance: {self.simulator.La} H\n\n")

                # AC operation results
                results = self.simulator.solve_ac_operation()
                f.write("AC Operation Results:\n")
                f.write(f"Speed: {results['speed_rpm']:.2f} rpm\n")
                f.write(f"Torque: {results['torque_nm']:.4f} N·m\n")
                f.write(f"Power Factor: {results['power_factor']:.4f}\n")
                f.write(f"Efficiency: {results['efficiency']:.2f}%\n")
                f.write(f"Temperature: {results['temperature_c']:.2f}°C\n")

            messagebox.showinfo("Export Complete", f"Results exported to {filename}")
        except Exception as e:
            messagebox.showerror("Error", f"Error exporting results: {str(e)}")

    def on_window_resize(self, event):
        """Handle window resize for auto-scaling"""
        # Redraw canvases to fit new size
        try:
            self.canvas_results.draw()
            self.canvas_physics.draw()
            self.canvas_loss.draw()
        except:
            pass


def main():
    """Main function to run the application"""
    root = tk.Tk()

    # Set theme
    style = ttk.Style()
    style.theme_use('clam')

    # Custom styles
    style.configure('Accent.TButton', background='#4CAF50', foreground='white',
                   font=('Arial', 10, 'bold'))

    app = UniversalMotorGUI(root)

    # Center window on screen
    root.update_idletasks()
    width = root.winfo_width()
    height = root.winfo_height()
    x = (root.winfo_screenwidth() // 2) - (width // 2)
    y = (root.winfo_screenheight() // 2) - (height // 2)
    root.geometry(f'{width}x{height}+{x}+{y}')

    root.mainloop()


if __name__ == "__main__":
    main()
