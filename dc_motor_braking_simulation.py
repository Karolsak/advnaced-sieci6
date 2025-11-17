"""
Advanced DC Motor Braking Simulation with Multi-Physics Modeling
Example 30.47 Solution + Comprehensive Tkinter GUI Application
"""

import tkinter as tk
from tkinter import ttk, messagebox
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure
from scipy.integrate import solve_ivp, odeint
import math
from datetime import datetime

# ==================== MATHEMATICAL MODELS ====================

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


# ==================== GUI APPLICATION ====================

class DCMotorSimulationGUI:
    """Advanced DC Motor Simulation GUI with Multi-Physics"""

    def __init__(self, root):
        self.root = root
        self.root.title("Advanced DC Motor Braking Simulation - Multi-Physics Analysis")
        self.root.geometry("1400x900")

        # Initialize motor model
        self.motor = DCMotorModel()
        self.control = ControlSystem(self.motor)

        # Simulation state
        self.simulation_running = False
        self.simulation_data = None
        self.current_time = 0
        self.solver_type = 'RK45'

        # Create GUI
        self.create_widgets()

        # Bind resize event
        self.root.bind('<Configure>', self.on_resize)

        # Solve Example 30.47 on startup
        self.solve_example_30_47()

    def create_widgets(self):
        """Create all GUI widgets"""
        # Main container with notebook (tabs)
        self.notebook = ttk.Notebook(self.root)
        self.notebook.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)

        # Create tabs
        self.tab_main = ttk.Frame(self.notebook)
        self.tab_simulation = ttk.Frame(self.notebook)
        self.tab_thermal = ttk.Frame(self.notebook)
        self.tab_mechanical = ttk.Frame(self.notebook)
        self.tab_economic = ttk.Frame(self.notebook)
        self.tab_results = ttk.Frame(self.notebook)

        self.notebook.add(self.tab_main, text="Main Menu")
        self.notebook.add(self.tab_simulation, text="Simulation")
        self.notebook.add(self.tab_thermal, text="Thermal Analysis")
        self.notebook.add(self.tab_mechanical, text="Mechanical Stress")
        self.notebook.add(self.tab_economic, text="Economic Analysis")
        self.notebook.add(self.tab_results, text="Results & Graphs")

        # Build each tab
        self.build_main_tab()
        self.build_simulation_tab()
        self.build_thermal_tab()
        self.build_mechanical_tab()
        self.build_economic_tab()
        self.build_results_tab()

    def build_main_tab(self):
        """Build main menu tab"""
        # Title
        title_frame = ttk.Frame(self.tab_main)
        title_frame.pack(fill=tk.X, padx=10, pady=10)

        title_label = ttk.Label(title_frame, text="DC Motor Braking Simulation System",
                               font=('Arial', 16, 'bold'))
        title_label.pack()

        subtitle_label = ttk.Label(title_frame, text="Example 30.47 Solution & Multi-Physics Analysis",
                                  font=('Arial', 10))
        subtitle_label.pack()

        # Example 30.47 Solution Frame
        solution_frame = ttk.LabelFrame(self.tab_main, text="Example 30.47 - Braking Calculation Results")
        solution_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)

        self.solution_text = tk.Text(solution_frame, height=15, width=80, font=('Courier', 10))
        self.solution_text.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)

        scrollbar = ttk.Scrollbar(solution_frame, command=self.solution_text.yview)
        scrollbar.pack(side=tk.RIGHT, fill=tk.Y)
        self.solution_text.config(yscrollcommand=scrollbar.set)

        # Input Parameters Frame
        input_frame = ttk.LabelFrame(self.tab_main, text="Motor Parameters")
        input_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)

        # Create parameter inputs
        params = [
            ("Voltage (V):", "V", 400),
            ("Power (W):", "P", 18650),
            ("Speed (rpm):", "N", 45),
            ("Efficiency:", "eta", 0.746),
            ("Armature Resistance (Ω):", "Ra", 0.2),
            ("Armature Inductance (H):", "La", 0.05),
            ("Inertia (kg·m²):", "J", 10),
            ("Friction (N·m·s):", "B", 0.1),
        ]

        self.param_vars = {}
        for i, (label, var_name, default) in enumerate(params):
            row = i // 2
            col = (i % 2) * 3

            ttk.Label(input_frame, text=label).grid(row=row, column=col, sticky=tk.W, padx=5, pady=5)
            var = tk.DoubleVar(value=default)
            entry = ttk.Entry(input_frame, textvariable=var, width=15)
            entry.grid(row=row, column=col+1, padx=5, pady=5)
            self.param_vars[var_name] = var

        # Update button
        update_btn = ttk.Button(input_frame, text="Update Parameters & Recalculate",
                               command=self.update_parameters)
        update_btn.grid(row=len(params)//2 + 1, column=0, columnspan=6, pady=10)

    def build_simulation_tab(self):
        """Build simulation control tab"""
        # Control Frame
        control_frame = ttk.LabelFrame(self.tab_simulation, text="Simulation Controls")
        control_frame.pack(fill=tk.X, padx=10, pady=10)

        # Solver selection
        ttk.Label(control_frame, text="ODE Solver:").grid(row=0, column=0, padx=5, pady=5, sticky=tk.W)
        self.solver_var = tk.StringVar(value='RK45')
        solver_combo = ttk.Combobox(control_frame, textvariable=self.solver_var,
                                    values=['RK45', 'Euler', 'LSODA'], state='readonly', width=15)
        solver_combo.grid(row=0, column=1, padx=5, pady=5)

        # Simulation time
        ttk.Label(control_frame, text="Simulation Time (s):").grid(row=0, column=2, padx=5, pady=5, sticky=tk.W)
        self.sim_time_var = tk.DoubleVar(value=5.0)
        ttk.Entry(control_frame, textvariable=self.sim_time_var, width=10).grid(row=0, column=3, padx=5, pady=5)

        # Braking mode
        ttk.Label(control_frame, text="Braking Mode:").grid(row=1, column=0, padx=5, pady=5, sticky=tk.W)
        self.braking_mode_var = tk.StringVar(value='Plugging')
        braking_combo = ttk.Combobox(control_frame, textvariable=self.braking_mode_var,
                                     values=['Plugging', 'Dynamic', 'Regenerative'],
                                     state='readonly', width=15)
        braking_combo.grid(row=1, column=1, padx=5, pady=5)

        # Control buttons
        btn_frame = ttk.Frame(control_frame)
        btn_frame.grid(row=2, column=0, columnspan=4, pady=10)

        self.start_btn = ttk.Button(btn_frame, text="Start Simulation", command=self.start_simulation)
        self.start_btn.pack(side=tk.LEFT, padx=5)

        self.stop_btn = ttk.Button(btn_frame, text="Stop Simulation", command=self.stop_simulation, state=tk.DISABLED)
        self.stop_btn.pack(side=tk.LEFT, padx=5)

        self.reset_btn = ttk.Button(btn_frame, text="Reset", command=self.reset_simulation)
        self.reset_btn.pack(side=tk.LEFT, padx=5)

        # Adjustment Sliders Frame
        slider_frame = ttk.LabelFrame(self.tab_simulation, text="Real-Time Control Adjustments")
        slider_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)

        # Voltage slider
        ttk.Label(slider_frame, text="Applied Voltage (V):").grid(row=0, column=0, sticky=tk.W, padx=5, pady=5)
        self.voltage_slider_var = tk.DoubleVar(value=400)
        self.voltage_slider = ttk.Scale(slider_frame, from_=0, to=600, variable=self.voltage_slider_var,
                                       orient=tk.HORIZONTAL, length=400)
        self.voltage_slider.grid(row=0, column=1, padx=5, pady=5)
        self.voltage_label = ttk.Label(slider_frame, text="400 V")
        self.voltage_label.grid(row=0, column=2, padx=5, pady=5)
        self.voltage_slider_var.trace('w', lambda *args: self.voltage_label.config(text=f"{self.voltage_slider_var.get():.1f} V"))

        # Load torque slider
        ttk.Label(slider_frame, text="Load Torque (N·m):").grid(row=1, column=0, sticky=tk.W, padx=5, pady=5)
        self.load_slider_var = tk.DoubleVar(value=self.motor.T_rated)
        self.load_slider = ttk.Scale(slider_frame, from_=0, to=2*self.motor.T_rated,
                                    variable=self.load_slider_var, orient=tk.HORIZONTAL, length=400)
        self.load_slider.grid(row=1, column=1, padx=5, pady=5)
        self.load_label = ttk.Label(slider_frame, text=f"{self.motor.T_rated:.1f} N·m")
        self.load_label.grid(row=1, column=2, padx=5, pady=5)
        self.load_slider_var.trace('w', lambda *args: self.load_label.config(text=f"{self.load_slider_var.get():.1f} N·m"))

        # Braking resistance slider
        ttk.Label(slider_frame, text="Braking Resistance (Ω):").grid(row=2, column=0, sticky=tk.W, padx=5, pady=5)
        self.rb_slider_var = tk.DoubleVar(value=0)
        self.rb_slider = ttk.Scale(slider_frame, from_=0, to=10, variable=self.rb_slider_var,
                                  orient=tk.HORIZONTAL, length=400)
        self.rb_slider.grid(row=2, column=1, padx=5, pady=5)
        self.rb_label = ttk.Label(slider_frame, text="0.0 Ω")
        self.rb_label.grid(row=2, column=2, padx=5, pady=5)
        self.rb_slider_var.trace('w', lambda *args: self.rb_label.config(text=f"{self.rb_slider_var.get():.2f} Ω"))

        # Status display
        status_frame = ttk.LabelFrame(self.tab_simulation, text="Current Status")
        status_frame.pack(fill=tk.X, padx=10, pady=10)

        self.status_text = tk.Text(status_frame, height=6, width=80, font=('Courier', 9))
        self.status_text.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)

    def build_thermal_tab(self):
        """Build thermal analysis tab"""
        # Thermal parameters
        param_frame = ttk.LabelFrame(self.tab_thermal, text="Thermal Parameters")
        param_frame.pack(fill=tk.X, padx=10, pady=10)

        ttk.Label(param_frame, text="Thermal Resistance (°C/W):").grid(row=0, column=0, sticky=tk.W, padx=5, pady=5)
        self.thermal_res_var = tk.DoubleVar(value=self.motor.thermal_resistance)
        ttk.Entry(param_frame, textvariable=self.thermal_res_var, width=15).grid(row=0, column=1, padx=5, pady=5)

        ttk.Label(param_frame, text="Thermal Capacitance (J/°C):").grid(row=0, column=2, sticky=tk.W, padx=5, pady=5)
        self.thermal_cap_var = tk.DoubleVar(value=self.motor.thermal_capacitance)
        ttk.Entry(param_frame, textvariable=self.thermal_cap_var, width=15).grid(row=0, column=3, padx=5, pady=5)

        ttk.Label(param_frame, text="Ambient Temp (°C):").grid(row=1, column=0, sticky=tk.W, padx=5, pady=5)
        self.ambient_temp_var = tk.DoubleVar(value=self.motor.ambient_temp)
        ttk.Entry(param_frame, textvariable=self.ambient_temp_var, width=15).grid(row=1, column=1, padx=5, pady=5)

        ttk.Label(param_frame, text="Max Temp (°C):").grid(row=1, column=2, sticky=tk.W, padx=5, pady=5)
        self.max_temp_var = tk.DoubleVar(value=self.motor.max_temp)
        ttk.Entry(param_frame, textvariable=self.max_temp_var, width=15).grid(row=1, column=3, padx=5, pady=5)

        # Thermal derating
        self.thermal_derating_var = tk.BooleanVar(value=True)
        ttk.Checkbutton(param_frame, text="Enable Thermal Derating",
                       variable=self.thermal_derating_var).grid(row=2, column=0, columnspan=2, pady=5)

        # Thermal graph
        graph_frame = ttk.LabelFrame(self.tab_thermal, text="Temperature & Loss Distribution")
        graph_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)

        self.thermal_fig = Figure(figsize=(10, 6), dpi=80)
        self.thermal_canvas = FigureCanvasTkAgg(self.thermal_fig, graph_frame)
        self.thermal_canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)

    def build_mechanical_tab(self):
        """Build mechanical stress analysis tab"""
        # Mechanical parameters
        param_frame = ttk.LabelFrame(self.tab_mechanical, text="Mechanical Parameters")
        param_frame.pack(fill=tk.X, padx=10, pady=10)

        ttk.Label(param_frame, text="Shaft Diameter (m):").grid(row=0, column=0, sticky=tk.W, padx=5, pady=5)
        self.shaft_dia_var = tk.DoubleVar(value=self.motor.shaft_diameter)
        ttk.Entry(param_frame, textvariable=self.shaft_dia_var, width=15).grid(row=0, column=1, padx=5, pady=5)

        ttk.Label(param_frame, text="Shaft Length (m):").grid(row=0, column=2, sticky=tk.W, padx=5, pady=5)
        self.shaft_len_var = tk.DoubleVar(value=self.motor.shaft_length)
        ttk.Entry(param_frame, textvariable=self.shaft_len_var, width=15).grid(row=0, column=3, padx=5, pady=5)

        ttk.Label(param_frame, text="Bearing Friction (N·m):").grid(row=1, column=0, sticky=tk.W, padx=5, pady=5)
        self.bearing_fric_var = tk.DoubleVar(value=self.motor.bearing_friction)
        ttk.Entry(param_frame, textvariable=self.bearing_fric_var, width=15).grid(row=1, column=1, padx=5, pady=5)

        # Mechanical analysis display
        analysis_frame = ttk.LabelFrame(self.tab_mechanical, text="Stress Analysis Results")
        analysis_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)

        self.mechanical_text = tk.Text(analysis_frame, height=10, width=80, font=('Courier', 10))
        self.mechanical_text.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)

        # Mechanical graph
        graph_frame = ttk.LabelFrame(self.tab_mechanical, text="Torque Transients & Shaft Stress")
        graph_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)

        self.mechanical_fig = Figure(figsize=(10, 6), dpi=80)
        self.mechanical_canvas = FigureCanvasTkAgg(self.mechanical_fig, graph_frame)
        self.mechanical_canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)

    def build_economic_tab(self):
        """Build economic analysis tab"""
        # Cost parameters
        param_frame = ttk.LabelFrame(self.tab_economic, text="Economic Parameters")
        param_frame.pack(fill=tk.X, padx=10, pady=10)

        ttk.Label(param_frame, text="Electricity Cost ($/kWh):").grid(row=0, column=0, sticky=tk.W, padx=5, pady=5)
        self.elec_cost_var = tk.DoubleVar(value=0.12)
        ttk.Entry(param_frame, textvariable=self.elec_cost_var, width=15).grid(row=0, column=1, padx=5, pady=5)

        ttk.Label(param_frame, text="Operating Hours/Year:").grid(row=0, column=2, sticky=tk.W, padx=5, pady=5)
        self.op_hours_var = tk.DoubleVar(value=4000)
        ttk.Entry(param_frame, textvariable=self.op_hours_var, width=15).grid(row=0, column=3, padx=5, pady=5)

        ttk.Label(param_frame, text="Maintenance Cost/Year ($):").grid(row=1, column=0, sticky=tk.W, padx=5, pady=5)
        self.maint_cost_var = tk.DoubleVar(value=500)
        ttk.Entry(param_frame, textvariable=self.maint_cost_var, width=15).grid(row=1, column=1, padx=5, pady=5)

        ttk.Label(param_frame, text="Motor Initial Cost ($):").grid(row=1, column=2, sticky=tk.W, padx=5, pady=5)
        self.initial_cost_var = tk.DoubleVar(value=5000)
        ttk.Entry(param_frame, textvariable=self.initial_cost_var, width=15).grid(row=1, column=3, padx=5, pady=5)

        ttk.Label(param_frame, text="Lifespan (years):").grid(row=2, column=0, sticky=tk.W, padx=5, pady=5)
        self.lifespan_var = tk.DoubleVar(value=15)
        ttk.Entry(param_frame, textvariable=self.lifespan_var, width=15).grid(row=2, column=1, padx=5, pady=5)

        # Calculate button
        calc_btn = ttk.Button(param_frame, text="Calculate Economic Analysis",
                             command=self.calculate_economics)
        calc_btn.grid(row=3, column=0, columnspan=4, pady=10)

        # Results display
        results_frame = ttk.LabelFrame(self.tab_economic, text="Economic Analysis Results")
        results_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)

        self.economic_text = tk.Text(results_frame, height=15, width=80, font=('Courier', 10))
        self.economic_text.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)

        # Economic graph
        graph_frame = ttk.LabelFrame(self.tab_economic, text="Cost Analysis Charts")
        graph_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)

        self.economic_fig = Figure(figsize=(10, 6), dpi=80)
        self.economic_canvas = FigureCanvasTkAgg(self.economic_fig, graph_frame)
        self.economic_canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)

    def build_results_tab(self):
        """Build results and visualization tab"""
        # Graph area
        self.results_fig = Figure(figsize=(12, 8), dpi=80)
        self.results_canvas = FigureCanvasTkAgg(self.results_fig, self.tab_results)
        self.results_canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True, padx=5, pady=5)

        # Control frame for graph options
        control_frame = ttk.Frame(self.tab_results)
        control_frame.pack(fill=tk.X, padx=5, pady=5)

        ttk.Button(control_frame, text="Export Data", command=self.export_data).pack(side=tk.LEFT, padx=5)
        ttk.Button(control_frame, text="Save Plots", command=self.save_plots).pack(side=tk.LEFT, padx=5)
        ttk.Button(control_frame, text="Refresh Graphs", command=self.update_results_graphs).pack(side=tk.LEFT, padx=5)

    def solve_example_30_47(self):
        """Solve Example 30.47 and display results"""
        result = self.motor.calculate_braking_resistance(max_current_factor=2)

        output = f"""
{'='*80}
EXAMPLE 30.47 - DC MOTOR BRAKING BY PLUGGING - SOLUTION
{'='*80}

GIVEN DATA:
  Supply Voltage (V)             : {self.motor.V} V
  Motor Power                    : 25 h.p. = {self.motor.P} W
  Motor Speed                    : {self.motor.N} r.p.m.
  Efficiency (η)                 : {self.motor.eta*100}%
  Armature Resistance (Ra)       : {self.motor.Ra} Ω
  Braking Method                 : Plugging
  Maximum Braking Current        : 2 × Full-load current

{'='*80}
STEP-BY-STEP SOLUTION:
{'='*80}

Step 1: Calculate Full-Load Current
  Input Power = Output Power / Efficiency
  Input Power = {self.motor.P} / {self.motor.eta} = {self.motor.P/self.motor.eta:.2f} W
  Full-load Current (If) = Input Power / V
  If = {self.motor.P/self.motor.eta:.2f} / {self.motor.V} = {self.motor.If_total:.2f} A

Step 2: Calculate Armature Current (assuming small field current)
  Armature Current (Ia) ≈ If = {self.motor.Ia_rated:.2f} A

Step 3: Calculate Back EMF at Full Load
  Back EMF (Eb) = V - Ia × Ra
  Eb = {self.motor.V} - {self.motor.Ia_rated:.2f} × {self.motor.Ra}
  Eb = {self.motor.Eb_rated:.2f} V

Step 4: Calculate Braking Resistance
  During Plugging, motor terminals are reversed:
  Total Voltage = V + Eb = {self.motor.V} + {self.motor.Eb_rated:.2f} = {self.motor.V + self.motor.Eb_rated:.2f} V

  Maximum Braking Current (Ib_max) = 2 × If = 2 × {self.motor.If_total:.2f} = {result['Ib_max']:.2f} A

  Total Resistance Required (R_total) = Total Voltage / Ib_max
  R_total = {self.motor.V + self.motor.Eb_rated:.2f} / {result['Ib_max']:.2f} = {result['R_total']:.3f} Ω

  Braking Resistance (Rb) = R_total - Ra
  Rb = {result['R_total']:.3f} - {self.motor.Ra} = {result['Rb']:.3f} Ω

Step 5: Calculate Maximum Braking Torque
  Torque is proportional to armature current (T ∝ Ia)
  Rated Torque (Tf) = Power / Angular Velocity
  Tf = {self.motor.P} / {self.motor.omega_rated:.4f} = {self.motor.T_rated:.2f} N·m

  Maximum Braking Torque = (Ib_max / Ia_rated) × Tf
  T_brake_max = ({result['Ib_max']:.2f} / {self.motor.Ia_rated:.2f}) × {self.motor.T_rated:.2f}
  T_brake_max = {result['T_brake_max']:.2f} N·m

Step 6: Calculate Braking Torque at Zero Speed
  When Speed = 0, Back EMF (Eb) = 0
  Current at Zero Speed (Ib_zero) = V / (Ra + Rb)
  Ib_zero = {self.motor.V} / ({self.motor.Ra} + {result['Rb']:.3f}) = {self.motor.V/(self.motor.Ra + result['Rb']):.2f} A

  Braking Torque at Zero Speed = (Ib_zero / Ia_rated) × Tf
  T_brake_zero = ({self.motor.V/(self.motor.Ra + result['Rb']):.2f} / {self.motor.Ia_rated:.2f}) × {self.motor.T_rated:.2f}
  T_brake_zero = {result['T_brake_zero']:.2f} N·m

{'='*80}
FINAL ANSWERS:
{'='*80}

  1. Required Braking Resistance (Rb)        : {result['Rb']:.3f} Ω

  2. Maximum Braking Torque                  : {result['T_brake_max']:.2f} N·m

  3. Braking Torque at Zero Speed            : {result['T_brake_zero']:.2f} N·m

{'='*80}
RMS VALUES FOR SIMULATION:
{'='*80}
  RMS Voltage                                : {self.motor.V/np.sqrt(2):.2f} V (for AC equivalent)
  RMS Current at Max Braking                 : {result['Ib_max']/np.sqrt(2):.2f} A (for AC equivalent)

  Note: For DC analysis, we use DC values directly. RMS conversion shown for reference.

{'='*80}
Generated: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}
{'='*80}
"""

        self.solution_text.delete(1.0, tk.END)
        self.solution_text.insert(1.0, output)

        # Set recommended braking resistance in slider
        self.rb_slider_var.set(result['Rb'])

    def update_parameters(self):
        """Update motor parameters from input fields"""
        try:
            self.motor.V = self.param_vars['V'].get()
            self.motor.P = self.param_vars['P'].get()
            self.motor.N = self.param_vars['N'].get()
            self.motor.eta = self.param_vars['eta'].get()
            self.motor.Ra = self.param_vars['Ra'].get()
            self.motor.La = self.param_vars['La'].get()
            self.motor.J = self.param_vars['J'].get()
            self.motor.B = self.param_vars['B'].get()

            # Recalculate derived parameters
            self.motor.omega_rated = (2 * np.pi * self.motor.N) / 60
            self.motor.If_total = self.motor.P / (self.motor.eta * self.motor.V)
            self.motor.Ish = self.motor.V / self.motor.Rf
            self.motor.Ia_rated = self.motor.If_total - self.motor.Ish
            self.motor.Eb_rated = self.motor.V - self.motor.Ia_rated * self.motor.Ra
            self.motor.Kt = self.motor.Eb_rated / self.motor.omega_rated
            self.motor.Ke = self.motor.Kt
            self.motor.T_rated = self.motor.P / self.motor.omega_rated

            # Reinitialize control system
            self.control = ControlSystem(self.motor)

            # Resolve example
            self.solve_example_30_47()

            messagebox.showinfo("Success", "Parameters updated successfully!")

        except Exception as e:
            messagebox.showerror("Error", f"Failed to update parameters:\n{str(e)}")

    def start_simulation(self):
        """Start the simulation"""
        self.simulation_running = True
        self.start_btn.config(state=tk.DISABLED)
        self.stop_btn.config(state=tk.NORMAL)

        try:
            # Get simulation parameters
            t_end = self.sim_time_var.get()
            V_applied = self.voltage_slider_var.get()
            T_load = self.load_slider_var.get()
            Rb = self.rb_slider_var.get()
            solver = self.solver_var.get()

            # For plugging, reverse voltage
            if self.braking_mode_var.get() == 'Plugging':
                V_applied = -V_applied

            # Initial conditions: [Ia, omega, theta, T_motor]
            # Start from rated conditions
            y0 = [self.motor.Ia_rated, self.motor.omega_rated, 0, self.motor.T_rated]

            # Time span
            t_span = (0, t_end)
            t_eval = np.linspace(0, t_end, 1000)

            # Solve based on selected method
            if solver == 'RK45':
                sol = solve_ivp(
                    lambda t, y: self.motor.motor_dynamics_rk45(t, y, V_applied, T_load, Rb),
                    t_span, y0, method='RK45', t_eval=t_eval, max_step=0.01
                )
                t = sol.t
                Ia = sol.y[0]
                omega = sol.y[1]
                theta = sol.y[2]
                T_motor = sol.y[3]

            elif solver == 'Euler':
                # Manual Euler implementation
                dt = 0.001
                t = np.arange(0, t_end, dt)
                y = np.zeros((len(t), 4))
                y[0] = y0

                for i in range(1, len(t)):
                    dydt = self.motor.motor_dynamics_euler(y[i-1], t[i-1], V_applied, T_load, Rb)
                    y[i] = y[i-1] + np.array(dydt) * dt

                Ia = y[:, 0]
                omega = y[:, 1]
                theta = y[:, 2]
                T_motor = y[:, 3]

            else:  # LSODA
                t = t_eval
                sol = odeint(
                    lambda y, t: self.motor.motor_dynamics_euler(y, t, V_applied, T_load, Rb),
                    y0, t
                )
                Ia = sol[:, 0]
                omega = sol[:, 1]
                theta = sol[:, 2]
                T_motor = sol[:, 3]

            # Calculate additional quantities
            N_rpm = omega * 60 / (2 * np.pi)
            Eb = self.motor.Ke * omega
            V_armature = Eb + Ia * (self.motor.Ra + Rb)
            P_mech = T_motor * omega
            P_elec = V_applied * Ia

            # Calculate losses over time
            losses = []
            temps = []
            shaft_stress = []
            bearing_loads = []

            T_temp = self.ambient_temp_var.get()

            for i in range(len(t)):
                loss = self.motor.calculate_losses(Ia[i], omega[i])
                losses.append(loss)

                # Thermal simulation
                if i > 0:
                    dt_thermal = t[i] - t[i-1]
                    dT = (loss['total'] - (T_temp - self.ambient_temp_var.get()) /
                          self.thermal_res_var.get()) / self.thermal_cap_var.get() * dt_thermal
                    T_temp += dT

                temps.append(T_temp)

                # Mechanical stress
                stress = self.motor.mechanical_stress_analysis(T_motor[i])
                shaft_stress.append(stress['shaft_stress'])
                bearing_loads.append(stress['bearing_load'])

            # Store simulation data
            self.simulation_data = {
                't': t,
                'Ia': Ia,
                'omega': omega,
                'N_rpm': N_rpm,
                'theta': theta,
                'T_motor': T_motor,
                'Eb': Eb,
                'V_armature': V_armature,
                'P_mech': P_mech,
                'P_elec': P_elec,
                'losses': losses,
                'temps': np.array(temps),
                'shaft_stress': np.array(shaft_stress),
                'bearing_loads': np.array(bearing_loads),
                'V_applied': V_applied,
                'T_load': T_load,
                'Rb': Rb
            }

            # Update all graphs
            self.update_results_graphs()
            self.update_thermal_graphs()
            self.update_mechanical_graphs()
            self.update_status()

            messagebox.showinfo("Success", f"Simulation completed using {solver} solver!")

        except Exception as e:
            messagebox.showerror("Error", f"Simulation failed:\n{str(e)}")
            import traceback
            traceback.print_exc()

        finally:
            self.simulation_running = False
            self.start_btn.config(state=tk.NORMAL)
            self.stop_btn.config(state=tk.DISABLED)

    def stop_simulation(self):
        """Stop the simulation"""
        self.simulation_running = False
        self.start_btn.config(state=tk.NORMAL)
        self.stop_btn.config(state=tk.DISABLED)

    def reset_simulation(self):
        """Reset simulation to initial state"""
        self.simulation_data = None
        self.current_time = 0
        self.voltage_slider_var.set(400)
        self.load_slider_var.set(self.motor.T_rated)
        self.rb_slider_var.set(0)

        # Clear graphs
        self.results_fig.clear()
        self.results_canvas.draw()
        self.thermal_fig.clear()
        self.thermal_canvas.draw()
        self.mechanical_fig.clear()
        self.mechanical_canvas.draw()

        self.status_text.delete(1.0, tk.END)

    def update_results_graphs(self):
        """Update result graphs with simulation data"""
        if self.simulation_data is None:
            return

        self.results_fig.clear()

        data = self.simulation_data

        # Create subplots
        ax1 = self.results_fig.add_subplot(3, 2, 1)
        ax2 = self.results_fig.add_subplot(3, 2, 2)
        ax3 = self.results_fig.add_subplot(3, 2, 3)
        ax4 = self.results_fig.add_subplot(3, 2, 4)
        ax5 = self.results_fig.add_subplot(3, 2, 5)
        ax6 = self.results_fig.add_subplot(3, 2, 6)

        # Current vs Time
        ax1.plot(data['t'], data['Ia'], 'b-', linewidth=2)
        ax1.set_xlabel('Time (s)')
        ax1.set_ylabel('Armature Current (A)')
        ax1.set_title('Armature Current vs Time')
        ax1.grid(True, alpha=0.3)
        ax1.axhline(y=0, color='k', linestyle='--', alpha=0.5)

        # Speed vs Time
        ax2.plot(data['t'], data['N_rpm'], 'r-', linewidth=2)
        ax2.set_xlabel('Time (s)')
        ax2.set_ylabel('Speed (rpm)')
        ax2.set_title('Motor Speed vs Time')
        ax2.grid(True, alpha=0.3)
        ax2.axhline(y=0, color='k', linestyle='--', alpha=0.5)

        # Torque vs Time
        ax3.plot(data['t'], data['T_motor'], 'g-', linewidth=2, label='Motor Torque')
        ax3.axhline(y=data['T_load'], color='orange', linestyle='--', linewidth=2, label='Load Torque')
        ax3.set_xlabel('Time (s)')
        ax3.set_ylabel('Torque (N·m)')
        ax3.set_title('Torque vs Time')
        ax3.legend()
        ax3.grid(True, alpha=0.3)
        ax3.axhline(y=0, color='k', linestyle='--', alpha=0.5)

        # Back EMF vs Time
        ax4.plot(data['t'], data['Eb'], 'm-', linewidth=2)
        ax4.set_xlabel('Time (s)')
        ax4.set_ylabel('Back EMF (V)')
        ax4.set_title('Back EMF vs Time')
        ax4.grid(True, alpha=0.3)
        ax4.axhline(y=0, color='k', linestyle='--', alpha=0.5)

        # Power vs Time
        ax5.plot(data['t'], data['P_mech']/1000, 'c-', linewidth=2, label='Mechanical')
        ax5.plot(data['t'], data['P_elec']/1000, 'y-', linewidth=2, label='Electrical')
        ax5.set_xlabel('Time (s)')
        ax5.set_ylabel('Power (kW)')
        ax5.set_title('Power vs Time')
        ax5.legend()
        ax5.grid(True, alpha=0.3)
        ax5.axhline(y=0, color='k', linestyle='--', alpha=0.5)

        # Torque-Speed Characteristic
        ax6.plot(data['N_rpm'], data['T_motor'], 'b-', linewidth=2)
        ax6.set_xlabel('Speed (rpm)')
        ax6.set_ylabel('Torque (N·m)')
        ax6.set_title('Torque-Speed Characteristic')
        ax6.grid(True, alpha=0.3)
        ax6.axhline(y=0, color='k', linestyle='--', alpha=0.5)
        ax6.axvline(x=0, color='k', linestyle='--', alpha=0.5)

        self.results_fig.tight_layout()
        self.results_canvas.draw()

    def update_thermal_graphs(self):
        """Update thermal analysis graphs"""
        if self.simulation_data is None:
            return

        self.thermal_fig.clear()

        data = self.simulation_data

        ax1 = self.thermal_fig.add_subplot(2, 2, 1)
        ax2 = self.thermal_fig.add_subplot(2, 2, 2)
        ax3 = self.thermal_fig.add_subplot(2, 2, 3)
        ax4 = self.thermal_fig.add_subplot(2, 2, 4)

        # Temperature vs Time
        ax1.plot(data['t'], data['temps'], 'r-', linewidth=2, label='Motor Temp')
        ax1.axhline(y=self.max_temp_var.get(), color='orange', linestyle='--',
                   linewidth=2, label=f"Max Temp ({self.max_temp_var.get()}°C)")
        ax1.axhline(y=self.ambient_temp_var.get(), color='b', linestyle='--',
                   linewidth=1, label=f"Ambient ({self.ambient_temp_var.get()}°C)")
        ax1.set_xlabel('Time (s)')
        ax1.set_ylabel('Temperature (°C)')
        ax1.set_title('Temperature Rise vs Time')
        ax1.legend()
        ax1.grid(True, alpha=0.3)

        # Loss breakdown (pie chart at final time)
        final_loss = data['losses'][-1]
        labels = ['Copper\nArmature', 'Copper\nField', 'Iron', 'Friction', 'Stray']
        sizes = [final_loss['copper_armature'], final_loss['copper_field'],
                final_loss['iron'], final_loss['friction'], final_loss['stray']]
        colors = ['#ff9999', '#ff6666', '#66b3ff', '#99ff99', '#ffcc99']
        ax2.pie(sizes, labels=labels, colors=colors, autopct='%1.1f%%', startangle=90)
        ax2.set_title('Loss Distribution Breakdown')

        # Total losses vs Time
        total_losses = [loss['total'] for loss in data['losses']]
        ax3.plot(data['t'], np.array(total_losses)/1000, 'r-', linewidth=2)
        ax3.set_xlabel('Time (s)')
        ax3.set_ylabel('Total Losses (kW)')
        ax3.set_title('Total Power Losses vs Time')
        ax3.grid(True, alpha=0.3)

        # Individual loss components vs Time
        copper_losses = [loss['copper_total'] for loss in data['losses']]
        iron_losses = [loss['iron'] for loss in data['losses']]
        friction_losses = [loss['friction'] for loss in data['losses']]

        ax4.plot(data['t'], np.array(copper_losses)/1000, label='Copper', linewidth=2)
        ax4.plot(data['t'], np.array(iron_losses)/1000, label='Iron', linewidth=2)
        ax4.plot(data['t'], np.array(friction_losses)/1000, label='Friction', linewidth=2)
        ax4.set_xlabel('Time (s)')
        ax4.set_ylabel('Losses (kW)')
        ax4.set_title('Loss Components vs Time')
        ax4.legend()
        ax4.grid(True, alpha=0.3)

        self.thermal_fig.tight_layout()
        self.thermal_canvas.draw()

    def update_mechanical_graphs(self):
        """Update mechanical stress graphs"""
        if self.simulation_data is None:
            return

        self.mechanical_fig.clear()

        data = self.simulation_data

        ax1 = self.mechanical_fig.add_subplot(2, 2, 1)
        ax2 = self.mechanical_fig.add_subplot(2, 2, 2)
        ax3 = self.mechanical_fig.add_subplot(2, 2, 3)
        ax4 = self.mechanical_fig.add_subplot(2, 2, 4)

        # Shaft stress vs Time
        ax1.plot(data['t'], data['shaft_stress'], 'b-', linewidth=2)
        ax1.set_xlabel('Time (s)')
        ax1.set_ylabel('Shaft Shear Stress (MPa)')
        ax1.set_title('Shaft Stress vs Time')
        ax1.grid(True, alpha=0.3)

        # Bearing load vs Time
        ax2.plot(data['t'], data['bearing_loads'], 'g-', linewidth=2)
        ax2.set_xlabel('Time (s)')
        ax2.set_ylabel('Bearing Load (kN)')
        ax2.set_title('Bearing Load vs Time')
        ax2.grid(True, alpha=0.3)

        # Torque transient analysis
        ax3.plot(data['t'], data['T_motor'], 'r-', linewidth=2)
        ax3.set_xlabel('Time (s)')
        ax3.set_ylabel('Torque (N·m)')
        ax3.set_title('Torque Transients')
        ax3.grid(True, alpha=0.3)
        ax3.axhline(y=0, color='k', linestyle='--', alpha=0.5)

        # Angular position vs Time
        ax4.plot(data['t'], data['theta'], 'm-', linewidth=2)
        ax4.set_xlabel('Time (s)')
        ax4.set_ylabel('Angular Position (rad)')
        ax4.set_title('Shaft Angular Position')
        ax4.grid(True, alpha=0.3)

        self.mechanical_fig.tight_layout()
        self.mechanical_canvas.draw()

        # Update text display
        self.mechanical_text.delete(1.0, tk.END)

        max_stress = np.max(data['shaft_stress'])
        max_bearing = np.max(data['bearing_loads'])
        max_torque = np.max(np.abs(data['T_motor']))

        analysis = f"""
{'='*70}
MECHANICAL STRESS ANALYSIS RESULTS
{'='*70}

Shaft Parameters:
  Diameter                    : {self.shaft_dia_var.get()*1000:.1f} mm
  Length                      : {self.shaft_len_var.get()*1000:.1f} mm
  Material                    : Steel (assumed)
  Yield Strength              : 250 MPa (typical)

Stress Analysis:
  Maximum Shaft Stress        : {max_stress:.2f} MPa
  Maximum Bearing Load        : {max_bearing:.2f} kN
  Maximum Torque              : {max_torque:.2f} N·m

Safety Assessment:
  Shaft Stress / Yield        : {max_stress/250*100:.1f}%
  Safety Factor               : {250/max_stress:.2f}

  Status: {'SAFE' if max_stress < 250/2 else 'CAUTION - High Stress'}

Bearing Analysis:
  Maximum Radial Load         : {max_bearing:.2f} kN
  Recommended Bearing Type    : {'Light Duty' if max_bearing < 5 else 'Heavy Duty'}

Dynamic Analysis:
  Peak Torque Transient       : {max_torque:.2f} N·m
  Torque Ripple               : {np.std(data['T_motor']):.2f} N·m (std dev)

{'='*70}
Analysis completed at: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}
{'='*70}
"""

        self.mechanical_text.insert(1.0, analysis)

    def update_status(self):
        """Update status display"""
        if self.simulation_data is None:
            return

        data = self.simulation_data

        # Get final values
        final_Ia = data['Ia'][-1]
        final_speed = data['N_rpm'][-1]
        final_torque = data['T_motor'][-1]
        final_temp = data['temps'][-1]

        status = f"""
Current Simulation Status:
  Time                 : {data['t'][-1]:.3f} s
  Armature Current     : {final_Ia:.2f} A
  Motor Speed          : {final_speed:.2f} rpm
  Motor Torque         : {final_torque:.2f} N·m
  Temperature          : {final_temp:.1f} °C

Simulation Parameters:
  Applied Voltage      : {data['V_applied']:.1f} V
  Load Torque          : {data['T_load']:.2f} N·m
  Braking Resistance   : {data['Rb']:.3f} Ω
  Solver Method        : {self.solver_var.get()}
"""

        self.status_text.delete(1.0, tk.END)
        self.status_text.insert(1.0, status)

    def calculate_economics(self):
        """Calculate economic analysis"""
        try:
            elec_cost = self.elec_cost_var.get()
            op_hours = self.op_hours_var.get()
            maint_cost = self.maint_cost_var.get()
            initial_cost = self.initial_cost_var.get()
            lifespan = self.lifespan_var.get()

            # Calculate annual energy consumption
            avg_power_kW = self.motor.P / 1000
            annual_energy_kWh = avg_power_kW * op_hours
            annual_energy_cost = annual_energy_kWh * elec_cost

            # Calculate losses
            avg_losses = self.motor.calculate_losses(self.motor.Ia_rated, self.motor.omega_rated)
            annual_loss_kWh = (avg_losses['total'] / 1000) * op_hours
            annual_loss_cost = annual_loss_kWh * elec_cost

            # Total annual operating cost
            annual_operating_cost = annual_energy_cost + maint_cost

            # Lifecycle cost
            total_lifecycle_cost = initial_cost + (annual_operating_cost * lifespan)

            # Efficiency improvement potential
            efficiency_improvement = (1 - self.motor.eta) * 100
            potential_savings = annual_energy_cost * (1 - self.motor.eta)

            # Display results
            self.economic_text.delete(1.0, tk.END)

            output = f"""
{'='*80}
ECONOMIC ANALYSIS RESULTS
{'='*80}

INPUT PARAMETERS:
  Electricity Cost                : ${elec_cost:.3f} /kWh
  Operating Hours per Year        : {op_hours:.0f} hours
  Annual Maintenance Cost         : ${maint_cost:.2f}
  Motor Initial Cost              : ${initial_cost:.2f}
  Expected Lifespan               : {lifespan:.0f} years

{'='*80}
ENERGY CONSUMPTION ANALYSIS:
{'='*80}

Annual Energy Consumption:
  Useful Output Energy            : {annual_energy_kWh:.2f} kWh
  Energy Losses                   : {annual_loss_kWh:.2f} kWh
  Total Input Energy              : {annual_energy_kWh + annual_loss_kWh:.2f} kWh

Annual Energy Costs:
  Useful Energy Cost              : ${annual_energy_cost:.2f}
  Wasted Energy Cost (Losses)     : ${annual_loss_cost:.2f}
  Total Annual Energy Cost        : ${annual_energy_cost + annual_loss_cost:.2f}

{'='*80}
LOSS BREAKDOWN:
{'='*80}

Annual Loss Costs by Type:
  Copper Losses (Armature)        : ${avg_losses['copper_armature']/1000 * op_hours * elec_cost:.2f}
  Copper Losses (Field)           : ${avg_losses['copper_field']/1000 * op_hours * elec_cost:.2f}
  Iron Losses                     : ${avg_losses['iron']/1000 * op_hours * elec_cost:.2f}
  Friction Losses                 : ${avg_losses['friction']/1000 * op_hours * elec_cost:.2f}
  Stray Losses                    : ${avg_losses['stray']/1000 * op_hours * elec_cost:.2f}
  Total Annual Loss Cost          : ${annual_loss_cost:.2f}

{'='*80}
OPERATING COST ANALYSIS:
{'='*80}

Annual Costs:
  Energy Cost                     : ${annual_energy_cost + annual_loss_cost:.2f}
  Maintenance Cost                : ${maint_cost:.2f}
  Total Annual Operating Cost     : ${annual_operating_cost + annual_loss_cost:.2f}

Lifecycle Costs ({lifespan:.0f} years):
  Initial Investment              : ${initial_cost:.2f}
  Total Operating Costs           : ${(annual_operating_cost + annual_loss_cost) * lifespan:.2f}
  Total Lifecycle Cost            : ${total_lifecycle_cost + annual_loss_cost * lifespan:.2f}

Average Annual Cost               : ${(total_lifecycle_cost + annual_loss_cost * lifespan) / lifespan:.2f}

{'='*80}
EFFICIENCY & SAVINGS POTENTIAL:
{'='*80}

Current Efficiency                : {self.motor.eta * 100:.2f}%
Efficiency Loss                   : {efficiency_improvement:.2f}%

Potential Annual Savings:
  If Efficiency → 85%             : ${potential_savings * 0.5:.2f}
  If Efficiency → 90%             : ${potential_savings * 0.7:.2f}
  If Efficiency → 95%             : ${potential_savings * 0.9:.2f}

Lifecycle Savings Potential ({lifespan:.0f} years):
  If Efficiency → 90%             : ${potential_savings * 0.7 * lifespan:.2f}

{'='*80}
COST PER UNIT OUTPUT:
{'='*80}

Cost per kWh of useful work       : ${(annual_operating_cost + annual_loss_cost) / annual_energy_kWh:.4f} /kWh
Cost per operating hour           : ${(annual_operating_cost + annual_loss_cost) / op_hours:.4f} /hour

{'='*80}
PAYBACK ANALYSIS:
{'='*80}

If upgrading to higher efficiency motor:
  Additional Cost (estimated)     : ${initial_cost * 0.3:.2f} (30% premium)
  Annual Savings (estimated)      : ${potential_savings * 0.7:.2f}
  Simple Payback Period           : {(initial_cost * 0.3) / (potential_savings * 0.7):.2f} years

{'='*80}
Generated: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}
{'='*80}
"""

            self.economic_text.insert(1.0, output)

            # Create economic graphs
            self.economic_fig.clear()

            ax1 = self.economic_fig.add_subplot(2, 2, 1)
            ax2 = self.economic_fig.add_subplot(2, 2, 2)
            ax3 = self.economic_fig.add_subplot(2, 2, 3)
            ax4 = self.economic_fig.add_subplot(2, 2, 4)

            # Annual cost breakdown (pie chart)
            cost_labels = ['Energy', 'Losses', 'Maintenance']
            cost_sizes = [annual_energy_cost, annual_loss_cost, maint_cost]
            colors = ['#ff9999', '#66b3ff', '#99ff99']
            ax1.pie(cost_sizes, labels=cost_labels, colors=colors, autopct='%1.1f%%', startangle=90)
            ax1.set_title('Annual Cost Breakdown')

            # Lifecycle cost projection
            years = np.arange(0, int(lifespan) + 1)
            cumulative_cost = initial_cost + years * (annual_operating_cost + annual_loss_cost)
            ax2.plot(years, cumulative_cost, 'b-', linewidth=2, marker='o')
            ax2.set_xlabel('Years')
            ax2.set_ylabel('Cumulative Cost ($)')
            ax2.set_title('Lifecycle Cost Projection')
            ax2.grid(True, alpha=0.3)

            # Loss distribution (bar chart)
            loss_types = ['Copper\nArm', 'Copper\nField', 'Iron', 'Friction', 'Stray']
            loss_costs = [
                avg_losses['copper_armature']/1000 * op_hours * elec_cost,
                avg_losses['copper_field']/1000 * op_hours * elec_cost,
                avg_losses['iron']/1000 * op_hours * elec_cost,
                avg_losses['friction']/1000 * op_hours * elec_cost,
                avg_losses['stray']/1000 * op_hours * elec_cost
            ]
            ax3.bar(loss_types, loss_costs, color=['#ff9999', '#ff6666', '#66b3ff', '#99ff99', '#ffcc99'])
            ax3.set_ylabel('Annual Cost ($)')
            ax3.set_title('Loss Cost Distribution')
            ax3.grid(True, alpha=0.3, axis='y')

            # Efficiency improvement savings
            eff_levels = ['Current\n74.6%', '85%', '90%', '95%']
            savings = [0, potential_savings * 0.5 * lifespan, potential_savings * 0.7 * lifespan,
                      potential_savings * 0.9 * lifespan]
            ax4.bar(eff_levels, savings, color=['#ff9999', '#ffcc99', '#99ff99', '#66ff66'])
            ax4.set_ylabel('Lifecycle Savings ($)')
            ax4.set_title(f'Savings Potential ({lifespan:.0f} years)')
            ax4.grid(True, alpha=0.3, axis='y')

            self.economic_fig.tight_layout()
            self.economic_canvas.draw()

        except Exception as e:
            messagebox.showerror("Error", f"Economic analysis failed:\n{str(e)}")

    def export_data(self):
        """Export simulation data to CSV"""
        if self.simulation_data is None:
            messagebox.showwarning("Warning", "No simulation data to export!")
            return

        try:
            from tkinter import filedialog
            filename = filedialog.asksaveasfilename(
                defaultextension=".csv",
                filetypes=[("CSV files", "*.csv"), ("All files", "*.*")]
            )

            if filename:
                data = self.simulation_data

                # Prepare data for export
                header = "Time(s),Current(A),Speed(rpm),Torque(Nm),BackEMF(V),Temperature(C),ShaftStress(MPa),BearingLoad(kN)\n"

                with open(filename, 'w') as f:
                    f.write(header)
                    for i in range(len(data['t'])):
                        f.write(f"{data['t'][i]:.6f},{data['Ia'][i]:.6f},{data['N_rpm'][i]:.6f}," +
                               f"{data['T_motor'][i]:.6f},{data['Eb'][i]:.6f},{data['temps'][i]:.6f}," +
                               f"{data['shaft_stress'][i]:.6f},{data['bearing_loads'][i]:.6f}\n")

                messagebox.showinfo("Success", f"Data exported to:\n{filename}")

        except Exception as e:
            messagebox.showerror("Error", f"Export failed:\n{str(e)}")

    def save_plots(self):
        """Save all plots as images"""
        if self.simulation_data is None:
            messagebox.showwarning("Warning", "No simulation data to save!")
            return

        try:
            from tkinter import filedialog
            filename = filedialog.asksaveasfilename(
                defaultextension=".png",
                filetypes=[("PNG files", "*.png"), ("PDF files", "*.pdf"), ("All files", "*.*")]
            )

            if filename:
                base_name = filename.rsplit('.', 1)[0]
                ext = filename.rsplit('.', 1)[1]

                # Save each figure
                self.results_fig.savefig(f"{base_name}_results.{ext}", dpi=300, bbox_inches='tight')
                self.thermal_fig.savefig(f"{base_name}_thermal.{ext}", dpi=300, bbox_inches='tight')
                self.mechanical_fig.savefig(f"{base_name}_mechanical.{ext}", dpi=300, bbox_inches='tight')
                self.economic_fig.savefig(f"{base_name}_economic.{ext}", dpi=300, bbox_inches='tight')

                messagebox.showinfo("Success", f"Plots saved:\n" +
                                  f"{base_name}_results.{ext}\n" +
                                  f"{base_name}_thermal.{ext}\n" +
                                  f"{base_name}_mechanical.{ext}\n" +
                                  f"{base_name}_economic.{ext}")

        except Exception as e:
            messagebox.showerror("Error", f"Save failed:\n{str(e)}")

    def on_resize(self, event):
        """Handle window resize event for auto-scaling"""
        # This ensures the canvas updates when window is resized
        if hasattr(self, 'results_canvas'):
            self.results_canvas.draw_idle()
        if hasattr(self, 'thermal_canvas'):
            self.thermal_canvas.draw_idle()
        if hasattr(self, 'mechanical_canvas'):
            self.mechanical_canvas.draw_idle()
        if hasattr(self, 'economic_canvas'):
            self.economic_canvas.draw_idle()


# ==================== MAIN APPLICATION ====================

def main():
    """Main application entry point"""
    root = tk.Tk()
    app = DCMotorSimulationGUI(root)
    root.mainloop()


if __name__ == "__main__":
    main()
