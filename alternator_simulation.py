"""
Advanced 3-Phase Alternator Simulation with Multi-Physics Modeling
Comprehensive electrical engineering laboratory application
"""

import tkinter as tk
from tkinter import ttk, messagebox
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure
from scipy.integrate import solve_ivp
import math
from datetime import datetime

class AlternatorPhysicsModel:
    """Multi-physics model for alternator simulation"""

    def __init__(self):
        # Electromagnetic parameters
        self.num_coils = 12
        self.coil_voltage = 10.0  # RMS voltage per coil (V)
        self.phase_shift = 10.0   # Electrical degrees between coils
        self.num_poles = 6
        self.speed_rpm = 100.0

        # Machine physical parameters
        self.stator_resistance = 0.5  # Ohms per phase
        self.stator_inductance = 0.01  # H
        self.rotor_resistance = 0.3   # Ohms
        self.rotor_inductance = 0.008  # H
        self.mutual_inductance = 0.05  # H
        self.rotor_inertia = 0.5  # kg·m²

        # Thermal parameters
        self.ambient_temp = 25.0  # °C
        self.stator_temp = 25.0
        self.rotor_temp = 25.0
        self.thermal_resistance_stator = 2.0  # °C/W
        self.thermal_resistance_rotor = 2.5   # °C/W
        self.thermal_capacitance_stator = 500  # J/°C
        self.thermal_capacitance_rotor = 300   # J/°C

        # Mechanical parameters
        self.shaft_stiffness = 1e6  # N·m/rad
        self.damping_coefficient = 10.0  # N·m·s/rad
        self.bearing_friction = 0.5  # N·m

        # Load parameters
        self.load_resistance = 10.0  # Ohms
        self.load_inductance = 0.005  # H
        self.load_power_factor = 0.9

        # Loss parameters
        self.iron_loss_constant = 0.02
        self.hysteresis_loss_constant = 0.01
        self.eddy_current_loss_constant = 0.005

        # Economic parameters
        self.energy_cost = 0.15  # $/kWh
        self.maintenance_cost_per_hour = 5.0  # $/hr
        self.efficiency_target = 0.92

        # State variables
        self.time = 0.0
        self.angle = 0.0
        self.stator_current = [0.0, 0.0, 0.0]
        self.rotor_current = 0.0
        self.torque = 0.0

    def calculate_phase_voltage(self):
        """Calculate total RMS phase voltage from coil phasors"""
        # Phasor addition of 12 coils
        real_sum = 0.0
        imag_sum = 0.0

        for n in range(self.num_coils):
            angle_rad = math.radians(n * self.phase_shift)
            real_sum += self.coil_voltage * math.cos(angle_rad)
            imag_sum += self.coil_voltage * math.sin(angle_rad)

        # Total RMS voltage
        total_voltage = math.sqrt(real_sum**2 + imag_sum**2)
        return total_voltage, real_sum, imag_sum

    def calculate_frequency(self):
        """Calculate electrical frequency"""
        # f = (P * N) / 120
        frequency = (self.num_poles * self.speed_rpm) / 120.0
        return frequency

    def calculate_angular_velocity(self):
        """Calculate mechanical angular velocity in rad/s"""
        omega_mech = (2 * math.pi * self.speed_rpm) / 60.0
        return omega_mech

    def calculate_copper_losses(self, currents):
        """Calculate copper losses in windings"""
        # Stator copper losses (3 phases)
        stator_loss = sum([self.stator_resistance * I**2 for I in currents])
        # Rotor copper losses
        rotor_loss = self.rotor_resistance * self.rotor_current**2
        return stator_loss + rotor_loss

    def calculate_iron_losses(self, frequency, flux_density):
        """Calculate iron losses (hysteresis + eddy current)"""
        # Hysteresis loss: proportional to f * B^2
        hysteresis_loss = self.hysteresis_loss_constant * frequency * flux_density**2
        # Eddy current loss: proportional to f^2 * B^2
        eddy_loss = self.eddy_current_loss_constant * frequency**2 * flux_density**2
        return hysteresis_loss + eddy_loss

    def calculate_mechanical_losses(self, omega):
        """Calculate mechanical losses (friction + windage)"""
        # Friction losses (constant + speed dependent)
        friction_loss = self.bearing_friction * omega
        # Windage losses (proportional to omega^2)
        windage_loss = 0.001 * omega**2
        return friction_loss + windage_loss

    def calculate_stray_losses(self, power_output):
        """Calculate stray load losses"""
        # Typically 1-2% of output power
        return 0.015 * power_output

    def calculate_efficiency(self, power_out, total_losses):
        """Calculate alternator efficiency"""
        power_in = power_out + total_losses
        if power_in > 0:
            return (power_out / power_in) * 100.0
        return 0.0

    def electromagnetic_ode(self, t, y, load_torque):
        """
        Differential equations for electromagnetic dynamics
        State vector: [i_a, i_b, i_c, i_rotor, omega, theta]
        """
        i_a, i_b, i_c, i_rotor, omega, theta = y

        freq = self.calculate_frequency()
        omega_elec = 2 * math.pi * freq

        # EMF induced in stator windings
        e_a = self.coil_voltage * self.num_coils * math.sin(omega_elec * t)
        e_b = self.coil_voltage * self.num_coils * math.sin(omega_elec * t - 2*math.pi/3)
        e_c = self.coil_voltage * self.num_coils * math.sin(omega_elec * t + 2*math.pi/3)

        # Voltage equations (stator)
        di_a_dt = (e_a - self.stator_resistance * i_a -
                   self.load_resistance * i_a) / (self.stator_inductance + self.load_inductance)
        di_b_dt = (e_b - self.stator_resistance * i_b -
                   self.load_resistance * i_b) / (self.stator_inductance + self.load_inductance)
        di_c_dt = (e_c - self.stator_resistance * i_c -
                   self.load_resistance * i_c) / (self.stator_inductance + self.load_inductance)

        # Rotor equation
        v_rotor = 100.0  # Field excitation voltage
        di_rotor_dt = (v_rotor - self.rotor_resistance * i_rotor) / self.rotor_inductance

        # Electromagnetic torque
        torque_em = self.mutual_inductance * i_rotor * (i_a + i_b + i_c)

        # Mechanical equation
        torque_damping = self.damping_coefficient * omega
        d_omega_dt = (torque_em - load_torque - torque_damping -
                      self.bearing_friction) / self.rotor_inertia

        # Angular position
        d_theta_dt = omega

        return [di_a_dt, di_b_dt, di_c_dt, di_rotor_dt, d_omega_dt, d_theta_dt]

    def thermal_ode(self, t, y, power_losses):
        """
        Differential equations for thermal dynamics
        State vector: [T_stator, T_rotor]
        """
        T_stator, T_rotor = y

        # Heat flow equations
        copper_loss = power_losses['copper']
        iron_loss = power_losses['iron']

        # Stator temperature rise
        heat_in_stator = copper_loss * 0.7 + iron_loss
        heat_out_stator = (T_stator - self.ambient_temp) / self.thermal_resistance_stator
        dT_stator_dt = (heat_in_stator - heat_out_stator) / self.thermal_capacitance_stator

        # Rotor temperature rise
        heat_in_rotor = copper_loss * 0.3
        heat_out_rotor = (T_rotor - self.ambient_temp) / self.thermal_resistance_rotor
        dT_rotor_dt = (heat_in_rotor - heat_out_rotor) / self.thermal_capacitance_rotor

        return [dT_stator_dt, dT_rotor_dt]


class ODESolver:
    """Advanced ODE solver with multiple methods"""

    @staticmethod
    def euler_method(func, y0, t_span, dt, args=()):
        """Euler method for ODE solving"""
        t0, tf = t_span
        t = np.arange(t0, tf, dt)
        y = np.zeros((len(t), len(y0)))
        y[0] = y0

        for i in range(len(t) - 1):
            dy = func(t[i], y[i], *args)
            y[i+1] = y[i] + dt * np.array(dy)

        return t, y

    @staticmethod
    def rk4_method(func, y0, t_span, dt, args=()):
        """4th order Runge-Kutta method"""
        t0, tf = t_span
        t = np.arange(t0, tf, dt)
        y = np.zeros((len(t), len(y0)))
        y[0] = y0

        for i in range(len(t) - 1):
            k1 = np.array(func(t[i], y[i], *args))
            k2 = np.array(func(t[i] + dt/2, y[i] + dt*k1/2, *args))
            k3 = np.array(func(t[i] + dt/2, y[i] + dt*k2/2, *args))
            k4 = np.array(func(t[i] + dt, y[i] + dt*k3, *args))

            y[i+1] = y[i] + (dt/6) * (k1 + 2*k2 + 2*k3 + k4)

        return t, y

    @staticmethod
    def rk45_method(func, y0, t_span, args=()):
        """Adaptive RK45 method using scipy"""
        sol = solve_ivp(func, t_span, y0, method='RK45', args=args,
                       dense_output=True, max_step=0.001)
        return sol


class AlternatorGUI:
    """Main GUI application for alternator simulation"""

    def __init__(self, root):
        self.root = root
        self.root.title("Advanced 3-Phase Alternator Simulation Laboratory")
        self.root.geometry("1400x900")

        # Initialize physics model
        self.model = AlternatorPhysicsModel()
        self.solver = ODESolver()

        # Simulation control
        self.simulation_running = False
        self.simulation_time = 0.0
        self.time_data = []
        self.voltage_data = []
        self.current_data = []
        self.torque_data = []
        self.temp_data = []
        self.efficiency_data = []

        # Economic tracking
        self.total_energy_consumed = 0.0
        self.total_cost = 0.0
        self.operating_hours = 0.0

        # Configure grid weights for auto-scaling
        self.root.grid_rowconfigure(0, weight=1)
        self.root.grid_columnconfigure(0, weight=1)

        # Create main interface
        self.create_menu()
        self.create_main_layout()

        # Bind resize event
        self.root.bind('<Configure>', self.on_resize)

    def create_menu(self):
        """Create menu bar"""
        menubar = tk.Menu(self.root)
        self.root.config(menu=menubar)

        # File menu
        file_menu = tk.Menu(menubar, tearoff=0)
        menubar.add_cascade(label="File", menu=file_menu)
        file_menu.add_command(label="New Simulation", command=self.reset_simulation)
        file_menu.add_command(label="Export Data", command=self.export_data)
        file_menu.add_separator()
        file_menu.add_command(label="Exit", command=self.root.quit)

        # Tools menu
        tools_menu = tk.Menu(menubar, tearoff=0)
        menubar.add_cascade(label="Tools", menu=tools_menu)
        tools_menu.add_command(label="Calculate Phase Voltage", command=self.show_phase_voltage_calculation)
        tools_menu.add_command(label="Frequency Calculator", command=self.show_frequency_calculation)

        # Help menu
        help_menu = tk.Menu(menubar, tearoff=0)
        menubar.add_cascade(label="Help", menu=help_menu)
        help_menu.add_command(label="About", command=self.show_about)

    def create_main_layout(self):
        """Create main layout with tabs"""
        # Create notebook for tabs
        self.notebook = ttk.Notebook(self.root)
        self.notebook.grid(row=0, column=0, sticky='nsew', padx=5, pady=5)

        # Create tabs
        self.create_main_control_tab()
        self.create_electromagnetic_tab()
        self.create_thermal_tab()
        self.create_mechanical_tab()
        self.create_economic_analysis_tab()
        self.create_advanced_control_tab()

    def create_main_control_tab(self):
        """Main control and visualization tab"""
        main_frame = ttk.Frame(self.notebook)
        self.notebook.add(main_frame, text="Main Control")

        # Configure grid
        main_frame.grid_rowconfigure(1, weight=1)
        main_frame.grid_columnconfigure(1, weight=1)

        # Left panel - Controls
        control_frame = ttk.LabelFrame(main_frame, text="Parameters & Controls", padding=10)
        control_frame.grid(row=0, column=0, rowspan=2, sticky='nsew', padx=5, pady=5)

        # Input parameters
        params = [
            ("Number of Coils:", "num_coils", 12, 6, 24),
            ("Coil Voltage (V):", "coil_voltage", 10.0, 1.0, 50.0),
            ("Phase Shift (deg):", "phase_shift", 10.0, 1.0, 30.0),
            ("Number of Poles:", "num_poles", 6, 2, 12),
            ("Speed (RPM):", "speed_rpm", 100.0, 50.0, 3000.0),
            ("Load Resistance (Ω):", "load_resistance", 10.0, 1.0, 100.0),
            ("Load Inductance (H):", "load_inductance", 0.005, 0.001, 0.1),
            ("Field Current (A):", "rotor_current", 2.0, 0.1, 10.0),
        ]

        self.param_vars = {}
        self.param_labels = {}

        for idx, (label, var_name, default, min_val, max_val) in enumerate(params):
            ttk.Label(control_frame, text=label).grid(row=idx, column=0, sticky='w', pady=3)

            var = tk.DoubleVar(value=default)
            self.param_vars[var_name] = var

            slider = ttk.Scale(control_frame, from_=min_val, to=max_val,
                             variable=var, orient='horizontal', length=200,
                             command=lambda v, n=var_name: self.update_parameter(n))
            slider.grid(row=idx, column=1, sticky='ew', pady=3, padx=5)

            label_val = ttk.Label(control_frame, text=f"{default:.2f}")
            label_val.grid(row=idx, column=2, sticky='w', pady=3)
            self.param_labels[var_name] = label_val

        # Control buttons
        button_frame = ttk.Frame(control_frame)
        button_frame.grid(row=len(params), column=0, columnspan=3, pady=20)

        self.start_button = ttk.Button(button_frame, text="Start", command=self.start_simulation, width=12)
        self.start_button.grid(row=0, column=0, padx=5)

        self.stop_button = ttk.Button(button_frame, text="Stop", command=self.stop_simulation,
                                     width=12, state='disabled')
        self.stop_button.grid(row=0, column=1, padx=5)

        self.reset_button = ttk.Button(button_frame, text="Reset", command=self.reset_simulation, width=12)
        self.reset_button.grid(row=0, column=2, padx=5)

        # Status display
        status_frame = ttk.LabelFrame(control_frame, text="System Status", padding=10)
        status_frame.grid(row=len(params)+1, column=0, columnspan=3, sticky='ew', pady=10)

        self.status_labels = {}
        status_items = [
            ("Phase Voltage:", "phase_voltage", "0.00 V"),
            ("Frequency:", "frequency", "0.00 Hz"),
            ("Output Power:", "power_out", "0.00 W"),
            ("Efficiency:", "efficiency", "0.00 %"),
            ("Temperature:", "temperature", "25.0 °C"),
        ]

        for idx, (label, key, default) in enumerate(status_items):
            ttk.Label(status_frame, text=label).grid(row=idx, column=0, sticky='w', pady=2)
            val_label = ttk.Label(status_frame, text=default, font=('Arial', 10, 'bold'))
            val_label.grid(row=idx, column=1, sticky='w', pady=2, padx=10)
            self.status_labels[key] = val_label

        # Right panel - Visualization
        viz_frame = ttk.Frame(main_frame)
        viz_frame.grid(row=0, column=1, rowspan=2, sticky='nsew', padx=5, pady=5)
        viz_frame.grid_rowconfigure(0, weight=1)
        viz_frame.grid_columnconfigure(0, weight=1)

        # Create matplotlib figure
        self.fig_main = Figure(figsize=(10, 8), dpi=100)

        # Subplots
        self.ax_voltage = self.fig_main.add_subplot(3, 2, 1)
        self.ax_current = self.fig_main.add_subplot(3, 2, 2)
        self.ax_power = self.fig_main.add_subplot(3, 2, 3)
        self.ax_torque = self.fig_main.add_subplot(3, 2, 4)
        self.ax_efficiency = self.fig_main.add_subplot(3, 2, 5)
        self.ax_temp = self.fig_main.add_subplot(3, 2, 6)

        self.fig_main.tight_layout(pad=3.0)

        self.canvas_main = FigureCanvasTkAgg(self.fig_main, viz_frame)
        self.canvas_main.get_tk_widget().grid(row=0, column=0, sticky='nsew')

    def create_electromagnetic_tab(self):
        """Electromagnetic analysis tab"""
        em_frame = ttk.Frame(self.notebook)
        self.notebook.add(em_frame, text="Electromagnetic")

        em_frame.grid_rowconfigure(0, weight=1)
        em_frame.grid_columnconfigure(0, weight=1)

        # Create figure for electromagnetic plots
        self.fig_em = Figure(figsize=(12, 8), dpi=100)

        self.ax_phasor = self.fig_em.add_subplot(2, 2, 1, projection='polar')
        self.ax_flux = self.fig_em.add_subplot(2, 2, 2)
        self.ax_emf = self.fig_em.add_subplot(2, 2, 3)
        self.ax_losses_em = self.fig_em.add_subplot(2, 2, 4)

        self.fig_em.tight_layout(pad=3.0)

        canvas_em = FigureCanvasTkAgg(self.fig_em, em_frame)
        canvas_em.get_tk_widget().grid(row=0, column=0, sticky='nsew')

    def create_thermal_tab(self):
        """Thermal analysis tab"""
        thermal_frame = ttk.Frame(self.notebook)
        self.notebook.add(thermal_frame, text="Thermal")

        thermal_frame.grid_rowconfigure(0, weight=1)
        thermal_frame.grid_columnconfigure(0, weight=1)

        # Create figure for thermal plots
        self.fig_thermal = Figure(figsize=(12, 8), dpi=100)

        self.ax_temp_stator = self.fig_thermal.add_subplot(2, 2, 1)
        self.ax_temp_rotor = self.fig_thermal.add_subplot(2, 2, 2)
        self.ax_heat_flow = self.fig_thermal.add_subplot(2, 2, 3)
        self.ax_derating = self.fig_thermal.add_subplot(2, 2, 4)

        self.fig_thermal.tight_layout(pad=3.0)

        canvas_thermal = FigureCanvasTkAgg(self.fig_thermal, thermal_frame)
        canvas_thermal.get_tk_widget().grid(row=0, column=0, sticky='nsew')

    def create_mechanical_tab(self):
        """Mechanical analysis tab"""
        mech_frame = ttk.Frame(self.notebook)
        self.notebook.add(mech_frame, text="Mechanical")

        mech_frame.grid_rowconfigure(0, weight=1)
        mech_frame.grid_columnconfigure(0, weight=1)

        # Create figure for mechanical plots
        self.fig_mech = Figure(figsize=(12, 8), dpi=100)

        self.ax_speed = self.fig_mech.add_subplot(2, 2, 1)
        self.ax_torque_mech = self.fig_mech.add_subplot(2, 2, 2)
        self.ax_shaft_stress = self.fig_mech.add_subplot(2, 2, 3)
        self.ax_bearing_loads = self.fig_mech.add_subplot(2, 2, 4)

        self.fig_mech.tight_layout(pad=3.0)

        canvas_mech = FigureCanvasTkAgg(self.fig_mech, mech_frame)
        canvas_mech.get_tk_widget().grid(row=0, column=0, sticky='nsew')

    def create_economic_analysis_tab(self):
        """Economic analysis tab"""
        econ_frame = ttk.Frame(self.notebook)
        self.notebook.add(econ_frame, text="Economic Analysis")

        econ_frame.grid_rowconfigure(0, weight=2)
        econ_frame.grid_rowconfigure(1, weight=1)
        econ_frame.grid_columnconfigure(0, weight=1)

        # Top - Graphs
        graph_frame = ttk.Frame(econ_frame)
        graph_frame.grid(row=0, column=0, sticky='nsew', padx=5, pady=5)
        graph_frame.grid_rowconfigure(0, weight=1)
        graph_frame.grid_columnconfigure(0, weight=1)

        self.fig_econ = Figure(figsize=(12, 6), dpi=100)

        self.ax_energy_cost = self.fig_econ.add_subplot(1, 3, 1)
        self.ax_loss_breakdown = self.fig_econ.add_subplot(1, 3, 2)
        self.ax_operating_cost = self.fig_econ.add_subplot(1, 3, 3)

        self.fig_econ.tight_layout(pad=3.0)

        canvas_econ = FigureCanvasTkAgg(self.fig_econ, graph_frame)
        canvas_econ.get_tk_widget().grid(row=0, column=0, sticky='nsew')

        # Bottom - Economic parameters
        param_frame = ttk.LabelFrame(econ_frame, text="Economic Parameters", padding=10)
        param_frame.grid(row=1, column=0, sticky='nsew', padx=5, pady=5)

        econ_params = [
            ("Energy Cost ($/kWh):", "energy_cost", 0.15, 0.05, 0.50),
            ("Maintenance Cost ($/hr):", "maintenance_cost_per_hour", 5.0, 1.0, 20.0),
            ("Efficiency Target (%):", "efficiency_target", 92.0, 70.0, 98.0),
        ]

        for idx, (label, var_name, default, min_val, max_val) in enumerate(econ_params):
            ttk.Label(param_frame, text=label).grid(row=idx, column=0, sticky='w', pady=5)

            var = tk.DoubleVar(value=default)
            self.param_vars[var_name] = var

            slider = ttk.Scale(param_frame, from_=min_val, to=max_val,
                             variable=var, orient='horizontal', length=300)
            slider.grid(row=idx, column=1, sticky='ew', pady=5, padx=10)

            label_val = ttk.Label(param_frame, text=f"{default:.3f}")
            label_val.grid(row=idx, column=2, sticky='w', pady=5)
            self.param_labels[var_name] = label_val

        # Economic summary
        summary_frame = ttk.LabelFrame(param_frame, text="Economic Summary", padding=10)
        summary_frame.grid(row=3, column=0, columnspan=3, sticky='ew', pady=10)

        self.econ_labels = {}
        econ_items = [
            ("Total Energy Consumed:", "energy_consumed", "0.00 kWh"),
            ("Total Energy Cost:", "energy_cost_total", "$0.00"),
            ("Total Operating Cost:", "operating_cost", "$0.00"),
            ("Operating Hours:", "operating_hours", "0.00 hrs"),
            ("Cost per Hour:", "cost_per_hour", "$0.00/hr"),
        ]

        for idx, (label, key, default) in enumerate(econ_items):
            ttk.Label(summary_frame, text=label).grid(row=idx, column=0, sticky='w', pady=2)
            val_label = ttk.Label(summary_frame, text=default, font=('Arial', 10, 'bold'))
            val_label.grid(row=idx, column=1, sticky='w', pady=2, padx=10)
            self.econ_labels[key] = val_label

    def create_advanced_control_tab(self):
        """Advanced control methods tab"""
        control_frame = ttk.Frame(self.notebook)
        self.notebook.add(control_frame, text="Advanced Control")

        control_frame.grid_rowconfigure(1, weight=1)
        control_frame.grid_columnconfigure(0, weight=1)

        # Control method selection
        method_frame = ttk.LabelFrame(control_frame, text="Control Methods", padding=10)
        method_frame.grid(row=0, column=0, sticky='ew', padx=5, pady=5)

        self.control_method = tk.StringVar(value="Voltage Control")
        methods = ["Voltage Control", "Frequency Control", "Power Factor Control",
                  "AVR (Automatic Voltage Regulator)", "Excitation Control"]

        for idx, method in enumerate(methods):
            ttk.Radiobutton(method_frame, text=method, variable=self.control_method,
                          value=method).grid(row=0, column=idx, padx=10)

        # Control parameters
        control_param_frame = ttk.LabelFrame(control_frame, text="Control Parameters", padding=10)
        control_param_frame.grid(row=1, column=0, sticky='nsew', padx=5, pady=5)

        control_params = [
            ("Voltage Setpoint (V):", "voltage_setpoint", 230.0, 100.0, 500.0),
            ("Power Factor Setpoint:", "pf_setpoint", 0.9, 0.7, 1.0),
            ("Controller Gain (Kp):", "kp_gain", 1.0, 0.1, 10.0),
            ("Integral Gain (Ki):", "ki_gain", 0.5, 0.01, 5.0),
            ("Derivative Gain (Kd):", "kd_gain", 0.1, 0.0, 2.0),
        ]

        for idx, (label, var_name, default, min_val, max_val) in enumerate(control_params):
            ttk.Label(control_param_frame, text=label).grid(row=idx, column=0, sticky='w', pady=5)

            var = tk.DoubleVar(value=default)
            self.param_vars[var_name] = var

            slider = ttk.Scale(control_param_frame, from_=min_val, to=max_val,
                             variable=var, orient='horizontal', length=400)
            slider.grid(row=idx, column=1, sticky='ew', pady=5, padx=10)

            label_val = ttk.Label(control_param_frame, text=f"{default:.2f}")
            label_val.grid(row=idx, column=2, sticky='w', pady=5)
            self.param_labels[var_name] = label_val

    def update_parameter(self, param_name):
        """Update parameter value from slider"""
        value = self.param_vars[param_name].get()
        self.param_labels[param_name].config(text=f"{value:.2f}")

        # Update model parameters
        if hasattr(self.model, param_name):
            setattr(self.model, param_name, value)

        # Update economic parameters
        if param_name == "energy_cost":
            self.model.energy_cost = value
        elif param_name == "maintenance_cost_per_hour":
            self.model.maintenance_cost_per_hour = value
        elif param_name == "efficiency_target":
            self.model.efficiency_target = value / 100.0

    def start_simulation(self):
        """Start simulation"""
        self.simulation_running = True
        self.start_button.config(state='disabled')
        self.stop_button.config(state='normal')

        # Run simulation loop
        self.run_simulation_step()

    def stop_simulation(self):
        """Stop simulation"""
        self.simulation_running = False
        self.start_button.config(state='normal')
        self.stop_button.config(state='disabled')

    def reset_simulation(self):
        """Reset simulation"""
        self.stop_simulation()
        self.simulation_time = 0.0
        self.time_data = []
        self.voltage_data = []
        self.current_data = []
        self.torque_data = []
        self.temp_data = []
        self.efficiency_data = []
        self.total_energy_consumed = 0.0
        self.total_cost = 0.0
        self.operating_hours = 0.0

        # Reset model
        self.model = AlternatorPhysicsModel()

        # Clear plots
        self.clear_all_plots()
        self.update_displays()

    def run_simulation_step(self):
        """Run one simulation step"""
        if not self.simulation_running:
            return

        dt = 0.01  # Time step (10ms)
        self.simulation_time += dt

        # Update model from parameters
        self.update_model_from_params()

        # Run electromagnetic simulation
        y0_em = [0.0, 0.0, 0.0, self.param_vars['rotor_current'].get(),
                 self.model.calculate_angular_velocity(), 0.0]
        t_span_em = (self.simulation_time, self.simulation_time + dt)

        # Use RK4 for electromagnetic dynamics
        load_torque = 10.0  # Sample load torque
        t_em, y_em = self.solver.rk4_method(self.model.electromagnetic_ode, y0_em,
                                           t_span_em, dt/10, args=(load_torque,))

        # Extract results
        i_a, i_b, i_c = y_em[-1, 0], y_em[-1, 1], y_em[-1, 2]
        currents = [i_a, i_b, i_c]

        # Calculate voltages
        phase_voltage, _, _ = self.model.calculate_phase_voltage()
        frequency = self.model.calculate_frequency()

        # Calculate losses
        copper_loss = self.model.calculate_copper_losses(currents)
        flux_density = 1.2  # Tesla (sample value)
        iron_loss = self.model.calculate_iron_losses(frequency, flux_density)
        omega = self.model.calculate_angular_velocity()
        mechanical_loss = self.model.calculate_mechanical_losses(omega)

        # Power calculations
        power_out = phase_voltage * sum([abs(i) for i in currents]) * 0.9  # RMS power
        stray_loss = self.model.calculate_stray_losses(power_out)
        total_losses = copper_loss + iron_loss + mechanical_loss + stray_loss

        # Efficiency
        efficiency = self.model.calculate_efficiency(power_out, total_losses)

        # Thermal simulation
        power_losses = {'copper': copper_loss, 'iron': iron_loss}
        y0_thermal = [self.model.stator_temp, self.model.rotor_temp]
        t_span_thermal = (self.simulation_time, self.simulation_time + dt)

        t_th, y_th = self.solver.rk4_method(self.model.thermal_ode, y0_thermal,
                                           t_span_thermal, dt/10, args=(power_losses,))

        self.model.stator_temp, self.model.rotor_temp = y_th[-1]

        # Store data
        self.time_data.append(self.simulation_time)
        self.voltage_data.append(phase_voltage)
        self.current_data.append(sum([abs(i) for i in currents])/3)
        self.torque_data.append(y_em[-1, 4] * self.model.rotor_inertia)
        self.temp_data.append(self.model.stator_temp)
        self.efficiency_data.append(efficiency)

        # Update economic data
        power_in_kw = (power_out + total_losses) / 1000.0
        energy_consumed_kwh = power_in_kw * (dt / 3600.0)
        self.total_energy_consumed += energy_consumed_kwh
        self.operating_hours = self.simulation_time / 3600.0
        self.total_cost = (self.total_energy_consumed * self.model.energy_cost +
                          self.operating_hours * self.model.maintenance_cost_per_hour)

        # Limit data storage
        max_points = 500
        if len(self.time_data) > max_points:
            self.time_data = self.time_data[-max_points:]
            self.voltage_data = self.voltage_data[-max_points:]
            self.current_data = self.current_data[-max_points:]
            self.torque_data = self.torque_data[-max_points:]
            self.temp_data = self.temp_data[-max_points:]
            self.efficiency_data = self.efficiency_data[-max_points:]

        # Update displays
        self.update_displays()
        self.update_plots()

        # Schedule next step
        self.root.after(20, self.run_simulation_step)

    def update_model_from_params(self):
        """Update model from GUI parameters"""
        self.model.num_coils = int(self.param_vars['num_coils'].get())
        self.model.coil_voltage = self.param_vars['coil_voltage'].get()
        self.model.phase_shift = self.param_vars['phase_shift'].get()
        self.model.num_poles = int(self.param_vars['num_poles'].get())
        self.model.speed_rpm = self.param_vars['speed_rpm'].get()
        self.model.load_resistance = self.param_vars['load_resistance'].get()
        self.model.load_inductance = self.param_vars['load_inductance'].get()

    def update_displays(self):
        """Update status displays"""
        phase_voltage, _, _ = self.model.calculate_phase_voltage()
        frequency = self.model.calculate_frequency()

        # Calculate current power and efficiency
        if len(self.voltage_data) > 0 and len(self.current_data) > 0:
            power_out = self.voltage_data[-1] * self.current_data[-1] * 3
            eff = self.efficiency_data[-1] if self.efficiency_data else 0.0
        else:
            power_out = 0.0
            eff = 0.0

        # Update status labels
        self.status_labels['phase_voltage'].config(text=f"{phase_voltage:.2f} V")
        self.status_labels['frequency'].config(text=f"{frequency:.2f} Hz")
        self.status_labels['power_out'].config(text=f"{power_out:.2f} W")
        self.status_labels['efficiency'].config(text=f"{eff:.2f} %")
        self.status_labels['temperature'].config(text=f"{self.model.stator_temp:.1f} °C")

        # Update economic labels
        if hasattr(self, 'econ_labels'):
            self.econ_labels['energy_consumed'].config(text=f"{self.total_energy_consumed:.2f} kWh")
            self.econ_labels['energy_cost_total'].config(
                text=f"${self.total_energy_consumed * self.model.energy_cost:.2f}")
            self.econ_labels['operating_cost'].config(text=f"${self.total_cost:.2f}")
            self.econ_labels['operating_hours'].config(text=f"{self.operating_hours:.2f} hrs")
            cost_per_hour = self.total_cost / self.operating_hours if self.operating_hours > 0 else 0
            self.econ_labels['cost_per_hour'].config(text=f"${cost_per_hour:.2f}/hr")

    def update_plots(self):
        """Update all plots"""
        if len(self.time_data) < 2:
            return

        # Main control tab plots
        self.update_main_plots()

        # Electromagnetic tab plots
        self.update_electromagnetic_plots()

        # Thermal tab plots
        self.update_thermal_plots()

        # Mechanical tab plots
        self.update_mechanical_plots()

        # Economic tab plots
        self.update_economic_plots()

    def update_main_plots(self):
        """Update main control tab plots"""
        t = self.time_data

        # Voltage plot
        self.ax_voltage.clear()
        self.ax_voltage.plot(t, self.voltage_data, 'b-', linewidth=2)
        self.ax_voltage.set_xlabel('Time (s)')
        self.ax_voltage.set_ylabel('Voltage (V)')
        self.ax_voltage.set_title('Phase Voltage')
        self.ax_voltage.grid(True, alpha=0.3)

        # Current plot
        self.ax_current.clear()
        self.ax_current.plot(t, self.current_data, 'r-', linewidth=2)
        self.ax_current.set_xlabel('Time (s)')
        self.ax_current.set_ylabel('Current (A)')
        self.ax_current.set_title('Phase Current')
        self.ax_current.grid(True, alpha=0.3)

        # Power plot
        if len(self.voltage_data) == len(self.current_data):
            power = [v * i * 3 for v, i in zip(self.voltage_data, self.current_data)]
            self.ax_power.clear()
            self.ax_power.plot(t, power, 'g-', linewidth=2)
            self.ax_power.set_xlabel('Time (s)')
            self.ax_power.set_ylabel('Power (W)')
            self.ax_power.set_title('Output Power')
            self.ax_power.grid(True, alpha=0.3)

        # Torque plot
        self.ax_torque.clear()
        self.ax_torque.plot(t, self.torque_data, 'm-', linewidth=2)
        self.ax_torque.set_xlabel('Time (s)')
        self.ax_torque.set_ylabel('Torque (N·m)')
        self.ax_torque.set_title('Electromagnetic Torque')
        self.ax_torque.grid(True, alpha=0.3)

        # Efficiency plot
        self.ax_efficiency.clear()
        self.ax_efficiency.plot(t, self.efficiency_data, 'c-', linewidth=2)
        self.ax_efficiency.axhline(y=self.model.efficiency_target*100, color='r',
                                   linestyle='--', label='Target')
        self.ax_efficiency.set_xlabel('Time (s)')
        self.ax_efficiency.set_ylabel('Efficiency (%)')
        self.ax_efficiency.set_title('System Efficiency')
        self.ax_efficiency.grid(True, alpha=0.3)
        self.ax_efficiency.legend()

        # Temperature plot
        self.ax_temp.clear()
        self.ax_temp.plot(t, self.temp_data, 'orange', linewidth=2)
        self.ax_temp.axhline(y=100, color='r', linestyle='--', label='Warning')
        self.ax_temp.set_xlabel('Time (s)')
        self.ax_temp.set_ylabel('Temperature (°C)')
        self.ax_temp.set_title('Stator Temperature')
        self.ax_temp.grid(True, alpha=0.3)
        self.ax_temp.legend()

        self.fig_main.tight_layout(pad=2.0)
        self.canvas_main.draw()

    def update_electromagnetic_plots(self):
        """Update electromagnetic tab plots"""
        # Phasor diagram
        self.ax_phasor.clear()
        angles = np.linspace(0, 2*np.pi, 100)
        magnitudes = np.ones(100) * self.model.coil_voltage
        self.ax_phasor.plot(angles, magnitudes, 'b-', alpha=0.3)

        # Plot individual coil phasors
        for n in range(self.model.num_coils):
            angle = np.radians(n * self.model.phase_shift)
            self.ax_phasor.plot([0, angle], [0, self.model.coil_voltage], 'r-', linewidth=2)

        self.ax_phasor.set_title('Coil Voltage Phasors')

        # Flux density plot
        self.ax_flux.clear()
        if len(self.time_data) > 0:
            freq = self.model.calculate_frequency()
            flux = [1.2 * np.sin(2 * np.pi * freq * t) for t in self.time_data]
            self.ax_flux.plot(self.time_data, flux, 'b-', linewidth=2)
            self.ax_flux.set_xlabel('Time (s)')
            self.ax_flux.set_ylabel('Flux Density (T)')
            self.ax_flux.set_title('Magnetic Flux Density')
            self.ax_flux.grid(True, alpha=0.3)

        # EMF waveform
        self.ax_emf.clear()
        if len(self.time_data) > 0:
            phase_voltage, _, _ = self.model.calculate_phase_voltage()
            freq = self.model.calculate_frequency()
            emf = [phase_voltage * np.sqrt(2) * np.sin(2 * np.pi * freq * t)
                   for t in self.time_data]
            self.ax_emf.plot(self.time_data, emf, 'g-', linewidth=2)
            self.ax_emf.set_xlabel('Time (s)')
            self.ax_emf.set_ylabel('EMF (V)')
            self.ax_emf.set_title('Induced EMF Waveform')
            self.ax_emf.grid(True, alpha=0.3)

        # Loss breakdown
        self.ax_losses_em.clear()
        if len(self.current_data) > 0:
            copper = self.model.calculate_copper_losses([self.current_data[-1]]*3)
            iron = self.model.calculate_iron_losses(self.model.calculate_frequency(), 1.2)
            losses = ['Copper', 'Iron Core']
            values = [copper, iron]
            colors = ['#ff9999', '#66b3ff']
            self.ax_losses_em.bar(losses, values, color=colors)
            self.ax_losses_em.set_ylabel('Loss (W)')
            self.ax_losses_em.set_title('Electromagnetic Losses')
            self.ax_losses_em.grid(True, alpha=0.3, axis='y')

        self.fig_em.tight_layout(pad=2.0)
        self.canvas_main.draw()

    def update_thermal_plots(self):
        """Update thermal tab plots"""
        # Stator temperature
        self.ax_temp_stator.clear()
        if len(self.time_data) > 0:
            self.ax_temp_stator.plot(self.time_data, self.temp_data, 'r-', linewidth=2)
            self.ax_temp_stator.axhline(y=120, color='orange', linestyle='--', label='Derating')
            self.ax_temp_stator.axhline(y=155, color='red', linestyle='--', label='Max')
            self.ax_temp_stator.set_xlabel('Time (s)')
            self.ax_temp_stator.set_ylabel('Temperature (°C)')
            self.ax_temp_stator.set_title('Stator Temperature Rise')
            self.ax_temp_stator.grid(True, alpha=0.3)
            self.ax_temp_stator.legend()

        # Rotor temperature (sample data)
        self.ax_temp_rotor.clear()
        if len(self.time_data) > 0:
            rotor_temp = [self.model.ambient_temp + (t - self.model.ambient_temp) * 0.8
                         for t in self.temp_data]
            self.ax_temp_rotor.plot(self.time_data, rotor_temp, 'b-', linewidth=2)
            self.ax_temp_rotor.axhline(y=130, color='orange', linestyle='--', label='Derating')
            self.ax_temp_rotor.set_xlabel('Time (s)')
            self.ax_temp_rotor.set_ylabel('Temperature (°C)')
            self.ax_temp_rotor.set_title('Rotor Temperature Rise')
            self.ax_temp_rotor.grid(True, alpha=0.3)
            self.ax_temp_rotor.legend()

        # Heat flow
        self.ax_heat_flow.clear()
        if len(self.time_data) > 0 and len(self.current_data) > 0:
            heat_gen = [self.model.calculate_copper_losses([i]*3) for i in self.current_data]
            heat_diss = [(t - self.model.ambient_temp) / self.model.thermal_resistance_stator
                        for t in self.temp_data]
            self.ax_heat_flow.plot(self.time_data, heat_gen, 'r-', linewidth=2, label='Generated')
            self.ax_heat_flow.plot(self.time_data, heat_diss, 'b-', linewidth=2, label='Dissipated')
            self.ax_heat_flow.set_xlabel('Time (s)')
            self.ax_heat_flow.set_ylabel('Heat Flow (W)')
            self.ax_heat_flow.set_title('Heat Generation vs Dissipation')
            self.ax_heat_flow.grid(True, alpha=0.3)
            self.ax_heat_flow.legend()

        # Derating curve
        self.ax_derating.clear()
        temp_range = np.linspace(25, 155, 100)
        derating_factor = np.where(temp_range < 120, 1.0,
                                   1.0 - (temp_range - 120) / 35 * 0.2)
        self.ax_derating.plot(temp_range, derating_factor * 100, 'g-', linewidth=2)
        if len(self.temp_data) > 0:
            current_temp = self.temp_data[-1]
            current_derating = 100.0 if current_temp < 120 else \
                              100.0 - (current_temp - 120) / 35 * 20
            self.ax_derating.plot(current_temp, current_derating, 'ro', markersize=10)
        self.ax_derating.set_xlabel('Temperature (°C)')
        self.ax_derating.set_ylabel('Power Rating (%)')
        self.ax_derating.set_title('Thermal Derating Curve')
        self.ax_derating.grid(True, alpha=0.3)

        self.fig_thermal.tight_layout(pad=2.0)
        self.canvas_main.draw()

    def update_mechanical_plots(self):
        """Update mechanical tab plots"""
        # Speed
        self.ax_speed.clear()
        if len(self.time_data) > 0:
            speed = [self.model.speed_rpm] * len(self.time_data)
            self.ax_speed.plot(self.time_data, speed, 'b-', linewidth=2)
            self.ax_speed.set_xlabel('Time (s)')
            self.ax_speed.set_ylabel('Speed (RPM)')
            self.ax_speed.set_title('Rotor Speed')
            self.ax_speed.grid(True, alpha=0.3)

        # Torque
        self.ax_torque_mech.clear()
        if len(self.time_data) > 0:
            self.ax_torque_mech.plot(self.time_data, self.torque_data, 'r-', linewidth=2)
            self.ax_torque_mech.set_xlabel('Time (s)')
            self.ax_torque_mech.set_ylabel('Torque (N·m)')
            self.ax_torque_mech.set_title('Shaft Torque')
            self.ax_torque_mech.grid(True, alpha=0.3)

        # Shaft stress
        self.ax_shaft_stress.clear()
        if len(self.torque_data) > 0:
            # Assume shaft diameter of 50mm
            shaft_radius = 0.025  # m
            shaft_stress = [t / (np.pi * shaft_radius**3 / 2) / 1e6
                           for t in self.torque_data]  # MPa
            self.ax_shaft_stress.plot(self.time_data, shaft_stress, 'g-', linewidth=2)
            self.ax_shaft_stress.axhline(y=250, color='r', linestyle='--', label='Yield')
            self.ax_shaft_stress.set_xlabel('Time (s)')
            self.ax_shaft_stress.set_ylabel('Shear Stress (MPa)')
            self.ax_shaft_stress.set_title('Shaft Stress Analysis')
            self.ax_shaft_stress.grid(True, alpha=0.3)
            self.ax_shaft_stress.legend()

        # Bearing loads
        self.ax_bearing_loads.clear()
        if len(self.torque_data) > 0:
            # Sample bearing load calculation
            radial_load = [abs(t) * 0.5 for t in self.torque_data]
            axial_load = [abs(t) * 0.1 for t in self.torque_data]
            self.ax_bearing_loads.plot(self.time_data, radial_load, 'b-',
                                       linewidth=2, label='Radial')
            self.ax_bearing_loads.plot(self.time_data, axial_load, 'r-',
                                      linewidth=2, label='Axial')
            self.ax_bearing_loads.set_xlabel('Time (s)')
            self.ax_bearing_loads.set_ylabel('Load (N)')
            self.ax_bearing_loads.set_title('Bearing Loads')
            self.ax_bearing_loads.grid(True, alpha=0.3)
            self.ax_bearing_loads.legend()

        self.fig_mech.tight_layout(pad=2.0)
        self.canvas_main.draw()

    def update_economic_plots(self):
        """Update economic analysis plots"""
        # Energy cost over time
        self.ax_energy_cost.clear()
        if len(self.time_data) > 0:
            time_hours = [t / 3600 for t in self.time_data]
            energy_cost = [self.total_energy_consumed * self.model.energy_cost
                          * (i+1)/len(time_hours) for i in range(len(time_hours))]
            self.ax_energy_cost.plot(time_hours, energy_cost, 'g-', linewidth=2)
            self.ax_energy_cost.set_xlabel('Time (hours)')
            self.ax_energy_cost.set_ylabel('Cost ($)')
            self.ax_energy_cost.set_title('Cumulative Energy Cost')
            self.ax_energy_cost.grid(True, alpha=0.3)

        # Loss breakdown pie chart
        self.ax_loss_breakdown.clear()
        if len(self.current_data) > 0:
            copper = self.model.calculate_copper_losses([self.current_data[-1]]*3)
            iron = self.model.calculate_iron_losses(self.model.calculate_frequency(), 1.2)
            mech = self.model.calculate_mechanical_losses(self.model.calculate_angular_velocity())
            power_out = self.voltage_data[-1] * self.current_data[-1] * 3 if self.voltage_data else 1
            stray = self.model.calculate_stray_losses(power_out)

            labels = ['Copper', 'Iron', 'Mechanical', 'Stray']
            sizes = [copper, iron, mech, stray]
            colors = ['#ff9999', '#66b3ff', '#99ff99', '#ffcc99']

            self.ax_loss_breakdown.pie(sizes, labels=labels, colors=colors, autopct='%1.1f%%',
                                       startangle=90)
            self.ax_loss_breakdown.set_title('Loss Distribution')

        # Operating cost breakdown
        self.ax_operating_cost.clear()
        if self.operating_hours > 0:
            energy_cost = self.total_energy_consumed * self.model.energy_cost
            maint_cost = self.operating_hours * self.model.maintenance_cost_per_hour

            categories = ['Energy', 'Maintenance']
            costs = [energy_cost, maint_cost]
            colors = ['#ff9999', '#66b3ff']

            self.ax_operating_cost.bar(categories, costs, color=colors)
            self.ax_operating_cost.set_ylabel('Cost ($)')
            self.ax_operating_cost.set_title('Operating Cost Breakdown')
            self.ax_operating_cost.grid(True, alpha=0.3, axis='y')

        self.fig_econ.tight_layout(pad=2.0)
        self.canvas_main.draw()

    def clear_all_plots(self):
        """Clear all plots"""
        for ax in [self.ax_voltage, self.ax_current, self.ax_power,
                   self.ax_torque, self.ax_efficiency, self.ax_temp]:
            ax.clear()
        self.canvas_main.draw()

    def on_resize(self, event):
        """Handle window resize event"""
        # Auto-scale figures
        if hasattr(self, 'fig_main'):
            self.fig_main.tight_layout(pad=2.0)
            self.canvas_main.draw()

    def show_phase_voltage_calculation(self):
        """Show phase voltage calculation dialog"""
        phase_voltage, real_sum, imag_sum = self.model.calculate_phase_voltage()

        msg = f"Phase Voltage Calculation\n\n"
        msg += f"Number of coils: {self.model.num_coils}\n"
        msg += f"Voltage per coil: {self.model.coil_voltage} V (RMS)\n"
        msg += f"Phase shift: {self.model.phase_shift}°\n\n"
        msg += f"Phasor Addition:\n"
        msg += f"Real component: {real_sum:.3f} V\n"
        msg += f"Imaginary component: {imag_sum:.3f} V\n\n"
        msg += f"Total RMS Phase Voltage: {phase_voltage:.3f} V"

        messagebox.showinfo("Phase Voltage Calculation", msg)

    def show_frequency_calculation(self):
        """Show frequency calculation dialog"""
        frequency = self.model.calculate_frequency()

        msg = f"Frequency Calculation\n\n"
        msg += f"Number of poles (P): {self.model.num_poles}\n"
        msg += f"Speed (N): {self.model.speed_rpm} RPM\n\n"
        msg += f"Formula: f = (P × N) / 120\n"
        msg += f"f = ({self.model.num_poles} × {self.model.speed_rpm}) / 120\n\n"
        msg += f"Electrical Frequency: {frequency:.3f} Hz"

        messagebox.showinfo("Frequency Calculation", msg)

    def export_data(self):
        """Export simulation data"""
        if len(self.time_data) == 0:
            messagebox.showwarning("No Data", "No simulation data to export")
            return

        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f"alternator_data_{timestamp}.csv"

        try:
            with open(filename, 'w') as f:
                f.write("Time(s),Voltage(V),Current(A),Torque(Nm),Temperature(C),Efficiency(%)\n")
                for i in range(len(self.time_data)):
                    f.write(f"{self.time_data[i]:.4f},{self.voltage_data[i]:.4f},")
                    f.write(f"{self.current_data[i]:.4f},{self.torque_data[i]:.4f},")
                    f.write(f"{self.temp_data[i]:.4f},{self.efficiency_data[i]:.4f}\n")

            messagebox.showinfo("Export Success", f"Data exported to {filename}")
        except Exception as e:
            messagebox.showerror("Export Error", f"Failed to export data: {str(e)}")

    def show_about(self):
        """Show about dialog"""
        msg = "Advanced 3-Phase Alternator Simulation\n\n"
        msg += "A comprehensive multi-physics simulation tool for\n"
        msg += "electrical engineering analysis including:\n\n"
        msg += "• Electromagnetic modeling\n"
        msg += "• Thermal analysis\n"
        msg += "• Mechanical stress analysis\n"
        msg += "• Economic evaluation\n"
        msg += "• Advanced control systems\n\n"
        msg += "Developed for practical electrical engineering applications"

        messagebox.showinfo("About", msg)


def main():
    """Main application entry point"""
    root = tk.Tk()
    app = AlternatorGUI(root)
    root.mainloop()


if __name__ == "__main__":
    main()
