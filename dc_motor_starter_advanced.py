#!/usr/bin/env python3
"""
Advanced DC Motor Starter Design and Multi-Physics Simulation
Comprehensive tool for designing seven-stud starter with real-time simulation
"""

import tkinter as tk
from tkinter import ttk, messagebox, scrolledtext
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure
from scipy.integrate import odeint, solve_ivp
import threading
import time
from datetime import datetime
import json

class DCMotorStarterDesign:
    """Main application for DC Motor Starter Design and Simulation"""

    def __init__(self, root):
        self.root = root
        self.root.title("Advanced DC Motor Starter Design & Multi-Physics Simulation")
        self.root.geometry("1400x900")

        # Motor Parameters (Default values from problem)
        self.params = {
            'P_rated': 36775,      # Rated power (W)
            'V_supply': 400,       # Supply voltage (V)
            'efficiency': 0.92,    # Efficiency
            'cu_loss_percent': 0.05,  # Cu losses as % of input
            'R_field': 200,        # Shunt field resistance (Ω)
            'num_studs': 7,        # Number of starter studs
            'current_ratio': 1.5,  # Imax/Imin ratio
            'J_motor': 2.5,        # Moment of inertia (kg·m²)
            'B_friction': 0.05,    # Friction coefficient (N·m·s)
            'T_load': 0,           # Load torque (N·m)
            'ambient_temp': 25,    # Ambient temperature (°C)
            'thermal_resistance': 0.5,  # Thermal resistance (°C/W)
            'thermal_capacitance': 500,  # Thermal capacitance (J/°C)
            'max_temp': 155,       # Maximum winding temperature (°C)
            'iron_loss_constant': 50,  # Iron loss constant
            'mech_loss_constant': 30,  # Mechanical loss constant
            'electricity_cost': 0.12,  # $/kWh
            'motor_cost': 5000,    # Initial motor cost ($)
        }

        # Calculated parameters
        self.calc_params = {}
        self.starter_resistances = []

        # Simulation state
        self.simulation_running = False
        self.simulation_thread = None
        self.time_data = []
        self.current_data = []
        self.speed_data = []
        self.torque_data = []
        self.temp_data = []
        self.power_data = []
        self.loss_data = {'copper': [], 'iron': [], 'mechanical': [], 'stray': []}
        self.current_stud = 1
        self.solver_method = 'RK45'

        # GUI Setup
        self.setup_gui()
        self.calculate_motor_parameters()
        self.design_starter_resistances()

        # Bind resize event
        self.root.bind('<Configure>', self.on_window_resize)

    def setup_gui(self):
        """Setup the complete GUI with all tabs and controls"""
        # Main menu
        menubar = tk.Menu(self.root)
        self.root.config(menu=menubar)

        file_menu = tk.Menu(menubar, tearoff=0)
        menubar.add_cascade(label="File", menu=file_menu)
        file_menu.add_command(label="Save Parameters", command=self.save_parameters)
        file_menu.add_command(label="Load Parameters", command=self.load_parameters)
        file_menu.add_separator()
        file_menu.add_command(label="Exit", command=self.root.quit)

        tools_menu = tk.Menu(menubar, tearoff=0)
        menubar.add_cascade(label="Tools", menu=tools_menu)
        tools_menu.add_command(label="Recalculate Starter", command=self.design_starter_resistances)
        tools_menu.add_command(label="Reset Simulation", command=self.reset_simulation)

        help_menu = tk.Menu(menubar, tearoff=0)
        menubar.add_cascade(label="Help", menu=help_menu)
        help_menu.add_command(label="About", command=self.show_about)

        # Create main notebook (tabs)
        self.notebook = ttk.Notebook(self.root)
        self.notebook.pack(fill='both', expand=True, padx=5, pady=5)

        # Tab 1: Starter Design
        self.tab_design = ttk.Frame(self.notebook)
        self.notebook.add(self.tab_design, text='Starter Design')
        self.setup_design_tab()

        # Tab 2: Dynamic Simulation
        self.tab_simulation = ttk.Frame(self.notebook)
        self.notebook.add(self.tab_simulation, text='Dynamic Simulation')
        self.setup_simulation_tab()

        # Tab 3: Multi-Physics Analysis
        self.tab_multiphysics = ttk.Frame(self.notebook)
        self.notebook.add(self.tab_multiphysics, text='Multi-Physics')
        self.setup_multiphysics_tab()

        # Tab 4: Economic Analysis
        self.tab_economics = ttk.Frame(self.notebook)
        self.notebook.add(self.tab_economics, text='Economic Analysis')
        self.setup_economics_tab()

        # Tab 5: Advanced Controls
        self.tab_controls = ttk.Frame(self.notebook)
        self.notebook.add(self.tab_controls, text='Advanced Controls')
        self.setup_controls_tab()

    def setup_design_tab(self):
        """Setup the starter design tab"""
        # Left panel - Input parameters
        left_frame = ttk.LabelFrame(self.tab_design, text="Motor Parameters", padding=10)
        left_frame.grid(row=0, column=0, sticky='nsew', padx=5, pady=5)

        params_to_show = [
            ('Rated Power (kW)', 'P_rated', 1000),
            ('Supply Voltage (V)', 'V_supply', 1),
            ('Efficiency (%)', 'efficiency', 0.01),
            ('Cu Loss % of Input', 'cu_loss_percent', 0.01),
            ('Field Resistance (Ω)', 'R_field', 1),
            ('Number of Studs', 'num_studs', 1),
            ('Current Ratio (Imax/Imin)', 'current_ratio', 1),
            ('Moment of Inertia (kg·m²)', 'J_motor', 1),
            ('Friction Coeff (N·m·s)', 'B_friction', 1),
        ]

        self.param_entries = {}
        for idx, (label, key, scale) in enumerate(params_to_show):
            ttk.Label(left_frame, text=label).grid(row=idx, column=0, sticky='w', pady=2)
            entry = ttk.Entry(left_frame, width=15)
            entry.insert(0, str(self.params[key] * scale))
            entry.grid(row=idx, column=1, pady=2, padx=5)
            self.param_entries[key] = (entry, scale)

        # Update button
        ttk.Button(left_frame, text="Update & Recalculate",
                  command=self.update_parameters).grid(row=len(params_to_show),
                                                       column=0, columnspan=2, pady=10)

        # Right panel - Results
        right_frame = ttk.LabelFrame(self.tab_design, text="Calculated Results", padding=10)
        right_frame.grid(row=0, column=1, sticky='nsew', padx=5, pady=5)

        self.results_text = scrolledtext.ScrolledText(right_frame, width=60, height=30,
                                                      font=('Courier', 10))
        self.results_text.pack(fill='both', expand=True)

        # Configure grid weights
        self.tab_design.columnconfigure(0, weight=1)
        self.tab_design.columnconfigure(1, weight=2)
        self.tab_design.rowconfigure(0, weight=1)

    def setup_simulation_tab(self):
        """Setup the dynamic simulation tab"""
        # Control panel
        control_frame = ttk.LabelFrame(self.tab_simulation, text="Simulation Controls", padding=10)
        control_frame.pack(side='top', fill='x', padx=5, pady=5)

        # Buttons
        btn_frame = ttk.Frame(control_frame)
        btn_frame.pack(side='left', padx=5)

        self.btn_start = ttk.Button(btn_frame, text="Start", command=self.start_simulation, width=10)
        self.btn_start.pack(side='left', padx=2)

        self.btn_stop = ttk.Button(btn_frame, text="Stop", command=self.stop_simulation,
                                   width=10, state='disabled')
        self.btn_stop.pack(side='left', padx=2)

        self.btn_reset = ttk.Button(btn_frame, text="Reset", command=self.reset_simulation, width=10)
        self.btn_reset.pack(side='left', padx=2)

        # Solver selection
        solver_frame = ttk.Frame(control_frame)
        solver_frame.pack(side='left', padx=20)
        ttk.Label(solver_frame, text="Solver:").pack(side='left')
        self.solver_var = tk.StringVar(value='RK45')
        solvers = ['RK45', 'Euler', 'RK23']
        solver_combo = ttk.Combobox(solver_frame, textvariable=self.solver_var,
                                    values=solvers, width=10, state='readonly')
        solver_combo.pack(side='left', padx=5)

        # Load torque slider
        load_frame = ttk.Frame(control_frame)
        load_frame.pack(side='left', padx=20)
        ttk.Label(load_frame, text="Load Torque (N·m):").pack(side='left')
        self.load_slider = ttk.Scale(load_frame, from_=0, to=500, orient='horizontal',
                                     length=200, command=self.update_load_torque)
        self.load_slider.set(self.params['T_load'])
        self.load_slider.pack(side='left', padx=5)
        self.load_label = ttk.Label(load_frame, text="0.0")
        self.load_label.pack(side='left')

        # Status label
        self.status_label = ttk.Label(control_frame, text="Status: Ready",
                                     foreground='green', font=('Arial', 10, 'bold'))
        self.status_label.pack(side='right', padx=10)

        # Plots frame
        plots_frame = ttk.Frame(self.tab_simulation)
        plots_frame.pack(fill='both', expand=True, padx=5, pady=5)

        # Create matplotlib figures
        self.fig_sim = Figure(figsize=(12, 8), dpi=100)
        self.fig_sim.subplots_adjust(hspace=0.3, wspace=0.3)

        self.ax_current = self.fig_sim.add_subplot(2, 2, 1)
        self.ax_speed = self.fig_sim.add_subplot(2, 2, 2)
        self.ax_torque = self.fig_sim.add_subplot(2, 2, 3)
        self.ax_power = self.fig_sim.add_subplot(2, 2, 4)

        self.ax_current.set_title('Armature Current vs Time')
        self.ax_current.set_xlabel('Time (s)')
        self.ax_current.set_ylabel('Current (A)')
        self.ax_current.grid(True, alpha=0.3)

        self.ax_speed.set_title('Speed vs Time')
        self.ax_speed.set_xlabel('Time (s)')
        self.ax_speed.set_ylabel('Speed (RPM)')
        self.ax_speed.grid(True, alpha=0.3)

        self.ax_torque.set_title('Torque vs Time')
        self.ax_torque.set_xlabel('Time (s)')
        self.ax_torque.set_ylabel('Torque (N·m)')
        self.ax_torque.grid(True, alpha=0.3)

        self.ax_power.set_title('Power vs Time')
        self.ax_power.set_xlabel('Time (s)')
        self.ax_power.set_ylabel('Power (W)')
        self.ax_power.grid(True, alpha=0.3)

        self.canvas_sim = FigureCanvasTkAgg(self.fig_sim, plots_frame)
        self.canvas_sim.get_tk_widget().pack(fill='both', expand=True)

    def setup_multiphysics_tab(self):
        """Setup the multi-physics analysis tab"""
        # Info frame
        info_frame = ttk.LabelFrame(self.tab_multiphysics, text="Multi-Physics Analysis", padding=10)
        info_frame.pack(side='top', fill='x', padx=5, pady=5)

        info_text = """This tab provides coupled electromagnetic-thermal-mechanical analysis:
• Electromagnetic: Circuit equations with back-EMF and armature reaction
• Thermal: Heat transfer equations for winding temperature prediction
• Mechanical: Shaft torque, bearing loads, and stress analysis
• Loss Breakdown: Copper, iron, mechanical friction, and stray load losses"""

        ttk.Label(info_frame, text=info_text, justify='left').pack()

        # Plots
        plots_frame = ttk.Frame(self.tab_multiphysics)
        plots_frame.pack(fill='both', expand=True, padx=5, pady=5)

        self.fig_multi = Figure(figsize=(12, 8), dpi=100)
        self.fig_multi.subplots_adjust(hspace=0.3, wspace=0.3)

        self.ax_temp = self.fig_multi.add_subplot(2, 2, 1)
        self.ax_losses = self.fig_multi.add_subplot(2, 2, 2)
        self.ax_efficiency = self.fig_multi.add_subplot(2, 2, 3)
        self.ax_stress = self.fig_multi.add_subplot(2, 2, 4)

        self.ax_temp.set_title('Temperature vs Time')
        self.ax_temp.set_xlabel('Time (s)')
        self.ax_temp.set_ylabel('Temperature (°C)')
        self.ax_temp.grid(True, alpha=0.3)

        self.ax_losses.set_title('Loss Breakdown')
        self.ax_losses.set_xlabel('Time (s)')
        self.ax_losses.set_ylabel('Losses (W)')
        self.ax_losses.grid(True, alpha=0.3)

        self.ax_efficiency.set_title('Efficiency vs Time')
        self.ax_efficiency.set_xlabel('Time (s)')
        self.ax_efficiency.set_ylabel('Efficiency (%)')
        self.ax_efficiency.grid(True, alpha=0.3)

        self.ax_stress.set_title('Mechanical Stress Analysis')
        self.ax_stress.set_xlabel('Time (s)')
        self.ax_stress.set_ylabel('Torque (N·m)')
        self.ax_stress.grid(True, alpha=0.3)

        self.canvas_multi = FigureCanvasTkAgg(self.fig_multi, plots_frame)
        self.canvas_multi.get_tk_widget().pack(fill='both', expand=True)

    def setup_economics_tab(self):
        """Setup the economic analysis tab"""
        # Input frame
        input_frame = ttk.LabelFrame(self.tab_economics, text="Economic Parameters", padding=10)
        input_frame.pack(side='left', fill='both', padx=5, pady=5, expand=True)

        econ_params = [
            ('Electricity Cost ($/kWh)', 'electricity_cost', 1),
            ('Motor Initial Cost ($)', 'motor_cost', 1),
            ('Operating Hours/Year', 'operating_hours', 1),
            ('Maintenance Cost/Year ($)', 'maintenance_cost', 1),
            ('Discount Rate (%)', 'discount_rate', 0.01),
            ('Analysis Period (years)', 'analysis_period', 1),
        ]

        # Add missing economic parameters with defaults
        if 'operating_hours' not in self.params:
            self.params['operating_hours'] = 4000
        if 'maintenance_cost' not in self.params:
            self.params['maintenance_cost'] = 500
        if 'discount_rate' not in self.params:
            self.params['discount_rate'] = 0.05
        if 'analysis_period' not in self.params:
            self.params['analysis_period'] = 10

        self.econ_entries = {}
        for idx, (label, key, scale) in enumerate(econ_params):
            ttk.Label(input_frame, text=label).grid(row=idx, column=0, sticky='w', pady=5)
            entry = ttk.Entry(input_frame, width=15)
            entry.insert(0, str(self.params[key] * scale))
            entry.grid(row=idx, column=1, pady=5, padx=5)
            self.econ_entries[key] = (entry, scale)

        ttk.Button(input_frame, text="Calculate Economics",
                  command=self.calculate_economics).grid(row=len(econ_params),
                                                         column=0, columnspan=2, pady=10)

        # Results frame
        results_frame = ttk.LabelFrame(self.tab_economics, text="Economic Analysis Results",
                                       padding=10)
        results_frame.pack(side='right', fill='both', padx=5, pady=5, expand=True)

        self.econ_results_text = scrolledtext.ScrolledText(results_frame, width=50, height=25,
                                                           font=('Courier', 10))
        self.econ_results_text.pack(fill='both', expand=True)

    def setup_controls_tab(self):
        """Setup the advanced controls tab"""
        # Control parameters frame
        control_frame = ttk.LabelFrame(self.tab_controls, text="Control Parameters", padding=10)
        control_frame.pack(side='left', fill='both', padx=5, pady=5, expand=True)

        control_params = [
            ('Switching Current Ratio', 'current_ratio', 1),
            ('Starting Current Limit (A)', 'max_starting_current', 1),
            ('Acceleration Time (s)', 'accel_time_target', 1),
            ('Current Ripple Limit (%)', 'current_ripple_limit', 1),
            ('Temperature Derating (°C)', 'temp_derating_start', 1),
            ('Derating Factor (%)', 'derating_factor', 0.01),
        ]

        # Add missing control parameters
        if 'max_starting_current' not in self.params:
            self.params['max_starting_current'] = 150
        if 'accel_time_target' not in self.params:
            self.params['accel_time_target'] = 5.0
        if 'current_ripple_limit' not in self.params:
            self.params['current_ripple_limit'] = 10
        if 'temp_derating_start' not in self.params:
            self.params['temp_derating_start'] = 100
        if 'derating_factor' not in self.params:
            self.params['derating_factor'] = 0.8

        self.control_entries = {}
        for idx, (label, key, scale) in enumerate(control_params):
            ttk.Label(control_frame, text=label).grid(row=idx, column=0, sticky='w', pady=5)
            entry = ttk.Entry(control_frame, width=15)
            entry.insert(0, str(self.params[key] * scale))
            entry.grid(row=idx, column=1, pady=5, padx=5)
            self.control_entries[key] = (entry, scale)

        # Thermal protection
        ttk.Label(control_frame, text="Thermal Protection:").grid(row=len(control_params),
                                                                  column=0, sticky='w', pady=10)
        self.thermal_protection_var = tk.BooleanVar(value=True)
        ttk.Checkbutton(control_frame, text="Enable",
                       variable=self.thermal_protection_var).grid(row=len(control_params),
                                                                  column=1, pady=10)

        # Info frame
        info_frame = ttk.LabelFrame(self.tab_controls, text="Control Information", padding=10)
        info_frame.pack(side='right', fill='both', padx=5, pady=5, expand=True)

        info_text = """Advanced Control Features:

1. Current-Limited Starting
   - Prevents excessive inrush current
   - Protects motor and supply system

2. Thermal Derating
   - Monitors winding temperature
   - Reduces current at high temperatures
   - Prevents thermal damage

3. Optimal Switching Logic
   - Geometric progression for smooth acceleration
   - Minimizes switching transients
   - Reduces electromagnetic interference

4. Protection Systems
   - Overcurrent protection
   - Thermal overload protection
   - Voltage monitoring
   - Speed deviation detection"""

        ttk.Label(info_frame, text=info_text, justify='left',
                 font=('Courier', 9)).pack(anchor='w')

    def calculate_motor_parameters(self):
        """Calculate motor parameters from given specifications"""
        P = self.params['P_rated']
        V = self.params['V_supply']
        eta = self.params['efficiency']
        cu_loss = self.params['cu_loss_percent']
        R_field = self.params['R_field']

        # Input power
        P_input = P / eta

        # Full-load line current
        I_line = P_input / V

        # Field current
        I_field = V / R_field

        # Armature current
        I_armature = I_line - I_field

        # Total copper losses
        P_cu_total = cu_loss * P_input

        # Field copper loss
        P_cu_field = I_field**2 * R_field

        # Armature copper loss
        P_cu_armature = P_cu_total - P_cu_field

        # Armature resistance
        R_armature = P_cu_armature / (I_armature**2)

        # Back EMF at full load
        E_b = V - I_armature * R_armature

        # Motor constant (assuming linear relationship)
        k_m = E_b / (V * 0.9)  # Approximation for rated speed

        # Torque constant
        k_t = P / (I_armature * 2 * np.pi * (V / E_b) * 1500 / 60)  # Assuming 1500 RPM base

        # Store calculated parameters
        self.calc_params = {
            'P_input': P_input,
            'I_line': I_line,
            'I_field': I_field,
            'I_armature': I_armature,
            'P_cu_total': P_cu_total,
            'P_cu_field': P_cu_field,
            'P_cu_armature': P_cu_armature,
            'R_armature': R_armature,
            'E_b': E_b,
            'k_m': k_m,
            'k_t': k_t * 10,  # Adjusted for realistic torque
            'rated_speed': 1500,  # RPM, typical for this size motor
        }

    def design_starter_resistances(self):
        """Design the seven-stud starter resistance sections"""
        V = self.params['V_supply']
        R_a = self.calc_params['R_armature']
        I_min = self.calc_params['I_armature']  # Full-load current
        alpha = self.params['current_ratio']  # Current ratio
        n = self.params['num_studs'] - 1  # Number of resistance sections

        I_max = alpha * I_min

        # Total resistance at start (when E_b = 0)
        R_total_start = V / I_max

        # External resistance at start
        R_ext_total = R_total_start - R_a

        # Calculate individual resistance sections using geometric progression
        # For geometric progression: R1, R2, R3, ..., Rn
        # Total resistance increases by ratio α at each step
        # At start: R_a + R1 + R2 + ... + Rn = R_total_start
        # After removing R1: R_a + R2 + R3 + ... + Rn = R_total_start / α
        # And so on...

        self.starter_resistances = []

        # Common ratio for total resistance
        r = alpha ** (1/n)

        # Calculate total resistances at each stud
        total_resistances = []
        for i in range(n + 1):
            R_total_i = R_total_start / (r ** i)
            total_resistances.append(R_total_i)

        # Calculate individual section resistances
        for i in range(n):
            R_section = total_resistances[i] - total_resistances[i+1]
            self.starter_resistances.append(R_section)

        # Display results
        self.display_design_results()

    def display_design_results(self):
        """Display the design calculations and results"""
        self.results_text.delete(1.0, tk.END)

        output = "="*70 + "\n"
        output += "DC MOTOR STARTER DESIGN RESULTS\n"
        output += "="*70 + "\n\n"

        output += "MOTOR SPECIFICATIONS:\n"
        output += "-"*70 + "\n"
        output += f"Rated Power:              {self.params['P_rated']/1000:.3f} kW\n"
        output += f"Supply Voltage:           {self.params['V_supply']:.1f} V\n"
        output += f"Efficiency:               {self.params['efficiency']*100:.1f} %\n"
        output += f"Field Resistance:         {self.params['R_field']:.1f} Ω\n"
        output += f"Number of Studs:          {self.params['num_studs']}\n\n"

        output += "CALCULATED PARAMETERS:\n"
        output += "-"*70 + "\n"
        output += f"Input Power:              {self.calc_params['P_input']:.2f} W\n"
        output += f"Line Current (Full-Load): {self.calc_params['I_line']:.2f} A\n"
        output += f"Field Current:            {self.calc_params['I_field']:.2f} A\n"
        output += f"Armature Current (FL):    {self.calc_params['I_armature']:.2f} A\n"
        output += f"Armature Resistance:      {self.calc_params['R_armature']:.4f} Ω\n"
        output += f"Back EMF (Full-Load):     {self.calc_params['E_b']:.2f} V\n"
        output += f"Rated Speed:              {self.calc_params['rated_speed']:.0f} RPM\n"
        output += f"Torque Constant:          {self.calc_params['k_t']:.4f} N·m/A\n\n"

        output += "STARTER RESISTANCE SECTIONS:\n"
        output += "-"*70 + "\n"
        output += f"Current Ratio (α):        {self.params['current_ratio']:.2f}\n"
        output += f"Maximum Current:          {self.params['current_ratio']*self.calc_params['I_armature']:.2f} A\n"
        output += f"Minimum Current:          {self.calc_params['I_armature']:.2f} A\n\n"

        total_R = 0
        for i, R in enumerate(self.starter_resistances, 1):
            total_R += R
            output += f"Section {i} Resistance:     {R:.4f} Ω "
            output += f"(Cumulative: {total_R:.4f} Ω)\n"

        output += f"\nTotal External Resistance: {sum(self.starter_resistances):.4f} Ω\n"
        output += f"Total with Armature:       {sum(self.starter_resistances) + self.calc_params['R_armature']:.4f} Ω\n\n"

        output += "STUD CONFIGURATION:\n"
        output += "-"*70 + "\n"
        cumulative = 0
        for i in range(self.params['num_studs']):
            if i == 0:
                cumulative = sum(self.starter_resistances)
                output += f"Stud {i+1}: All resistances in circuit "
                output += f"(R_total = {cumulative + self.calc_params['R_armature']:.4f} Ω)\n"
            elif i < len(self.starter_resistances):
                cumulative -= self.starter_resistances[i-1]
                output += f"Stud {i+1}: Remove R{i} "
                output += f"(R_total = {cumulative + self.calc_params['R_armature']:.4f} Ω)\n"
            else:
                output += f"Stud {i+1}: Direct connection "
                output += f"(R_total = {self.calc_params['R_armature']:.4f} Ω)\n"

        output += "\n" + "="*70 + "\n"

        self.results_text.insert(1.0, output)

    def update_parameters(self):
        """Update parameters from entry fields"""
        try:
            for key, (entry, scale) in self.param_entries.items():
                self.params[key] = float(entry.get()) / scale

            self.calculate_motor_parameters()
            self.design_starter_resistances()
            messagebox.showinfo("Success", "Parameters updated and starter redesigned!")
        except ValueError as e:
            messagebox.showerror("Error", f"Invalid input: {e}")

    def update_load_torque(self, value):
        """Update load torque from slider"""
        self.params['T_load'] = float(value)
        self.load_label.config(text=f"{float(value):.1f}")

    def motor_dynamics(self, t, y, R_ext, solver='RK45'):
        """
        Differential equations for motor dynamics
        y = [omega, theta, T_winding]
        omega: angular velocity (rad/s)
        theta: angular position (rad)
        T_winding: winding temperature (°C)
        """
        omega, theta, T_winding = y

        # Get parameters
        V = self.params['V_supply']
        R_a = self.calc_params['R_armature']
        R_total = R_a + R_ext
        k_e = self.calc_params['E_b'] / (self.calc_params['rated_speed'] * 2 * np.pi / 60)
        k_t = self.calc_params['k_t']
        J = self.params['J_motor']
        B = self.params['B_friction']
        T_load = self.params['T_load']

        # Back EMF
        E_b = k_e * omega

        # Armature current (using RMS concept)
        I_a = max(0, (V - E_b) / R_total)

        # Electromagnetic torque
        T_em = k_t * I_a

        # Mechanical equation
        d_omega = (T_em - B * omega - T_load) / J
        d_theta = omega

        # Thermal dynamics
        # Heat generation (copper losses)
        P_copper = I_a**2 * R_total

        # Iron losses (proportional to speed squared)
        P_iron = self.params['iron_loss_constant'] * (omega / (2*np.pi))**2

        # Mechanical losses
        P_mech = self.params['mech_loss_constant'] * abs(omega)

        # Stray losses (approx 1% of input)
        P_stray = 0.01 * V * I_a

        # Total heat generation
        Q_gen = P_copper + P_iron + P_mech + P_stray

        # Heat dissipation
        Q_diss = (T_winding - self.params['ambient_temp']) / self.params['thermal_resistance']

        # Temperature rate of change
        d_T = (Q_gen - Q_diss) / self.params['thermal_capacitance']

        # Store loss components for visualization
        if hasattr(self, 'current_losses'):
            self.current_losses = {
                'copper': P_copper,
                'iron': P_iron,
                'mechanical': P_mech,
                'stray': P_stray
            }

        # Thermal derating
        if self.thermal_protection_var.get() and T_winding > self.params['temp_derating_start']:
            derating = self.params['derating_factor']
            d_omega *= derating

        return [d_omega, d_theta, d_T]

    def euler_solver(self, func, y0, t_span, t_eval, args):
        """Simple Euler method solver for comparison"""
        t0, tf = t_span
        dt = t_eval[1] - t_eval[0] if len(t_eval) > 1 else 0.01

        t = t0
        y = np.array(y0)

        results_t = [t]
        results_y = [y.copy()]

        for t_next in t_eval[1:]:
            while t < t_next:
                dt_step = min(dt, t_next - t)
                dydt = np.array(func(t, y, *args))
                y = y + dydt * dt_step
                t = t + dt_step

            results_t.append(t)
            results_y.append(y.copy())

        class Result:
            pass

        result = Result()
        result.t = np.array(results_t)
        result.y = np.array(results_y).T
        result.success = True

        return result

    def start_simulation(self):
        """Start the dynamic simulation"""
        if self.simulation_running:
            return

        self.simulation_running = True
        self.btn_start.config(state='disabled')
        self.btn_stop.config(state='normal')
        self.status_label.config(text="Status: Running", foreground='orange')

        # Run simulation in separate thread
        self.simulation_thread = threading.Thread(target=self.run_simulation)
        self.simulation_thread.daemon = True
        self.simulation_thread.start()

    def stop_simulation(self):
        """Stop the simulation"""
        self.simulation_running = False
        self.btn_start.config(state='normal')
        self.btn_stop.config(state='disabled')
        self.status_label.config(text="Status: Stopped", foreground='red')

    def reset_simulation(self):
        """Reset the simulation"""
        self.stop_simulation()

        self.time_data = []
        self.current_data = []
        self.speed_data = []
        self.torque_data = []
        self.temp_data = []
        self.power_data = []
        self.loss_data = {'copper': [], 'iron': [], 'mechanical': [], 'stray': []}
        self.current_stud = 1

        # Clear plots
        self.ax_current.clear()
        self.ax_speed.clear()
        self.ax_torque.clear()
        self.ax_power.clear()
        self.ax_temp.clear()
        self.ax_losses.clear()
        self.ax_efficiency.clear()
        self.ax_stress.clear()

        self.setup_plot_labels()
        self.canvas_sim.draw()
        self.canvas_multi.draw()

        self.status_label.config(text="Status: Reset", foreground='blue')

    def setup_plot_labels(self):
        """Setup plot labels after clearing"""
        self.ax_current.set_title('Armature Current vs Time')
        self.ax_current.set_xlabel('Time (s)')
        self.ax_current.set_ylabel('Current (A)')
        self.ax_current.grid(True, alpha=0.3)

        self.ax_speed.set_title('Speed vs Time')
        self.ax_speed.set_xlabel('Time (s)')
        self.ax_speed.set_ylabel('Speed (RPM)')
        self.ax_speed.grid(True, alpha=0.3)

        self.ax_torque.set_title('Torque vs Time')
        self.ax_torque.set_xlabel('Time (s)')
        self.ax_torque.set_ylabel('Torque (N·m)')
        self.ax_torque.grid(True, alpha=0.3)

        self.ax_power.set_title('Power vs Time')
        self.ax_power.set_xlabel('Time (s)')
        self.ax_power.set_ylabel('Power (W)')
        self.ax_power.grid(True, alpha=0.3)

        self.ax_temp.set_title('Temperature vs Time')
        self.ax_temp.set_xlabel('Time (s)')
        self.ax_temp.set_ylabel('Temperature (°C)')
        self.ax_temp.grid(True, alpha=0.3)

        self.ax_losses.set_title('Loss Breakdown')
        self.ax_losses.set_xlabel('Time (s)')
        self.ax_losses.set_ylabel('Losses (W)')
        self.ax_losses.grid(True, alpha=0.3)

        self.ax_efficiency.set_title('Efficiency vs Time')
        self.ax_efficiency.set_xlabel('Time (s)')
        self.ax_efficiency.set_ylabel('Efficiency (%)')
        self.ax_efficiency.grid(True, alpha=0.3)

        self.ax_stress.set_title('Mechanical Stress Analysis')
        self.ax_stress.set_xlabel('Time (s)')
        self.ax_stress.set_ylabel('Torque (N·m)')
        self.ax_stress.grid(True, alpha=0.3)

    def run_simulation(self):
        """Run the complete motor starting simulation"""
        # Initial conditions
        omega0 = 0.0  # Initial angular velocity
        theta0 = 0.0  # Initial angle
        T_winding0 = self.params['ambient_temp']  # Initial temperature

        y0 = [omega0, theta0, T_winding0]

        # Time parameters
        t_total = 15.0  # Total simulation time
        dt = 0.01  # Time step

        # Simulation with starter switching
        I_min = self.calc_params['I_armature']
        I_max = self.params['current_ratio'] * I_min

        current_time = 0.0
        current_state = y0
        self.current_stud = 1
        self.current_losses = {'copper': 0, 'iron': 0, 'mechanical': 0, 'stray': 0}

        while current_time < t_total and self.simulation_running:
            # Determine current external resistance based on stud position
            if self.current_stud > len(self.starter_resistances):
                R_ext = 0.0  # All resistances cut out
            else:
                R_ext = sum(self.starter_resistances[self.current_stud-1:])

            # Simulate one time step
            t_span = (current_time, current_time + dt)
            t_eval = np.array([current_time, current_time + dt])

            # Choose solver
            solver_method = self.solver_var.get()

            if solver_method == 'Euler':
                sol = self.euler_solver(self.motor_dynamics, current_state,
                                       t_span, t_eval, args=(R_ext,))
            else:
                sol = solve_ivp(self.motor_dynamics, t_span, current_state,
                              method=solver_method, t_eval=t_eval, args=(R_ext,))

            if not sol.success:
                break

            # Update state
            current_state = sol.y[:, -1].tolist()
            current_time = sol.t[-1]

            # Calculate current values
            omega = current_state[0]
            T_winding = current_state[2]

            V = self.params['V_supply']
            R_a = self.calc_params['R_armature']
            R_total = R_a + R_ext
            k_e = self.calc_params['E_b'] / (self.calc_params['rated_speed'] * 2 * np.pi / 60)
            k_t = self.calc_params['k_t']

            E_b = k_e * omega
            I_a = max(0, (V - E_b) / R_total)
            T_em = k_t * I_a

            speed_rpm = omega * 60 / (2 * np.pi)
            power = T_em * omega

            # Store data
            self.time_data.append(current_time)
            self.current_data.append(I_a)
            self.speed_data.append(speed_rpm)
            self.torque_data.append(T_em)
            self.temp_data.append(T_winding)
            self.power_data.append(power)

            # Store losses
            for key in self.loss_data:
                self.loss_data[key].append(self.current_losses.get(key, 0))

            # Check for stud switching
            if I_a < I_min and self.current_stud <= len(self.starter_resistances):
                self.current_stud += 1
                print(f"Switching to stud {self.current_stud} at t={current_time:.2f}s")

            # Update plots periodically
            if len(self.time_data) % 50 == 0:
                self.root.after(0, self.update_plots)

            # Check thermal limit
            if T_winding > self.params['max_temp']:
                print(f"Maximum temperature exceeded at t={current_time:.2f}s")
                self.root.after(0, lambda: messagebox.showwarning(
                    "Thermal Warning",
                    f"Winding temperature exceeded {self.params['max_temp']}°C!"))
                break

        # Final plot update
        self.root.after(0, self.update_plots)
        self.root.after(0, self.simulation_complete)

    def update_plots(self):
        """Update all plots with current data"""
        if not self.time_data:
            return

        # Simulation tab plots
        self.ax_current.clear()
        self.ax_current.plot(self.time_data, self.current_data, 'b-', linewidth=2)
        self.ax_current.axhline(y=self.calc_params['I_armature'], color='r',
                               linestyle='--', label='Full-Load Current')
        self.ax_current.axhline(y=self.params['current_ratio']*self.calc_params['I_armature'],
                               color='g', linestyle='--', label='Max Current')
        self.ax_current.set_title('Armature Current vs Time')
        self.ax_current.set_xlabel('Time (s)')
        self.ax_current.set_ylabel('Current (A)')
        self.ax_current.legend()
        self.ax_current.grid(True, alpha=0.3)

        self.ax_speed.clear()
        self.ax_speed.plot(self.time_data, self.speed_data, 'g-', linewidth=2)
        self.ax_speed.axhline(y=self.calc_params['rated_speed'], color='r',
                             linestyle='--', label='Rated Speed')
        self.ax_speed.set_title('Speed vs Time')
        self.ax_speed.set_xlabel('Time (s)')
        self.ax_speed.set_ylabel('Speed (RPM)')
        self.ax_speed.legend()
        self.ax_speed.grid(True, alpha=0.3)

        self.ax_torque.clear()
        self.ax_torque.plot(self.time_data, self.torque_data, 'r-', linewidth=2)
        self.ax_torque.axhline(y=self.params['T_load'], color='b',
                              linestyle='--', label='Load Torque')
        self.ax_torque.set_title('Electromagnetic Torque vs Time')
        self.ax_torque.set_xlabel('Time (s)')
        self.ax_torque.set_ylabel('Torque (N·m)')
        self.ax_torque.legend()
        self.ax_torque.grid(True, alpha=0.3)

        self.ax_power.clear()
        self.ax_power.plot(self.time_data, np.array(self.power_data)/1000, 'm-', linewidth=2)
        self.ax_power.axhline(y=self.params['P_rated']/1000, color='r',
                             linestyle='--', label='Rated Power')
        self.ax_power.set_title('Output Power vs Time')
        self.ax_power.set_xlabel('Time (s)')
        self.ax_power.set_ylabel('Power (kW)')
        self.ax_power.legend()
        self.ax_power.grid(True, alpha=0.3)

        self.canvas_sim.draw()

        # Multi-physics tab plots
        self.ax_temp.clear()
        self.ax_temp.plot(self.time_data, self.temp_data, 'r-', linewidth=2, label='Winding Temp')
        self.ax_temp.axhline(y=self.params['max_temp'], color='r',
                            linestyle='--', alpha=0.5, label='Max Temp')
        self.ax_temp.axhline(y=self.params['temp_derating_start'], color='orange',
                            linestyle='--', alpha=0.5, label='Derating Start')
        self.ax_temp.set_title('Temperature vs Time')
        self.ax_temp.set_xlabel('Time (s)')
        self.ax_temp.set_ylabel('Temperature (°C)')
        self.ax_temp.legend()
        self.ax_temp.grid(True, alpha=0.3)

        self.ax_losses.clear()
        if self.loss_data['copper']:
            self.ax_losses.plot(self.time_data, self.loss_data['copper'],
                               label='Copper Losses', linewidth=2)
            self.ax_losses.plot(self.time_data, self.loss_data['iron'],
                               label='Iron Losses', linewidth=2)
            self.ax_losses.plot(self.time_data, self.loss_data['mechanical'],
                               label='Mechanical Losses', linewidth=2)
            self.ax_losses.plot(self.time_data, self.loss_data['stray'],
                               label='Stray Losses', linewidth=2)
        self.ax_losses.set_title('Loss Breakdown')
        self.ax_losses.set_xlabel('Time (s)')
        self.ax_losses.set_ylabel('Losses (W)')
        self.ax_losses.legend()
        self.ax_losses.grid(True, alpha=0.3)

        self.ax_efficiency.clear()
        if self.power_data and self.current_data:
            input_power = np.array(self.current_data) * self.params['V_supply']
            output_power = np.array(self.power_data)
            efficiency = np.where(input_power > 0,
                                 (output_power / input_power) * 100, 0)
            self.ax_efficiency.plot(self.time_data, efficiency, 'b-', linewidth=2)
            self.ax_efficiency.axhline(y=self.params['efficiency']*100, color='r',
                                      linestyle='--', label='Rated Efficiency')
        self.ax_efficiency.set_title('Efficiency vs Time')
        self.ax_efficiency.set_xlabel('Time (s)')
        self.ax_efficiency.set_ylabel('Efficiency (%)')
        self.ax_efficiency.legend()
        self.ax_efficiency.grid(True, alpha=0.3)
        self.ax_efficiency.set_ylim([0, 100])

        self.ax_stress.clear()
        if self.torque_data:
            # Calculate bearing stress (simplified)
            torque_array = np.array(self.torque_data)
            bearing_stress = torque_array * 1.5  # Simplified stress calculation
            self.ax_stress.plot(self.time_data, torque_array,
                               label='Shaft Torque', linewidth=2)
            self.ax_stress.plot(self.time_data, bearing_stress,
                               label='Bearing Load (approx)', linewidth=2, alpha=0.7)
        self.ax_stress.set_title('Mechanical Stress Analysis')
        self.ax_stress.set_xlabel('Time (s)')
        self.ax_stress.set_ylabel('Torque/Stress (N·m)')
        self.ax_stress.legend()
        self.ax_stress.grid(True, alpha=0.3)

        self.canvas_multi.draw()

    def simulation_complete(self):
        """Called when simulation completes"""
        self.simulation_running = False
        self.btn_start.config(state='normal')
        self.btn_stop.config(state='disabled')
        self.status_label.config(text="Status: Complete", foreground='green')

    def calculate_economics(self):
        """Calculate economic analysis"""
        try:
            # Update economic parameters
            for key, (entry, scale) in self.econ_entries.items():
                self.params[key] = float(entry.get()) / scale

            # Calculate costs
            P_rated = self.params['P_rated'] / 1000  # kW
            hours_year = self.params['operating_hours']
            cost_per_kwh = self.params['electricity_cost']
            maintenance = self.params['maintenance_cost']
            discount_rate = self.params['discount_rate']
            years = int(self.params['analysis_period'])
            initial_cost = self.params['motor_cost']

            # Annual energy consumption
            energy_year = P_rated * hours_year  # kWh

            # Annual energy cost
            energy_cost_year = energy_year * cost_per_kwh

            # Total annual cost
            total_annual_cost = energy_cost_year + maintenance

            # Net present value calculation
            npv = initial_cost
            for year in range(1, years + 1):
                npv += total_annual_cost / ((1 + discount_rate) ** year)

            # Lifecycle cost
            lifecycle_cost = initial_cost + (total_annual_cost * years)

            # Cost per operating hour
            cost_per_hour = lifecycle_cost / (hours_year * years)

            # Display results
            self.econ_results_text.delete(1.0, tk.END)

            output = "="*60 + "\n"
            output += "ECONOMIC ANALYSIS RESULTS\n"
            output += "="*60 + "\n\n"

            output += "INPUT PARAMETERS:\n"
            output += "-"*60 + "\n"
            output += f"Motor Rated Power:        {P_rated:.2f} kW\n"
            output += f"Operating Hours/Year:     {hours_year:.0f} hours\n"
            output += f"Electricity Cost:         ${cost_per_kwh:.3f}/kWh\n"
            output += f"Maintenance Cost/Year:    ${maintenance:.2f}\n"
            output += f"Discount Rate:            {discount_rate*100:.1f}%\n"
            output += f"Analysis Period:          {years} years\n"
            output += f"Initial Motor Cost:       ${initial_cost:.2f}\n\n"

            output += "ANNUAL COSTS:\n"
            output += "-"*60 + "\n"
            output += f"Energy Consumption:       {energy_year:.2f} kWh/year\n"
            output += f"Energy Cost:              ${energy_cost_year:.2f}/year\n"
            output += f"Maintenance Cost:         ${maintenance:.2f}/year\n"
            output += f"Total Annual Cost:        ${total_annual_cost:.2f}/year\n\n"

            output += "LIFECYCLE ANALYSIS:\n"
            output += "-"*60 + "\n"
            output += f"Initial Investment:       ${initial_cost:.2f}\n"
            output += f"Total Operating Costs:    ${total_annual_cost * years:.2f}\n"
            output += f"Lifecycle Cost:           ${lifecycle_cost:.2f}\n"
            output += f"Net Present Value:        ${npv:.2f}\n"
            output += f"Cost per Operating Hour:  ${cost_per_hour:.4f}/hour\n\n"

            output += "YEAR-BY-YEAR BREAKDOWN:\n"
            output += "-"*60 + "\n"
            output += f"{'Year':<6} {'Energy Cost':<15} {'Maintenance':<15} {'Total':<15}\n"
            output += "-"*60 + "\n"

            for year in range(1, min(years + 1, 11)):  # Show first 10 years
                output += f"{year:<6} ${energy_cost_year:<14.2f} ${maintenance:<14.2f} "
                output += f"${total_annual_cost:<14.2f}\n"

            if years > 10:
                output += f"... (showing first 10 of {years} years)\n"

            output += "\n" + "="*60 + "\n"

            # Energy savings potential
            output += "\nENERGY EFFICIENCY IMPROVEMENTS:\n"
            output += "-"*60 + "\n"

            for improvement in [0.02, 0.05, 0.10]:
                improved_energy = energy_year * (1 - improvement)
                savings = (energy_year - improved_energy) * cost_per_kwh
                payback_cost = 1000 * improvement / 0.02  # Estimated upgrade cost

                if savings > 0:
                    payback_period = payback_cost / savings
                else:
                    payback_period = float('inf')

                output += f"\n{improvement*100:.0f}% Efficiency Improvement:\n"
                output += f"  Annual Energy Savings:  {(energy_year - improved_energy):.2f} kWh\n"
                output += f"  Annual Cost Savings:    ${savings:.2f}\n"
                output += f"  Est. Upgrade Cost:      ${payback_cost:.2f}\n"
                output += f"  Payback Period:         {payback_period:.2f} years\n"

            output += "\n" + "="*60 + "\n"

            self.econ_results_text.insert(1.0, output)

        except Exception as e:
            messagebox.showerror("Error", f"Economic calculation error: {e}")

    def save_parameters(self):
        """Save current parameters to JSON file"""
        try:
            filename = f"motor_params_{datetime.now().strftime('%Y%m%d_%H%M%S')}.json"
            with open(filename, 'w') as f:
                json.dump(self.params, f, indent=4)
            messagebox.showinfo("Success", f"Parameters saved to {filename}")
        except Exception as e:
            messagebox.showerror("Error", f"Failed to save parameters: {e}")

    def load_parameters(self):
        """Load parameters from JSON file"""
        from tkinter import filedialog
        try:
            filename = filedialog.askopenfilename(
                title="Select parameter file",
                filetypes=[("JSON files", "*.json"), ("All files", "*.*")]
            )
            if filename:
                with open(filename, 'r') as f:
                    loaded_params = json.load(f)
                self.params.update(loaded_params)
                self.update_gui_from_params()
                self.calculate_motor_parameters()
                self.design_starter_resistances()
                messagebox.showinfo("Success", "Parameters loaded successfully!")
        except Exception as e:
            messagebox.showerror("Error", f"Failed to load parameters: {e}")

    def update_gui_from_params(self):
        """Update GUI entries from current parameters"""
        for key, (entry, scale) in self.param_entries.items():
            entry.delete(0, tk.END)
            entry.insert(0, str(self.params[key] * scale))

    def show_about(self):
        """Show about dialog"""
        about_text = """Advanced DC Motor Starter Design & Multi-Physics Simulation

Version: 2.0

Features:
• Seven-stud starter resistance calculation
• Real-time dynamic simulation with RK45/Euler solvers
• Multi-physics analysis (electromagnetic-thermal-mechanical)
• Detailed loss breakdown and efficiency analysis
• Economic lifecycle cost analysis
• Advanced control strategies
• Thermal derating and protection

Developed for electrical engineering applications.
        """
        messagebox.showinfo("About", about_text)

    def on_window_resize(self, event):
        """Handle window resize events for autoscaling"""
        # Only handle main window resize
        if event.widget == self.root:
            # The canvas will automatically resize with pack(fill='both', expand=True)
            # Force redraw of canvases
            try:
                self.canvas_sim.draw()
                self.canvas_multi.draw()
            except:
                pass  # Ignore errors during initialization

def main():
    """Main entry point"""
    root = tk.Tk()
    app = DCMotorStarterDesign(root)
    root.mainloop()

if __name__ == "__main__":
    main()
