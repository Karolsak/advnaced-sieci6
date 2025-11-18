"""
Advanced Train Braking Energy Recovery Laboratory
Multi-Physics Simulation with Regenerative Braking Analysis

Features:
- Train braking energy calculation with regenerative braking
- Multi-physics simulation (electromagnetic-thermal-mechanical)
- Real-time ODE solvers (RK45, Euler)
- Dynamic visualization
- Economic analysis
- Advanced control systems
- Thermal modeling and derating
"""

import tkinter as tk
from tkinter import ttk, messagebox
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure
from scipy.integrate import solve_ivp
import math


class TrainBrakingEnergyLab:
    """Advanced Train Braking Energy Laboratory with Multi-Physics Simulation"""

    def __init__(self, root):
        self.root = root
        self.root.title("Advanced Train Braking Energy Recovery Laboratory")
        self.root.geometry("1400x900")

        # Simulation state
        self.is_running = False
        self.simulation_data = {
            'time': [],
            'speed': [],
            'power': [],
            'energy': [],
            'temperature': [],
            'torque': [],
            'voltage': [],
            'current': [],
            'losses': [],
            'efficiency': []
        }

        # Default parameters
        self.params = {
            # Train parameters
            'mass': tk.DoubleVar(value=400.0),  # tonnes
            'gradient': tk.DoubleVar(value=1/100),  # 1 in 100
            'initial_speed': tk.DoubleVar(value=80.0),  # km/h
            'final_speed': tk.DoubleVar(value=50.0),  # km/h
            'braking_time': tk.DoubleVar(value=20.0),  # seconds
            'tractive_resistance': tk.DoubleVar(value=49.0),  # N/t
            'rotational_inertia': tk.DoubleVar(value=7.5),  # %
            'motor_efficiency': tk.DoubleVar(value=75.0),  # %

            # Electrical parameters
            'rated_voltage': tk.DoubleVar(value=750.0),  # V DC
            'rated_current': tk.DoubleVar(value=500.0),  # A
            'armature_resistance': tk.DoubleVar(value=0.05),  # Ohm
            'field_resistance': tk.DoubleVar(value=0.08),  # Ohm
            'inductance': tk.DoubleVar(value=0.01),  # H

            # Thermal parameters
            'ambient_temp': tk.DoubleVar(value=25.0),  # °C
            'thermal_resistance': tk.DoubleVar(value=0.5),  # °C/W
            'thermal_capacitance': tk.DoubleVar(value=5000.0),  # J/°C
            'max_temp': tk.DoubleVar(value=150.0),  # °C

            # Mechanical parameters
            'wheel_diameter': tk.DoubleVar(value=1.0),  # m
            'gear_ratio': tk.DoubleVar(value=4.5),
            'moment_of_inertia': tk.DoubleVar(value=50.0),  # kg·m²

            # Economic parameters
            'electricity_cost': tk.DoubleVar(value=0.15),  # $/kWh
            'maintenance_cost': tk.DoubleVar(value=0.05),  # $/kWh

            # Simulation parameters
            'solver_method': tk.StringVar(value='RK45'),
            'time_step': tk.DoubleVar(value=0.01)
        }

        # Create GUI
        self.create_gui()

        # Bind resize event
        self.root.bind('<Configure>', self.on_window_resize)

    def create_gui(self):
        """Create the main GUI structure"""
        # Create notebook for tabs
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
        self.notebook.add(self.tab_mechanical, text="Mechanical Analysis")
        self.notebook.add(self.tab_economic, text="Economic Analysis")
        self.notebook.add(self.tab_results, text="Results")

        # Build each tab
        self.build_main_tab()
        self.build_simulation_tab()
        self.build_thermal_tab()
        self.build_mechanical_tab()
        self.build_economic_tab()
        self.build_results_tab()

    def build_main_tab(self):
        """Build main menu tab with input parameters"""
        # Main frame with scrollbar
        canvas = tk.Canvas(self.tab_main)
        scrollbar = ttk.Scrollbar(self.tab_main, orient="vertical", command=canvas.yview)
        scrollable_frame = ttk.Frame(canvas)

        scrollable_frame.bind(
            "<Configure>",
            lambda e: canvas.configure(scrollregion=canvas.bbox("all"))
        )

        canvas.create_window((0, 0), window=scrollable_frame, anchor="nw")
        canvas.configure(yscrollcommand=scrollbar.set)

        # Title
        title = ttk.Label(scrollable_frame, text="Train Braking Energy Recovery System",
                         font=("Arial", 16, "bold"))
        title.pack(pady=10)

        # Train Parameters
        train_frame = ttk.LabelFrame(scrollable_frame, text="Train Parameters", padding=10)
        train_frame.pack(fill=tk.X, padx=10, pady=5)

        self.create_parameter_slider(train_frame, "Mass (tonnes)", self.params['mass'],
                                     100, 1000, row=0)
        self.create_parameter_slider(train_frame, "Initial Speed (km/h)",
                                     self.params['initial_speed'], 0, 120, row=1)
        self.create_parameter_slider(train_frame, "Final Speed (km/h)",
                                     self.params['final_speed'], 0, 120, row=2)
        self.create_parameter_slider(train_frame, "Braking Time (s)",
                                     self.params['braking_time'], 5, 60, row=3)
        self.create_parameter_slider(train_frame, "Tractive Resistance (N/t)",
                                     self.params['tractive_resistance'], 20, 100, row=4)
        self.create_parameter_slider(train_frame, "Rotational Inertia (%)",
                                     self.params['rotational_inertia'], 0, 20, row=5)
        self.create_parameter_slider(train_frame, "Motor Efficiency (%)",
                                     self.params['motor_efficiency'], 50, 95, row=6)

        # Electrical Parameters
        elec_frame = ttk.LabelFrame(scrollable_frame, text="Electrical Parameters", padding=10)
        elec_frame.pack(fill=tk.X, padx=10, pady=5)

        self.create_parameter_slider(elec_frame, "Rated Voltage (V)",
                                     self.params['rated_voltage'], 400, 1500, row=0)
        self.create_parameter_slider(elec_frame, "Rated Current (A)",
                                     self.params['rated_current'], 100, 1000, row=1)
        self.create_parameter_slider(elec_frame, "Armature Resistance (Ω)",
                                     self.params['armature_resistance'], 0.01, 0.5, row=2)
        self.create_parameter_slider(elec_frame, "Field Resistance (Ω)",
                                     self.params['field_resistance'], 0.01, 0.5, row=3)
        self.create_parameter_slider(elec_frame, "Inductance (H)",
                                     self.params['inductance'], 0.001, 0.1, row=4)

        # Thermal Parameters
        thermal_frame = ttk.LabelFrame(scrollable_frame, text="Thermal Parameters", padding=10)
        thermal_frame.pack(fill=tk.X, padx=10, pady=5)

        self.create_parameter_slider(thermal_frame, "Ambient Temperature (°C)",
                                     self.params['ambient_temp'], 0, 50, row=0)
        self.create_parameter_slider(thermal_frame, "Thermal Resistance (°C/W)",
                                     self.params['thermal_resistance'], 0.1, 2.0, row=1)
        self.create_parameter_slider(thermal_frame, "Thermal Capacitance (J/°C)",
                                     self.params['thermal_capacitance'], 1000, 10000, row=2)
        self.create_parameter_slider(thermal_frame, "Max Temperature (°C)",
                                     self.params['max_temp'], 100, 200, row=3)

        # Control Buttons
        control_frame = ttk.Frame(scrollable_frame)
        control_frame.pack(fill=tk.X, padx=10, pady=20)

        ttk.Button(control_frame, text="▶ Start Simulation",
                  command=self.start_simulation).pack(side=tk.LEFT, padx=5)
        ttk.Button(control_frame, text="⏸ Stop Simulation",
                  command=self.stop_simulation).pack(side=tk.LEFT, padx=5)
        ttk.Button(control_frame, text="↻ Reset",
                  command=self.reset_simulation).pack(side=tk.LEFT, padx=5)
        ttk.Button(control_frame, text="Calculate Energy",
                  command=self.calculate_braking_energy).pack(side=tk.LEFT, padx=5)

        canvas.pack(side="left", fill="both", expand=True)
        scrollbar.pack(side="right", fill="y")

    def create_parameter_slider(self, parent, label, variable, min_val, max_val, row):
        """Create a parameter slider with label and value display"""
        frame = ttk.Frame(parent)
        frame.grid(row=row, column=0, sticky="ew", pady=2)
        parent.grid_columnconfigure(0, weight=1)

        ttk.Label(frame, text=label, width=30).pack(side=tk.LEFT)

        slider = ttk.Scale(frame, from_=min_val, to=max_val, orient=tk.HORIZONTAL,
                          variable=variable, length=200)
        slider.pack(side=tk.LEFT, padx=5, fill=tk.X, expand=True)

        value_label = ttk.Label(frame, text=f"{variable.get():.2f}", width=10)
        value_label.pack(side=tk.LEFT)

        def update_label(*args):
            value_label.config(text=f"{variable.get():.2f}")

        variable.trace('w', update_label)

    def build_simulation_tab(self):
        """Build simulation tab with real-time graphs"""
        # Control panel
        control_panel = ttk.Frame(self.tab_simulation)
        control_panel.pack(side=tk.TOP, fill=tk.X, padx=5, pady=5)

        ttk.Label(control_panel, text="Solver Method:").pack(side=tk.LEFT, padx=5)
        solver_combo = ttk.Combobox(control_panel, textvariable=self.params['solver_method'],
                                    values=['RK45', 'Euler', 'RK23', 'DOP853'],
                                    state='readonly', width=10)
        solver_combo.pack(side=tk.LEFT, padx=5)

        ttk.Label(control_panel, text="Time Step (s):").pack(side=tk.LEFT, padx=5)
        ttk.Entry(control_panel, textvariable=self.params['time_step'],
                 width=10).pack(side=tk.LEFT, padx=5)

        # Create matplotlib figure for dynamic graphs
        self.sim_fig = Figure(figsize=(12, 8))
        self.sim_canvas = FigureCanvasTkAgg(self.sim_fig, self.tab_simulation)
        self.sim_canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True, padx=5, pady=5)

        # Create subplots
        self.ax_speed = self.sim_fig.add_subplot(2, 3, 1)
        self.ax_power = self.sim_fig.add_subplot(2, 3, 2)
        self.ax_energy = self.sim_fig.add_subplot(2, 3, 3)
        self.ax_voltage = self.sim_fig.add_subplot(2, 3, 4)
        self.ax_current = self.sim_fig.add_subplot(2, 3, 5)
        self.ax_efficiency = self.sim_fig.add_subplot(2, 3, 6)

        self.sim_fig.tight_layout()

    def build_thermal_tab(self):
        """Build thermal analysis tab"""
        # Info panel
        info_frame = ttk.LabelFrame(self.tab_thermal, text="Thermal Model Information",
                                   padding=10)
        info_frame.pack(fill=tk.X, padx=5, pady=5)

        info_text = """
        Multi-Physics Thermal Model:
        - Coupled electromagnetic-thermal equations
        - Heat transfer analysis (conduction, convection, radiation)
        - Temperature-dependent material properties
        - Thermal derating and protection

        Heat Sources:
        - Copper losses (I²R) in armature and field windings
        - Iron losses (hysteresis and eddy currents)
        - Mechanical friction losses
        - Stray load losses
        """
        ttk.Label(info_frame, text=info_text, justify=tk.LEFT).pack()

        # Thermal graphs
        self.thermal_fig = Figure(figsize=(12, 6))
        self.thermal_canvas = FigureCanvasTkAgg(self.thermal_fig, self.tab_thermal)
        self.thermal_canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True, padx=5, pady=5)

        self.ax_temp = self.thermal_fig.add_subplot(1, 2, 1)
        self.ax_losses = self.thermal_fig.add_subplot(1, 2, 2)

        self.thermal_fig.tight_layout()

    def build_mechanical_tab(self):
        """Build mechanical analysis tab"""
        # Info panel
        info_frame = ttk.LabelFrame(self.tab_mechanical, text="Mechanical Model Information",
                                   padding=10)
        info_frame.pack(fill=tk.X, padx=5, pady=5)

        info_text = """
        Mechanical Stress Analysis:
        - Shaft torque transients
        - Bearing load calculations
        - Rotational dynamics with moment of inertia
        - Gear train analysis
        - Vibration analysis
        """
        ttk.Label(info_frame, text=info_text, justify=tk.LEFT).pack()

        # Mechanical parameters
        mech_frame = ttk.LabelFrame(self.tab_mechanical, text="Mechanical Parameters",
                                   padding=10)
        mech_frame.pack(fill=tk.X, padx=10, pady=5)

        self.create_parameter_slider(mech_frame, "Wheel Diameter (m)",
                                     self.params['wheel_diameter'], 0.5, 2.0, row=0)
        self.create_parameter_slider(mech_frame, "Gear Ratio",
                                     self.params['gear_ratio'], 2.0, 10.0, row=1)
        self.create_parameter_slider(mech_frame, "Moment of Inertia (kg·m²)",
                                     self.params['moment_of_inertia'], 10, 200, row=2)

        # Mechanical graphs
        self.mech_fig = Figure(figsize=(12, 6))
        self.mech_canvas = FigureCanvasTkAgg(self.mech_fig, self.tab_mechanical)
        self.mech_canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True, padx=5, pady=5)

        self.ax_torque = self.mech_fig.add_subplot(1, 2, 1)
        self.ax_force = self.mech_fig.add_subplot(1, 2, 2)

        self.mech_fig.tight_layout()

    def build_economic_tab(self):
        """Build economic analysis tab"""
        # Economic parameters
        econ_frame = ttk.LabelFrame(self.tab_economic, text="Economic Parameters",
                                   padding=10)
        econ_frame.pack(fill=tk.X, padx=10, pady=5)

        self.create_parameter_slider(econ_frame, "Electricity Cost ($/kWh)",
                                     self.params['electricity_cost'], 0.05, 0.50, row=0)
        self.create_parameter_slider(econ_frame, "Maintenance Cost ($/kWh)",
                                     self.params['maintenance_cost'], 0.01, 0.20, row=1)

        # Results display
        results_frame = ttk.LabelFrame(self.tab_economic, text="Economic Analysis Results",
                                      padding=10)
        results_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=5)

        self.econ_text = tk.Text(results_frame, height=20, width=80, font=("Courier", 10))
        self.econ_text.pack(fill=tk.BOTH, expand=True)

        # Economic graphs
        self.econ_fig = Figure(figsize=(12, 4))
        self.econ_canvas = FigureCanvasTkAgg(self.econ_fig, self.tab_economic)
        self.econ_canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True, padx=5, pady=5)

        self.ax_savings = self.econ_fig.add_subplot(1, 2, 1)
        self.ax_payback = self.econ_fig.add_subplot(1, 2, 2)

        self.econ_fig.tight_layout()

    def build_results_tab(self):
        """Build results display tab"""
        # Results text area
        results_frame = ttk.LabelFrame(self.tab_results, text="Detailed Results",
                                      padding=10)
        results_frame.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)

        # Add scrollbar
        scrollbar = ttk.Scrollbar(results_frame)
        scrollbar.pack(side=tk.RIGHT, fill=tk.Y)

        self.results_text = tk.Text(results_frame, height=30, width=100,
                                   font=("Courier", 10),
                                   yscrollcommand=scrollbar.set)
        self.results_text.pack(fill=tk.BOTH, expand=True)
        scrollbar.config(command=self.results_text.yview)

        # Export button
        ttk.Button(self.tab_results, text="Export Results",
                  command=self.export_results).pack(pady=5)

    def calculate_braking_energy(self):
        """Calculate train braking energy with regenerative braking"""
        try:
            # Get parameters
            mass = self.params['mass'].get() * 1000  # Convert to kg
            gradient = self.params['gradient'].get()
            v1 = self.params['initial_speed'].get() / 3.6  # Convert km/h to m/s
            v2 = self.params['final_speed'].get() / 3.6
            time = self.params['braking_time'].get()
            resistance = self.params['tractive_resistance'].get()
            rot_inertia = self.params['rotational_inertia'].get() / 100
            efficiency = self.params['motor_efficiency'].get() / 100

            # Calculate effective mass (including rotational inertia)
            m_eff = mass * (1 + rot_inertia)

            # Calculate kinetic energy change
            KE1 = 0.5 * m_eff * v1**2
            KE2 = 0.5 * m_eff * v2**2
            delta_KE = KE1 - KE2

            # Calculate distance traveled (average velocity method)
            v_avg = (v1 + v2) / 2
            distance = v_avg * time

            # Calculate potential energy change (gradient component)
            # Gradient of 1 in 100 means: for every 100m horizontal, 1m vertical drop
            height_change = distance * gradient
            delta_PE = mass * 9.81 * height_change  # This is energy gained from going downhill

            # Calculate energy lost to tractive resistance
            total_resistance_force = resistance * (mass / 1000)  # N/t * tonnes = N
            energy_resistance = total_resistance_force * distance

            # Total mechanical energy available for regeneration
            # Energy from deceleration + Energy from downhill - Energy lost to resistance
            total_mech_energy = delta_KE + delta_PE - energy_resistance

            # Energy returned to lines (accounting for motor efficiency)
            energy_returned = total_mech_energy * efficiency

            # Calculate average braking power
            avg_power = energy_returned / time

            # Calculate braking force
            deceleration = (v2 - v1) / time
            braking_force = m_eff * abs(deceleration)

            # Display results
            results = f"""
{'='*70}
TRAIN BRAKING ENERGY RECOVERY ANALYSIS
{'='*70}

INPUT PARAMETERS:
  Train mass:                {mass/1000:.1f} tonnes ({mass:.0f} kg)
  Gradient:                  1 in {1/gradient:.0f} ({gradient*100:.2f}%)
  Initial speed:             {v1*3.6:.1f} km/h ({v1:.2f} m/s)
  Final speed:               {v2*3.6:.1f} km/h ({v2:.2f} m/s)
  Braking time:              {time:.1f} s
  Tractive resistance:       {resistance:.1f} N/t
  Rotational inertia:        {rot_inertia*100:.1f}%
  Motor efficiency:          {efficiency*100:.1f}%

CALCULATED VALUES:
  Effective mass:            {m_eff:.0f} kg
  Average velocity:          {v_avg:.2f} m/s ({v_avg*3.6:.2f} km/h)
  Distance traveled:         {distance:.2f} m
  Height change (descent):   {height_change:.2f} m
  Deceleration:              {abs(deceleration):.3f} m/s²
  Braking force:             {braking_force:.2f} N

ENERGY ANALYSIS:
  Initial kinetic energy:    {KE1/1e6:.3f} MJ
  Final kinetic energy:      {KE2/1e6:.3f} MJ
  Change in KE:              {delta_KE/1e6:.3f} MJ

  Potential energy change:   {delta_PE/1e6:.3f} MJ (from gradient)
  Energy lost to resistance: {energy_resistance/1e6:.3f} MJ

  Total mechanical energy:   {total_mech_energy/1e6:.3f} MJ
  Energy returned to lines:  {energy_returned/1e6:.3f} MJ ({energy_returned/1e3:.2f} kWh)

  Average braking power:     {avg_power/1e3:.2f} kW ({avg_power/1e6:.3f} MW)
  Peak power (estimated):    {avg_power*1.5/1e3:.2f} kW

ELECTRICAL PARAMETERS (RMS VALUES):
  Line voltage:              {self.params['rated_voltage'].get():.0f} V DC
  Average current:           {avg_power/self.params['rated_voltage'].get():.2f} A
  Peak current (estimated):  {avg_power*1.5/self.params['rated_voltage'].get():.2f} A

ECONOMIC ANALYSIS:
  Energy cost savings:       ${energy_returned/1e3 * self.params['electricity_cost'].get():.2f}
  Annual savings (est.):     ${energy_returned/1e3 * self.params['electricity_cost'].get() * 365:.2f}

{'='*70}
            """

            self.results_text.delete(1.0, tk.END)
            self.results_text.insert(1.0, results)

            # Update economic tab
            self.update_economic_analysis(energy_returned)

            messagebox.showinfo("Calculation Complete",
                              f"Energy returned to lines: {energy_returned/1e3:.2f} kWh\n"
                              f"Average power: {avg_power/1e3:.2f} kW")

        except Exception as e:
            messagebox.showerror("Error", f"Calculation error: {str(e)}")

    def start_simulation(self):
        """Start the dynamic simulation"""
        self.is_running = True
        self.reset_data()
        self.run_dynamic_simulation()

    def stop_simulation(self):
        """Stop the simulation"""
        self.is_running = False

    def reset_simulation(self):
        """Reset all simulation data"""
        self.is_running = False
        self.reset_data()
        self.clear_all_plots()

    def reset_data(self):
        """Reset simulation data arrays"""
        for key in self.simulation_data:
            self.simulation_data[key] = []

    def run_dynamic_simulation(self):
        """Run dynamic simulation with ODE solver"""
        try:
            # Initial conditions
            v0 = self.params['initial_speed'].get() / 3.6  # m/s
            temp0 = self.params['ambient_temp'].get()
            current0 = 0.0

            # State vector: [velocity, temperature, current, energy]
            y0 = [v0, temp0, current0, 0.0]

            # Time span
            t_span = (0, self.params['braking_time'].get())
            t_eval = np.linspace(0, self.params['braking_time'].get(), 500)

            # Select solver method
            method = self.params['solver_method'].get()

            # Solve ODE system
            if method == 'Euler':
                solution = self.euler_solve(self.system_dynamics, y0, t_eval)
            else:
                solution = solve_ivp(self.system_dynamics, t_span, y0,
                                    method=method, t_eval=t_eval,
                                    dense_output=True)

            # Extract results
            if method == 'Euler':
                t = solution['t']
                y = solution['y']
            else:
                t = solution.t
                y = solution.y

            # Store results
            self.simulation_data['time'] = t
            self.simulation_data['speed'] = y[0] * 3.6  # Convert to km/h
            self.simulation_data['temperature'] = y[1]
            self.simulation_data['current'] = y[2]
            self.simulation_data['energy'] = y[3] / 1e6  # Convert to MJ

            # Calculate derived quantities
            self.calculate_derived_quantities(t, y)

            # Update all plots
            self.update_all_plots()

            messagebox.showinfo("Simulation Complete",
                              "Dynamic simulation completed successfully!")

        except Exception as e:
            messagebox.showerror("Simulation Error", f"Error during simulation: {str(e)}")

    def system_dynamics(self, t, y):
        """System of differential equations for multi-physics simulation"""
        v, temp, current, energy = y

        # Get parameters
        mass = self.params['mass'].get() * 1000
        gradient = self.params['gradient'].get()
        resistance = self.params['tractive_resistance'].get()
        rot_inertia = self.params['rotational_inertia'].get() / 100
        efficiency = self.params['motor_efficiency'].get() / 100

        Ra = self.params['armature_resistance'].get()
        Rf = self.params['field_resistance'].get()
        L = self.params['inductance'].get()
        V_rated = self.params['rated_voltage'].get()

        R_th = self.params['thermal_resistance'].get()
        C_th = self.params['thermal_capacitance'].get()
        T_amb = self.params['ambient_temp'].get()

        # Effective mass
        m_eff = mass * (1 + rot_inertia)

        # Forces
        F_gravity = mass * 9.81 * gradient  # Downhill force
        F_resistance = resistance * (mass / 1000)

        # Braking force (proportional to desired deceleration)
        v_target = self.params['final_speed'].get() / 3.6
        t_brake = self.params['braking_time'].get()
        target_decel = (v - v_target) / (t_brake - t)if t < t_brake else 0
        F_brake = m_eff * max(0, target_decel)

        # Net force
        F_net = F_gravity - F_resistance - F_brake

        # Velocity derivative (equation of motion)
        dv_dt = F_net / m_eff

        # Electrical dynamics
        # Back EMF proportional to velocity
        Ke = 0.5  # EMF constant
        E_back = Ke * v

        # Current dynamics (considering inductance)
        # V = E_back + I*Ra + L*dI/dt
        # Regenerative braking: current flows back to supply
        dI_dt = (E_back - V_rated - current * Ra) / L

        # Power
        P_mech = F_brake * v  # Mechanical power from braking
        P_elec = V_rated * current * efficiency  # Electrical power to grid

        # Losses
        P_copper = current**2 * (Ra + Rf)  # Copper losses
        P_iron = 0.01 * v**2  # Iron losses (proportional to speed squared)
        P_friction = 0.005 * v  # Mechanical friction
        P_total_loss = P_copper + P_iron + P_friction

        # Thermal dynamics
        # dT/dt = (P_loss - (T - T_amb)/R_th) / C_th
        dT_dt = (P_total_loss - (temp - T_amb) / R_th) / C_th

        # Thermal derating (reduce current if overheating)
        if temp > self.params['max_temp'].get() * 0.9:
            dI_dt *= 0.5  # Reduce current rate of change

        # Energy derivative
        dE_dt = P_elec

        return [dv_dt, dT_dt, dI_dt, dE_dt]

    def euler_solve(self, func, y0, t_eval):
        """Simple Euler method ODE solver"""
        y = np.zeros((len(y0), len(t_eval)))
        y[:, 0] = y0

        for i in range(len(t_eval) - 1):
            dt = t_eval[i+1] - t_eval[i]
            dydt = func(t_eval[i], y[:, i])
            y[:, i+1] = y[:, i] + np.array(dydt) * dt

        return {'t': t_eval, 'y': y}

    def calculate_derived_quantities(self, t, y):
        """Calculate power, voltage, losses, etc."""
        v = y[0]
        temp = y[1]
        current = y[2]

        V_rated = self.params['rated_voltage'].get()
        Ra = self.params['armature_resistance'].get()
        Rf = self.params['field_resistance'].get()
        efficiency = self.params['motor_efficiency'].get() / 100

        # Power
        power = V_rated * current * efficiency / 1000  # kW
        self.simulation_data['power'] = power

        # Voltage (RMS value)
        Ke = 0.5
        voltage = np.abs(Ke * v + current * Ra)
        self.simulation_data['voltage'] = voltage

        # Losses
        losses_copper = current**2 * (Ra + Rf)
        losses_iron = 0.01 * v**2
        losses_friction = 0.005 * v
        total_losses = losses_copper + losses_iron + losses_friction
        self.simulation_data['losses'] = total_losses / 1000  # kW

        # Torque
        mass = self.params['mass'].get() * 1000
        wheel_dia = self.params['wheel_diameter'].get()
        gear_ratio = self.params['gear_ratio'].get()

        # Calculate torque from deceleration
        dv_dt = np.gradient(v, t)
        force = mass * abs(dv_dt)
        torque = force * (wheel_dia / 2) * gear_ratio / 1000  # kN·m
        self.simulation_data['torque'] = torque

        # Efficiency
        P_mech = force * v / 1000  # kW
        P_elec = power
        eff = np.where(P_mech > 0, (P_elec / P_mech) * 100, 0)
        eff = np.clip(eff, 0, 100)
        self.simulation_data['efficiency'] = eff

    def update_all_plots(self):
        """Update all visualization plots"""
        self.update_simulation_plots()
        self.update_thermal_plots()
        self.update_mechanical_plots()

    def update_simulation_plots(self):
        """Update simulation tab plots"""
        t = self.simulation_data['time']

        # Speed plot
        self.ax_speed.clear()
        self.ax_speed.plot(t, self.simulation_data['speed'], 'b-', linewidth=2)
        self.ax_speed.set_xlabel('Time (s)')
        self.ax_speed.set_ylabel('Speed (km/h)')
        self.ax_speed.set_title('Train Speed vs Time')
        self.ax_speed.grid(True, alpha=0.3)

        # Power plot
        self.ax_power.clear()
        self.ax_power.plot(t, self.simulation_data['power'], 'r-', linewidth=2)
        self.ax_power.set_xlabel('Time (s)')
        self.ax_power.set_ylabel('Power (kW)')
        self.ax_power.set_title('Regenerative Power')
        self.ax_power.grid(True, alpha=0.3)

        # Energy plot
        self.ax_energy.clear()
        self.ax_energy.plot(t, self.simulation_data['energy'], 'g-', linewidth=2)
        self.ax_energy.set_xlabel('Time (s)')
        self.ax_energy.set_ylabel('Energy (MJ)')
        self.ax_energy.set_title('Recovered Energy')
        self.ax_energy.grid(True, alpha=0.3)

        # Voltage plot
        self.ax_voltage.clear()
        self.ax_voltage.plot(t, self.simulation_data['voltage'], 'm-', linewidth=2)
        self.ax_voltage.set_xlabel('Time (s)')
        self.ax_voltage.set_ylabel('Voltage (V RMS)')
        self.ax_voltage.set_title('Generator Voltage')
        self.ax_voltage.grid(True, alpha=0.3)

        # Current plot
        self.ax_current.clear()
        self.ax_current.plot(t, self.simulation_data['current'], 'c-', linewidth=2)
        self.ax_current.set_xlabel('Time (s)')
        self.ax_current.set_ylabel('Current (A RMS)')
        self.ax_current.set_title('Line Current')
        self.ax_current.grid(True, alpha=0.3)

        # Efficiency plot
        self.ax_efficiency.clear()
        self.ax_efficiency.plot(t, self.simulation_data['efficiency'], 'orange', linewidth=2)
        self.ax_efficiency.set_xlabel('Time (s)')
        self.ax_efficiency.set_ylabel('Efficiency (%)')
        self.ax_efficiency.set_title('System Efficiency')
        self.ax_efficiency.set_ylim([0, 100])
        self.ax_efficiency.grid(True, alpha=0.3)

        self.sim_fig.tight_layout()
        self.sim_canvas.draw()

    def update_thermal_plots(self):
        """Update thermal analysis plots"""
        if len(self.simulation_data['time']) == 0:
            return

        t = self.simulation_data['time']

        # Temperature plot
        self.ax_temp.clear()
        self.ax_temp.plot(t, self.simulation_data['temperature'], 'r-',
                         linewidth=2, label='Winding Temperature')
        self.ax_temp.axhline(y=self.params['max_temp'].get(), color='r',
                            linestyle='--', label='Max Temperature')
        self.ax_temp.axhline(y=self.params['ambient_temp'].get(), color='b',
                            linestyle='--', label='Ambient Temperature')
        self.ax_temp.set_xlabel('Time (s)')
        self.ax_temp.set_ylabel('Temperature (°C)')
        self.ax_temp.set_title('Thermal Analysis')
        self.ax_temp.legend()
        self.ax_temp.grid(True, alpha=0.3)

        # Losses breakdown
        self.ax_losses.clear()

        # Calculate individual losses
        current = np.array(self.simulation_data['current'])
        speed = np.array(self.simulation_data['speed']) / 3.6  # Convert to m/s

        Ra = self.params['armature_resistance'].get()
        Rf = self.params['field_resistance'].get()

        copper_losses = current**2 * (Ra + Rf) / 1000  # kW
        iron_losses = 0.01 * speed**2 / 1000
        friction_losses = 0.005 * speed / 1000

        self.ax_losses.plot(t, copper_losses, 'r-', label='Copper Losses', linewidth=2)
        self.ax_losses.plot(t, iron_losses, 'b-', label='Iron Losses', linewidth=2)
        self.ax_losses.plot(t, friction_losses, 'g-', label='Friction Losses', linewidth=2)
        self.ax_losses.plot(t, self.simulation_data['losses'], 'k--',
                           label='Total Losses', linewidth=2)
        self.ax_losses.set_xlabel('Time (s)')
        self.ax_losses.set_ylabel('Power Loss (kW)')
        self.ax_losses.set_title('Loss Breakdown')
        self.ax_losses.legend()
        self.ax_losses.grid(True, alpha=0.3)

        self.thermal_fig.tight_layout()
        self.thermal_canvas.draw()

    def update_mechanical_plots(self):
        """Update mechanical analysis plots"""
        if len(self.simulation_data['time']) == 0:
            return

        t = self.simulation_data['time']

        # Torque plot
        self.ax_torque.clear()
        self.ax_torque.plot(t, self.simulation_data['torque'], 'b-', linewidth=2)
        self.ax_torque.set_xlabel('Time (s)')
        self.ax_torque.set_ylabel('Torque (kN·m)')
        self.ax_torque.set_title('Shaft Torque')
        self.ax_torque.grid(True, alpha=0.3)

        # Force analysis
        self.ax_force.clear()

        mass = self.params['mass'].get() * 1000
        resistance = self.params['tractive_resistance'].get()
        gradient = self.params['gradient'].get()

        speed = np.array(self.simulation_data['speed']) / 3.6
        decel = np.gradient(speed, t)

        F_brake = mass * abs(decel) / 1000  # kN
        F_resistance = np.ones_like(t) * resistance * (mass / 1000) / 1000
        F_gravity = np.ones_like(t) * mass * 9.81 * gradient / 1000

        self.ax_force.plot(t, F_brake, 'r-', label='Braking Force', linewidth=2)
        self.ax_force.plot(t, F_resistance, 'b-', label='Resistance Force', linewidth=2)
        self.ax_force.plot(t, F_gravity, 'g-', label='Gravity Force', linewidth=2)
        self.ax_force.set_xlabel('Time (s)')
        self.ax_force.set_ylabel('Force (kN)')
        self.ax_force.set_title('Force Analysis')
        self.ax_force.legend()
        self.ax_force.grid(True, alpha=0.3)

        self.mech_fig.tight_layout()
        self.mech_canvas.draw()

    def update_economic_analysis(self, energy_returned):
        """Update economic analysis"""
        electricity_cost = self.params['electricity_cost'].get()
        maintenance_cost = self.params['maintenance_cost'].get()

        # Energy in kWh
        energy_kwh = energy_returned / 1e3

        # Calculate savings
        energy_value = energy_kwh * electricity_cost
        maintenance = energy_kwh * maintenance_cost
        net_savings = energy_value - maintenance

        # Annual projections (assuming 365 braking events per year)
        annual_energy = energy_kwh * 365
        annual_savings = net_savings * 365

        # Payback analysis (assuming system cost)
        system_cost = 50000  # Example: $50,000
        payback_years = system_cost / annual_savings if annual_savings > 0 else float('inf')

        # Display results
        results = f"""
{'='*70}
ECONOMIC ANALYSIS - REGENERATIVE BRAKING SYSTEM
{'='*70}

SINGLE BRAKING EVENT:
  Energy recovered:          {energy_kwh:.2f} kWh
  Energy value:              ${energy_value:.2f}
  Maintenance cost:          ${maintenance:.2f}
  Net savings:               ${net_savings:.2f}

ANNUAL PROJECTIONS (365 events/year):
  Total energy recovered:    {annual_energy:.2f} kWh/year
  Annual energy value:       ${annual_energy * electricity_cost:.2f}
  Annual maintenance:        ${annual_energy * maintenance_cost:.2f}
  Net annual savings:        ${annual_savings:.2f}

INVESTMENT ANALYSIS:
  Estimated system cost:     ${system_cost:,.2f}
  Simple payback period:     {payback_years:.1f} years
  20-year NPV (5% discount): ${self.calculate_npv(annual_savings, 20, 0.05):,.2f}
  ROI (20 years):            {(annual_savings * 20 / system_cost - 1) * 100:.1f}%

ENVIRONMENTAL IMPACT:
  CO2 reduction/year:        {annual_energy * 0.5:.2f} kg
  Equivalent trees planted:  {annual_energy * 0.5 / 20:.0f} trees

{'='*70}
        """

        self.econ_text.delete(1.0, tk.END)
        self.econ_text.insert(1.0, results)

        # Update economic plots
        self.plot_economic_analysis(annual_savings, payback_years)

    def calculate_npv(self, annual_cash_flow, years, discount_rate):
        """Calculate Net Present Value"""
        npv = 0
        for year in range(1, years + 1):
            npv += annual_cash_flow / ((1 + discount_rate) ** year)
        return npv

    def plot_economic_analysis(self, annual_savings, payback_years):
        """Plot economic analysis graphs"""
        # Savings over time
        self.ax_savings.clear()
        years = np.arange(0, 21)
        cumulative_savings = years * annual_savings

        self.ax_savings.plot(years, cumulative_savings / 1000, 'g-', linewidth=2)
        self.ax_savings.set_xlabel('Years')
        self.ax_savings.set_ylabel('Cumulative Savings ($1000s)')
        self.ax_savings.set_title('Cumulative Savings Over Time')
        self.ax_savings.grid(True, alpha=0.3)

        # Payback period
        self.ax_payback.clear()
        system_cost = 50000
        payback_curve = years * annual_savings - system_cost

        self.ax_payback.plot(years, payback_curve / 1000, 'b-', linewidth=2)
        self.ax_payback.axhline(y=0, color='r', linestyle='--', label='Break-even')
        self.ax_payback.set_xlabel('Years')
        self.ax_payback.set_ylabel('Net Cash Flow ($1000s)')
        self.ax_payback.set_title('Payback Analysis')
        self.ax_payback.legend()
        self.ax_payback.grid(True, alpha=0.3)

        self.econ_fig.tight_layout()
        self.econ_canvas.draw()

    def clear_all_plots(self):
        """Clear all plots"""
        for ax in [self.ax_speed, self.ax_power, self.ax_energy,
                  self.ax_voltage, self.ax_current, self.ax_efficiency]:
            ax.clear()
        self.sim_canvas.draw()

        for ax in [self.ax_temp, self.ax_losses]:
            ax.clear()
        self.thermal_canvas.draw()

        for ax in [self.ax_torque, self.ax_force]:
            ax.clear()
        self.mech_canvas.draw()

    def on_window_resize(self, event):
        """Handle window resize events for auto-scaling"""
        try:
            # Only resize if the event is for the main window
            if event.widget == self.root:
                # Redraw all canvases
                if hasattr(self, 'sim_canvas'):
                    self.sim_fig.tight_layout()
                    self.sim_canvas.draw_idle()
                if hasattr(self, 'thermal_canvas'):
                    self.thermal_fig.tight_layout()
                    self.thermal_canvas.draw_idle()
                if hasattr(self, 'mech_canvas'):
                    self.mech_fig.tight_layout()
                    self.mech_canvas.draw_idle()
                if hasattr(self, 'econ_canvas'):
                    self.econ_fig.tight_layout()
                    self.econ_canvas.draw_idle()
        except:
            pass

    def export_results(self):
        """Export results to text file"""
        try:
            with open('train_braking_results.txt', 'w') as f:
                f.write(self.results_text.get(1.0, tk.END))
            messagebox.showinfo("Export Success", "Results exported to train_braking_results.txt")
        except Exception as e:
            messagebox.showerror("Export Error", f"Error exporting results: {str(e)}")


def main():
    """Main application entry point"""
    root = tk.Tk()
    app = TrainBrakingEnergyLab(root)
    root.mainloop()


if __name__ == "__main__":
    main()
