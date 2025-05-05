import tkinter as tk
from tkinter import ttk

def start_experiment():
    participant = participant_entry.get()
    trial_number = trial_number_entry.get()
    weather = weather_var.get()
    driving_mode = driving_mode_var.get()
    approaching_speed = approaching_speed_var.get()

    print(f"Starting experiment with:")
    print(f"Participant: {participant}")
    print(f"Trial Number: {trial_number}")
    print(f"Weather: {weather}")
    print(f"Driving Mode: {driving_mode}")
    print(f"Approaching Speed: {approaching_speed}")

def stop_experiment():
    print("Experiment stopped.")

# Create the main window
root = tk.Tk()
root.title("Experiment Configuration")

# Participant and Trial Number
participant_label = ttk.Label(root, text="Participant:")
participant_label.grid(row=0, column=0, padx=10, pady=5, sticky="w")
participant_entry = ttk.Entry(root)
participant_entry.grid(row=0, column=1, padx=10, pady=5)

trial_number_label = ttk.Label(root, text="Trial Number:")
trial_number_label.grid(row=1, column=0, padx=10, pady=5, sticky="w")
trial_number_entry = ttk.Entry(root)
trial_number_entry.grid(row=1, column=1, padx=10, pady=5)

# Weather Options
weather_label = ttk.Label(root, text="Weather:")
weather_label.grid(row=2, column=0, padx=10, pady=5, sticky="w")
weather_var = tk.StringVar(value="Day-Clear")
weather_options = ["Day-Clear", "Day-Rain", "Night-Clear"]
for i, option in enumerate(weather_options):
    ttk.Radiobutton(root, text=option, variable=weather_var, value=option).grid(row=2+i, column=1, padx=10, pady=2, sticky="w")

# Driving Mode Options
driving_mode_label = ttk.Label(root, text="Driving Mode:")
driving_mode_label.grid(row=5, column=0, padx=10, pady=5, sticky="w")
driving_mode_var = tk.StringVar(value="Manual Driving")
driving_mode_options = ["Manual Driving", "ADAS Driving"]
for i, option in enumerate(driving_mode_options):
    ttk.Radiobutton(root, text=option, variable=driving_mode_var, value=option).grid(row=5+i, column=1, padx=10, pady=2, sticky="w")

# Approaching Speed Options
approaching_speed_label = ttk.Label(root, text="Approaching Speed:")
approaching_speed_label.grid(row=7, column=0, padx=10, pady=5, sticky="w")
approaching_speed_var = tk.StringVar(value="10")
approaching_speed_options = ["10", "40", "70"]
for i, option in enumerate(approaching_speed_options):
    ttk.Radiobutton(root, text=option, variable=approaching_speed_var, value=option).grid(row=7+i, column=1, padx=10, pady=2, sticky="w")

# Start and Stop Buttons
start_button = ttk.Button(root, text="Start", command=start_experiment)
start_button.grid(row=10, column=0, padx=10, pady=10, sticky="w")

stop_button = ttk.Button(root, text="Stop", command=stop_experiment)
stop_button.grid(row=10, column=1, padx=10, pady=10, sticky="w")

# Run the application
root.mainloop()