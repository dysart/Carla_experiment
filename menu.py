import tkinter as tk
from tkinter import ttk
import subprocess
import socket

def set_weather(weather_option):
    import carla
    client = carla.Client("localhost", 2000)
    client.set_timeout(30.0)
    world = client.get_world()
    weather_presets = {
        "Clear Noon": carla.WeatherParameters.ClearNoon,
        "Hard Rain Noon": carla.WeatherParameters.HardRainNoon,
        "Clear Night": carla.WeatherParameters.ClearNight,
    }
    selected_weather = weather_presets.get(weather_option, carla.WeatherParameters.ClearNoon)
    world.set_weather(selected_weather)
    print(f"Weather set to {weather_option}")

def apply_weather_change(*args):
    selected_weather = weather_var.get()
    set_weather(selected_weather)

adas_process = None

def start_experiment():
    global adas_process
    print("Starting ADAS_L.py (vehicles spawn and move)...")
    target_vehicle = target_vehicle_var.get()
    approaching_speed = approaching_speed_var.get()
    driving_mode = driving_mode_var.get()
    if driving_mode == "ADAS Driving":
        adas_process = subprocess.Popen([
            "python", "ADAS_L.py",
            "--target-vehicle", target_vehicle,
            "--approaching-speed", approaching_speed
        ])
    else:
        adas_process = subprocess.Popen([
            "python", "manual_control_dash_following.py"
        ])

def reset_experiment():
    print("Requesting vehicle respawn in ADAS_L.py...")
    # Send a UDP message to localhost:9090 to request respawn
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
            sock.sendto(b"respawn", ("127.0.0.1", 9090))
    except Exception as e:
        print(f"Failed to send respawn message: {e}")

root = tk.Tk()
root.title("Experiment Configuration")

participant_label = ttk.Label(root, text="Participant:")
participant_label.grid(row=0, column=0, padx=10, pady=5, sticky="w")
participant_entry = ttk.Entry(root)
participant_entry.grid(row=0, column=1, padx=10, pady=5)

trial_number_label = ttk.Label(root, text="Trial Number:")
trial_number_label.grid(row=1, column=0, padx=10, pady=5, sticky="w")
trial_number_entry = ttk.Entry(root)
trial_number_entry.grid(row=1, column=1, padx=10, pady=5)

weather_label = ttk.Label(root, text="Weather:")
weather_label.grid(row=2, column=0, padx=10, pady=5, sticky="w")
weather_var = tk.StringVar(value="Clear Noon")
weather_var.trace_add("write", apply_weather_change)
weather_options = ["Clear Noon", "Hard Rain Noon", "Clear Night"]
for i, option in enumerate(weather_options):
    ttk.Radiobutton(root, text=option, variable=weather_var, value=option).grid(row=2+i, column=1, padx=10, pady=2, sticky="w")

divider1 = ttk.Separator(root, orient='horizontal')
divider1.grid(row=5+len(weather_options), column=0, columnspan=3, padx=5, pady=8, sticky="ew")

target_vehicle_label = ttk.Label(root, text="Target Vehicle:")
target_vehicle_label.grid(row=6+len(weather_options), column=0, padx=10, pady=5, sticky="w")
target_vehicle_var = tk.StringVar(value="vehicle.mini.cooper_s_2021")
target_vehicle_options = [
    ("mini", "vehicle.mini.cooper_s_2021"),
    ("Lincoln MKZ", "vehicle.lincoln.mkz_2020"),
]
for i, (label, value) in enumerate(target_vehicle_options):
    ttk.Radiobutton(root, text=label, variable=target_vehicle_var, value=value).grid(row=6+len(weather_options)+i, column=1, padx=10, pady=2, sticky="w")

divider2 = ttk.Separator(root, orient='horizontal')
divider2.grid(row=8+len(weather_options)+len(target_vehicle_options), column=0, columnspan=3, padx=5, pady=8, sticky="ew")

driving_mode_label = ttk.Label(root, text="Driving Mode:")
driving_mode_label.grid(row=9+len(weather_options)+len(target_vehicle_options), column=0, padx=10, pady=5, sticky="w")
driving_mode_var = tk.StringVar(value="Manual Driving")
driving_mode_options = ["Manual Driving", "ADAS Driving"]
for i, option in enumerate(driving_mode_options):
    ttk.Radiobutton(root, text=option, variable=driving_mode_var, value=option).grid(row=9+len(weather_options)+len(target_vehicle_options)+i, column=1, padx=10, pady=2, sticky="w")

divider3 = ttk.Separator(root, orient='horizontal')
divider3.grid(row=11+len(weather_options)+len(target_vehicle_options)+len(driving_mode_options), column=0, columnspan=3, padx=5, pady=8, sticky="ew")

approaching_speed_label = ttk.Label(root, text="Approaching Speed:")
approaching_speed_label.grid(row=12+len(weather_options)+len(target_vehicle_options)+len(driving_mode_options), column=0, padx=10, pady=5, sticky="w")
approaching_speed_var = tk.StringVar(value="10")
approaching_speed_options = ["10", "40", "70"]
for i, option in enumerate(approaching_speed_options):
    ttk.Radiobutton(root, text=option, variable=approaching_speed_var, value=option).grid(row=12+len(weather_options)+len(target_vehicle_options)+len(driving_mode_options)+i, column=1, padx=10, pady=2, sticky="w")

row_for_buttons = 15+len(weather_options)+len(target_vehicle_options)+len(driving_mode_options)+len(approaching_speed_options)
start_button = ttk.Button(root, text="Start", command=start_experiment)
start_button.grid(row=row_for_buttons, column=0, padx=10, pady=10)

reset_button = ttk.Button(root, text="Reset", command=reset_experiment)
reset_button.grid(row=row_for_buttons, column=1, padx=10, pady=10)

root.mainloop()