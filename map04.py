import carla
import time
import subprocess


def load_town(town_name="Town04"):
    client = carla.Client("localhost", 2000)
    client.set_timeout(10.0)

    world = client.get_world()
    current_map = world.get_map().name
    print(f"Current map: {current_map}")

    if town_name not in current_map:
        print(f"Loading map {town_name}...")
        client.load_world(town_name)
        print(f"Map {town_name} loaded successfully!")
    else:
        print(f"Map {town_name} is already loaded.")


def run_manual_control():
    print("Running manual_control.py...")
    subprocess.run(["python", "manual_control.py"])


def main():
    load_town("Town04")

    # Optional: wait for world to stabilize
    time.sleep(2)

    run_manual_control()


if __name__ == "__main__":
    main()