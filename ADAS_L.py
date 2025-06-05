#!/usr/bin/env python

import argparse
import math
import random
import carla
import pygame
import os
import json
import time
import numpy as np
from manual_control_dash import World, HUD

def load_polyline(filename):
    try:
        with open(filename, "r") as f:
            data = json.load(f)
        return [carla.Location(x=pt["x"], y=pt["y"], z=pt.get("z", 0.3)) for pt in data]
    except Exception as e:
        print(f"[WARNING] Polyline loading failed: {e}")
        return None

def set_vehicle_lights(vehicle, brake_on=False):
    state = (
        carla.VehicleLightState.Position |
        carla.VehicleLightState.LowBeam |
        carla.VehicleLightState.HighBeam |
        carla.VehicleLightState.Interior
    )
    if hasattr(carla.VehicleLightState, "Rear"):
        state |= carla.VehicleLightState.Rear
    if brake_on:
        state |= carla.VehicleLightState.Brake
    try:
        vehicle.set_light_state(carla.VehicleLightState(state))
    except Exception:
        pass

def mph_to_mps(mph):
    return mph * 0.44704

def cleanup_all(world, sensors_from_class=None):
    actors = world.get_actors()
    destroyed_ids = set()
    if sensors_from_class:
        for sensor in sensors_from_class:
            try:
                sensor.stop()
            except Exception:
                pass
            try:
                sensor.destroy()
                destroyed_ids.add(sensor.id)
            except Exception:
                pass
        sensors_from_class.clear()
    for sensor in actors.filter('sensor.*'):
        if sensor.id in destroyed_ids:
            continue
        try:
            sensor.stop()
        except Exception:
            pass
        try:
            sensor.destroy()
        except Exception:
            pass
    time.sleep(0.2)
    for filter_str in ['walker.*', 'vehicle.*']:
        for actor in actors.filter(filter_str):
            if actor.id in destroyed_ids:
                continue
            try:
                actor.destroy()
            except Exception:
                pass
    world.tick()
    time.sleep(0.2)

class PolylineFollowingWorld(World):
    def __init__(self, client, sim_world, hud, args, polyline):
        self.client = client
        self.sensors = []
        self.target_vehicle = None
        self.args = args
        self.paused = getattr(args, 'paused', False)
        self.motion_enabled = not self.paused
        self.respawn_requested = False
        self.polyline = polyline
        self.server_fps = 0
        self.simulation_time = 0.0
        self.lead_speed_mph = int(args.approaching_speed)
        self.lead_vehicle_model = args.target_vehicle
        self.last_status_msg = ""
        self.spawn_frame_count = 0
        self.last_steer = 0.0
        super().__init__(sim_world, hud, args)

    def tick(self, clock):
        try:
            fixed_delta_seconds = self.client.get_world().get_settings().fixed_delta_seconds
            self.server_fps = 1.0 / fixed_delta_seconds if fixed_delta_seconds and fixed_delta_seconds > 0 else 0
        except Exception:
            self.server_fps = 0
        try:
            self.simulation_time = self.client.get_world().get_snapshot().timestamp.elapsed_seconds
        except Exception:
            self.simulation_time = 0.0
        super().tick(clock)
        if self.respawn_requested:
            self.restart()
            self.respawn_requested = False
        if self.spawn_frame_count > 0:
            self.spawn_frame_count -= 1
            if self.player:
                self.player.set_transform(self.ego_spawn_tf)
                self.player.apply_control(carla.VehicleControl(steer=0.0, throttle=0.0, brake=1.0))
            if self.target_vehicle:
                self.target_vehicle.set_transform(self.lead_spawn_tf)
                self.target_vehicle.apply_control(carla.VehicleControl(steer=0.0, throttle=0.0, brake=1.0))

    def restart(self):
        cleanup_all(self.world, self.sensors)
        time.sleep(0.2)
        world = self.world
        blueprint_library = world.get_blueprint_library()
        carla_map = world.get_map()
        polyline = self.polyline

        if not polyline:
            print("[WARNING] No polyline: Spawning vehicle at a default location.")
            spawn_points = carla_map.get_spawn_points()
            ego_tf = spawn_points[0] if spawn_points else carla.Transform()
        else:
            first = polyline[0]
            ego_wp = carla_map.get_waypoint(first, project_to_road=True, lane_type=carla.LaneType.Driving)
            if ego_wp is None:
                print("[ERROR] Polyline first waypoint not on road. Coordinates:", first)
                return
            ego_tf = ego_wp.transform
            ego_tf.location.z += 0.4

        ego_bp = blueprint_library.find('vehicle.dodge.charger_2020')
        ego_bp.set_attribute('role_name', self.actor_role_name)
        if ego_bp.has_attribute('color'):
            color = random.choice(ego_bp.get_attribute('color').recommended_values)
            ego_bp.set_attribute('color', color)
        self.player = self._force_spawn_actor(ego_bp, ego_tf, max_attempts=10)
        if not self.player:
            print(f"[ERROR] Failed to spawn ego vehicle at: {ego_tf.location}")
            return
        self.player.apply_control(carla.VehicleControl(hand_brake=False))
        self.ego_spawn_tf = ego_tf

        if polyline:
            lead_wp_list = ego_wp.next(40.0)
            if not lead_wp_list:
                print("[ERROR] Could not find waypoint 40m ahead.")
                return
            lead_wp = lead_wp_list[0]
            lead_tf = lead_wp.transform
            lead_tf.location.z += 0.4
            self.lead_spawn_tf = carla.Transform(lead_tf.location, lead_tf.rotation)
            lead_bp = blueprint_library.find(self.lead_vehicle_model)
            lead_bp.set_attribute('role_name', 'target')
            if lead_bp.has_attribute('color'):
                color = random.choice(lead_bp.get_attribute('color').recommended_values)
                lead_bp.set_attribute('color', color)
            self.target_vehicle = self._force_spawn_actor(lead_bp, self.lead_spawn_tf, max_attempts=10)
            if not self.target_vehicle:
                print(f"[ERROR] Failed to spawn lead vehicle at: {self.lead_spawn_tf.location}")
                return

        set_vehicle_lights(self.target_vehicle, brake_on=False)
        set_vehicle_lights(self.player, brake_on=False)
        self.spawn_frame_count = 2
        self.last_steer = 0.0
        try:
            self.setup_driver_camera()
        except Exception as e:
            print(f"[ERROR] Camera setup failed: {e}")

    def _is_location_free(self, world, location, threshold=2.0):
        vehicles = world.get_actors().filter('vehicle.*')
        for vehicle in vehicles:
            if vehicle.get_location().distance(location) < threshold:
                return False
        return True

    def _force_spawn_actor(self, blueprint, transform, max_attempts=10):
        for _ in range(max_attempts):
            if self._is_location_free(self.world, transform.location):
                actor = self.world.try_spawn_actor(blueprint, transform)
                if actor:
                    return actor
            transform.location.y += 1.0  # Adjust position slightly to retry
        return None

    def stanley_control_smooth(self, vehicle, lead_vehicle=None, lookahead=5.0, target_speed=31.3):
        carla_map = self.world.get_map()
        vehicle_loc = vehicle.get_location()
        vehicle_tf = vehicle.get_transform()
        yaw = np.deg2rad(vehicle_tf.rotation.yaw)
        v = np.hypot(vehicle.get_velocity().x, vehicle.get_velocity().y)

        if lead_vehicle:
            lead_loc = lead_vehicle.get_location()
            lead_wp = carla_map.get_waypoint(lead_loc, project_to_road=True)
            target_wp = lead_wp.next(lookahead)[0]
        else:
            wp = carla_map.get_waypoint(vehicle_loc, project_to_road=True, lane_type=carla.LaneType.Driving)
            target_wp = wp.next(lookahead)[0]

        path_yaw = np.deg2rad(target_wp.transform.rotation.yaw)
        heading_error = path_yaw - yaw
        heading_error = math.atan2(math.sin(heading_error), math.cos(heading_error))
        dx = vehicle_loc.x - target_wp.transform.location.x
        dy = vehicle_loc.y - target_wp.transform.location.y
        lateral_error = -math.sin(path_yaw) * dx + math.cos(path_yaw) * dy

        steer_limit = 0.005  # Reduce sensitivity
        steer = 0.05 * heading_error + math.atan2(0.05 * lateral_error, max(v, 2.5))
        steer = np.clip(steer, -steer_limit, steer_limit)

        max_delta = 0.002  # Smooth steering adjustments
        steer = np.clip(steer, self.last_steer - max_delta, self.last_steer + max_delta)
        self.last_steer = steer

        throttle = 0.85 if v < target_speed else 0.0
        brake = 0.2 if v > target_speed + 2 else 0.0

        self.last_status_msg = f"v={v:.1f}m/s lat_err={lateral_error:.3f} steer={steer:.4f} stlim={steer_limit:.3f}"
        return carla.VehicleControl(steer=steer, throttle=throttle, brake=brake)

    def lead_vehicle_control(self):
        if not self.target_vehicle or self.spawn_frame_count > 0:
            return
        ctrl = self.stanley_control_smooth(
            self.target_vehicle,
            lookahead=8.0,
            target_speed=mph_to_mps(70)
        )
        self.target_vehicle.apply_control(ctrl)

    def ego_vehicle_control(self):
        if not self.player or self.spawn_frame_count > 0:
            return
        ctrl = self.stanley_control_smooth(
            self.player,
            lead_vehicle=self.target_vehicle,
            lookahead=10.0,
            target_speed=mph_to_mps(70)
        )
        self.player.apply_control(ctrl)

    def setup_driver_camera(self):
        try:
            camera_blueprint = self.world.get_blueprint_library().find('sensor.camera.rgb')
            camera_blueprint.set_attribute('image_size_x', '1280')
            camera_blueprint.set_attribute('image_size_y', '720')
            camera_blueprint.set_attribute('fov', '110')
            camera_blueprint.set_attribute('sensor_tick', '0.05')
            camera_transform = carla.Transform(carla.Location(x=0.5, y=-0.3, z=1.2), carla.Rotation(pitch=0.0, yaw=0.0, roll=0.0))
            if self.player:
                self.driver_camera = self.world.spawn_actor(camera_blueprint, camera_transform, attach_to=self.player)
                if self.driver_camera:
                    self.sensors.append(self.driver_camera)
                    self.camera_surface = None
                    self.driver_camera.listen(self._on_camera_image)
        except Exception as e:
            print(f"[ERROR] Camera setup failed: {e}")

    def _on_camera_image(self, image):
        array = np.frombuffer(image.raw_data, dtype=np.uint8)
        array = array.reshape((image.height, image.width, 4))
        array = array[:, :, :3][:, :, ::-1]
        surf = pygame.surfarray.make_surface(array.swapaxes(0, 1))
        self.camera_surface = surf

    def render(self, display):
        if hasattr(self, 'camera_surface') and self.camera_surface is not None:
            display.blit(self.camera_surface, (0, 0))
        self.hud.render(display)
        font = pygame.font.SysFont("monospace", 22)
        label = font.render(self.last_status_msg, 1, (255, 255, 0))
        display.blit(label, (10, 40))

    def destroy(self):
        cleanup_all(self.world, self.sensors)
        try:
            super().destroy()
        except Exception:
            pass

def game_loop(args):
    pygame.init()
    pygame.font.init()
    world = None
    original_settings = None
    polyline = load_polyline(args.polyline)
    try:
        client = carla.Client(args.host, args.port)
        client.set_timeout(2000.0)
        sim_world = client.get_world()
        if args.sync:
            original_settings = sim_world.get_settings()
            settings = sim_world.get_settings()
            if not settings.synchronous_mode:
                settings.synchronous_mode = True
                settings.fixed_delta_seconds = 0.05
            sim_world.apply_settings(settings)
        display = pygame.display.set_mode(
            (args.width, args.height),
            pygame.HWSURFACE | pygame.DOUBLEBUF)
        hud = HUD(args.width, args.height)
        world = PolylineFollowingWorld(client, sim_world, hud, args, polyline)
        hud.toggle_info()
        if args.sync:
            sim_world.tick()
        else:
            sim_world.wait_for_tick()
        clock = pygame.time.Clock()
        running = True
        while running:
            pygame.event.pump()
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False
                    break
            if args.sync:
                sim_world.tick()
            clock.tick_busy_loop(60)
            world.tick(clock)
            world.render(display)
            pygame.display.flip()
            world.lead_vehicle_control()
            world.ego_vehicle_control()
    finally:
        if original_settings:
            sim_world.apply_settings(original_settings)
        if world is not None:
            world.destroy()
        pygame.quit()

def main():
    argparser = argparse.ArgumentParser(
        description='CARLA ADAS_L Polyline Experiment (Aligned Lane Driving)')
    argparser.add_argument('--host', metavar='H', default='127.0.0.1', help='IP of the host server (default: 127.0.0.1)')
    argparser.add_argument('-p', '--port', metavar='P', default=2000, type=int, help='TCP port (default: 2000)')
    argparser.add_argument('--res', metavar='WIDTHxHEIGHT', default='1280x720', help='window resolution (default: 1280x720)')
    argparser.add_argument('--filter', metavar='PATTERN', default='vehicle.*', help='actor filter (default: "vehicle.*")')
    argparser.add_argument('--generation', metavar='G', default='2', help='actor generation (default: "2")')
    argparser.add_argument('--rolename', metavar='NAME', default='hero', help='actor role name (default: "hero")')
    argparser.add_argument('--gamma', default=2.2, type=float, help='Gamma correction (default: 2.2)')
    argparser.add_argument('--sync', action='store_true', help='Activate synchronous mode')
    argparser.add_argument('--target-vehicle', default='vehicle.mini.cooper_s_2021', help='Target vehicle blueprint (mini or MKZ)')
    argparser.add_argument('--approaching-speed', default='70', type=str, help='Approaching speed in mph (10, 40, 70)')
    argparser.add_argument('--paused', action='store_true', help='Spawn only, do not move until start.flag is present')
    argparser.add_argument('--polyline', default='my_polyline_L.json', help='Waypoint polyline json file')
    args = argparser.parse_args()
    args.width, args.height = [int(x) for x in args.res.split('x')]
    try:
        if os.path.exists("start.flag"):
            os.remove("start.flag")
        game_loop(args)
    except KeyboardInterrupt:
        print('\nCancelled by user. Bye!')

if __name__ == '__main__':
    main()