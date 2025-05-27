#!/usr/bin/env python

import glob
import os
import sys
import random
import argparse
import math
import carla
import pygame
import time
from manual_control_dash import World, HUD

def set_all_traffic_lights_to_green(world):
    traffic_lights = world.get_actors().filter('traffic.traffic_light')
    for light in traffic_lights:
        light.set_state(carla.TrafficLightState.Green)
        light.set_green_time(9999)
        light.set_red_time(0)
        light.set_yellow_time(0)

def get_curvature(wp1, wp2, wp3):
    x1, y1 = wp1.transform.location.x, wp1.transform.location.y
    x2, y2 = wp2.transform.location.x, wp2.transform.location.y
    x3, y3 = wp3.transform.location.x, wp3.transform.location.y
    try:
        k_num = 2 * abs((x2-x1)*(y3-y1)-(y2-y1)*(x3-x1))
        k_den = math.sqrt(((x2-x1)**2 + (y2-y1)**2) * ((x3-x2)**2 + (y3-y2)**2) * ((x1-x3)**2 + (y1-y3)**2))
        if k_den == 0:
            return 0.0
        curvature = k_num / k_den
        return curvature
    except Exception:
        return 0.0

class FollowingWorld(World):
    def __init__(self, client, sim_world, hud, args):
        self.client = client
        self.follow_camera = None
        self.sensors = []
        self.target_speed_mps = 0.0
        self.target_vehicle = None
        self.lead_target_speed_mps = 0.0
        self.last_log_time = 0
        self.lead_force_stop = False  # 앞차 강제정지 플래그
        self.ego_follow_mode = True   # 내차가 앞차를 따라가는지 여부
        self.ego_force_brake = False  # 내차 수동 브레이크 플래그
        self.sim_start_time = None    # 시뮬레이션 시작 시간 (초)
        super().__init__(sim_world, hud, args)
        self.server_fps = 0
        self.simulation_time = 0.0

    def tick(self, clock):
        fixed_delta_seconds = self.client.get_world().get_settings().fixed_delta_seconds
        if fixed_delta_seconds and fixed_delta_seconds > 0:
            self.server_fps = 1.0 / fixed_delta_seconds
        else:
            self.server_fps = 0
        self.simulation_time = self.client.get_world().get_snapshot().timestamp.elapsed_seconds

        # Simulation 시작 시간 저장 (최초 1회)
        if self.sim_start_time is None:
            self.sim_start_time = self.simulation_time
        super().tick(clock)

    def restart(self):
        carla_map = self.map

        # 1. 내차를 배열 좌표 중 랜덤 선택해서 차선위에 스폰
        spawn_points = [
            carla.Location(x=3637.5, y=3093.4, z=371),
        ]
        spawn_location = random.choice(spawn_points)
        wp_player = carla_map.get_waypoint(spawn_location, project_to_road=True, lane_type=carla.LaneType.Driving)
        if wp_player is not None:
            spawn_transform = wp_player.transform
            spawn_transform.location.z += 0.5
        else:
            print("[ERROR] 지정 위치에서 차선 waypoint를 찾지 못했습니다. 기본 스폰포인트 사용.")
            all_spawn_points = self.map.get_spawn_points()
            spawn_transform = random.choice(all_spawn_points) if all_spawn_points else carla.Transform()

        # 2. 내차 스폰
        dodge_blueprint = self.world.get_blueprint_library().find('vehicle.dodge.charger_2020')
        dodge_blueprint.set_attribute('role_name', self.actor_role_name)
        if dodge_blueprint.has_attribute('color'):
            color = random.choice(dodge_blueprint.get_attribute('color').recommended_values)
            dodge_blueprint.set_attribute('color', color)
        self._actor_filter = 'vehicle.dodge.charger_2020'
        self.player = self.world.try_spawn_actor(dodge_blueprint, spawn_transform)
        if not self.player:
            print("[ERROR] Failed to spawn Dodge Charger 2020 at given point.")
            return
        self.player.apply_control(carla.VehicleControl(hand_brake=False))

        set_all_traffic_lights_to_green(self.world)
        super().restart()

        # 3. 앞차 스폰: 내차와 같은 차선, 30~1m 앞까지 차례로 시도 (직진만 허용)
        player_transform = self.player.get_transform()
        wp_player = carla_map.get_waypoint(player_transform.location, project_to_road=True, lane_type=carla.LaneType.Driving)
        if wp_player is None:
            print("[ERROR] 내차가 차선 위에 없음(앞차 스폰 불가)")
            return

        mini_blueprint = self.world.get_blueprint_library().find('vehicle.mini.cooper_s_2021')
        mini_blueprint.set_attribute('role_name', 'target')
        if mini_blueprint.has_attribute('color'):
            color = random.choice(mini_blueprint.get_attribute('color').recommended_values)
            mini_blueprint.set_attribute('color', color)
        forward_vector = wp_player.transform.get_forward_vector()
        offset_distances = [30.0, 25.0, 20.0, 15.0, 10.0, 5.0, 3.0, 1.0]
        self.target_vehicle = None
        for offset_distance in offset_distances:
            target_location = carla.Location(
                x=wp_player.transform.location.x + offset_distance * forward_vector.x,
                y=wp_player.transform.location.y + offset_distance * forward_vector.y,
                z=wp_player.transform.location.z
            )
            wp_target = carla_map.get_waypoint(target_location, project_to_road=True, lane_type=carla.LaneType.Driving)
            if wp_target is None or wp_target.road_id != wp_player.road_id or wp_target.lane_id * wp_player.lane_id <= 0:
                continue
            target_transform = wp_target.transform
            target_transform.location.z += 0.5
            self.target_vehicle = self.world.try_spawn_actor(mini_blueprint, target_transform)
            if self.target_vehicle:
                print(f"Target vehicle spawned at: {target_transform}")
                self.set_target_vehicle_speed(70)
                break
        if not self.target_vehicle:
            print("[ERROR] Failed to spawn target vehicle at any nearby location. Trying fallback...")
            for offset_distance in range(1, 40):
                target_location = carla.Location(
                    x=wp_player.transform.location.x + offset_distance * forward_vector.x,
                    y=wp_player.transform.location.y + offset_distance * forward_vector.y,
                    z=wp_player.transform.location.z
                )
                wp_target = carla_map.get_waypoint(target_location, project_to_road=True, lane_type=carla.LaneType.Driving)
                if wp_target is None:
                    continue
                target_transform = wp_target.transform
                target_transform.location.z += 0.5
                self.target_vehicle = self.world.try_spawn_actor(mini_blueprint, target_transform)
                if self.target_vehicle:
                    print(f"Target vehicle spawned at: {target_transform} (fallback mode)")
                    self.set_target_vehicle_speed(70)
                    break

        if self.player is not None and isinstance(self.player, carla.Vehicle):
            all_lights = (
                    carla.VehicleLightState.Position |
                    carla.VehicleLightState.LowBeam |
                    carla.VehicleLightState.HighBeam |
                    carla.VehicleLightState.Brake |
                    carla.VehicleLightState.RightBlinker |
                    carla.VehicleLightState.LeftBlinker |
                    carla.VehicleLightState.Reverse |
                    carla.VehicleLightState.Fog |
                    carla.VehicleLightState.Interior |
                    carla.VehicleLightState.Special1 |
                    carla.VehicleLightState.Special2 |
                    carla.VehicleLightState.All
            )
            self.player.set_light_state(carla.VehicleLightState(all_lights))
        self.set_vehicle_lights()
        try:
            self.setup_follow_camera()
        except RuntimeError as e:
            print(f"[ERROR] Follow camera setup failed: {e}")

    def set_target_vehicle_speed(self, speed_mph):
        if self.target_vehicle is None:
            print("[ERROR] Target vehicle is not initialized!")
            return
        self.target_vehicle.set_autopilot(False)
        self.lead_target_speed_mps = speed_mph * 0.44704
        self.target_speed_mps = self.lead_target_speed_mps

    def log_status(self, lead_data, ego_data, distance, rel_speed):
        # 시뮬레이션 시작 이후 경과 시간(ms)로 첫 컬럼 기록
        sim_time_msec = int((self.simulation_time - self.sim_start_time) * 1000)
        log_line = (
            f"{sim_time_msec}, "
            f"{lead_data['pos_x']:.1f},{lead_data['pos_y']:.1f},"
            f"{lead_data['speed_mph']:.2f},{lead_data['steer']:.2f},{lead_data['throttle']:.2f},{lead_data['brake']:.2f}, "
            f"{ego_data['pos_x']:.1f},{ego_data['pos_y']:.1f},"
            f"{ego_data['speed_mph']:.2f},{ego_data['steer']:.2f},{ego_data['throttle']:.2f},{ego_data['brake']:.2f}, "
            f"{distance:.2f},{rel_speed:.2f}"
        )
        print(log_line)
        # with open("carla_log.txt", "a") as f:
        #     print(log_line, file=f)

    def lead_vehicle_cruise_and_curve(self):
        if self.target_vehicle is None or self.player is None:
            return
        map_inst = self.world.get_map()
        cur_location = self.target_vehicle.get_location()
        cur_wp = map_inst.get_waypoint(cur_location, project_to_road=True, lane_type=carla.LaneType.Driving)
        if not cur_wp:
            return

        next_wps = cur_wp.next(10.0)
        if not next_wps:
            return
        next_wp = next_wps[0]

        vehicle_transform = self.target_vehicle.get_transform()
        vehicle_yaw = vehicle_transform.rotation.yaw
        wp_yaw = next_wp.transform.rotation.yaw
        steer_angle = (wp_yaw - vehicle_yaw + 180) % 360 - 180
        steer = max(-1.0, min(1.0, steer_angle / 45.0))

        velocity = self.target_vehicle.get_velocity()
        speed = math.sqrt(velocity.x ** 2 + velocity.y ** 2 + velocity.z ** 2)
        speed_mph = speed * 2.23694

        # --- 강제 정지(0키) 처리 ---
        if self.lead_force_stop:
            speed_target = 0.0  # 0mph
        else:
            speed_target = 70 * 0.44704  # 70mph in m/s

        control = carla.VehicleControl()
        speed_error = speed_target - speed
        if speed < speed_target - 0.5:
            control.throttle = min(1.0, speed_error / 6.0)
            control.brake = 0.0
        elif speed > speed_target + 0.5:
            control.throttle = 0.0
            control.brake = min(1.0, (speed - speed_target) / 4.0)
        else:
            control.throttle = 0.0
            control.brake = 0.0
        control.steer = steer

        ego_transform = self.player.get_transform()
        ego_velocity = self.player.get_velocity()
        ego_speed = math.sqrt(ego_velocity.x ** 2 + ego_velocity.y ** 2 + ego_velocity.z ** 2)
        ego_speed_mph = ego_speed * 2.23694
        ego_steer = getattr(self, '_last_ego_steer', 0.0)
        ego_throttle = getattr(self, '_last_ego_throttle', 0.0)
        ego_brake = getattr(self, '_last_ego_brake', 0.0)
        distance = ego_transform.location.distance(vehicle_transform.location)
        rel_speed = speed - ego_speed

        lead_data = {
            "pos_x": vehicle_transform.location.x,
            "pos_y": vehicle_transform.location.y,
            "speed_mph": speed_mph,
            "steer": control.steer,
            "throttle": control.throttle,
            "brake": control.brake
        }
        ego_data = {
            "pos_x": ego_transform.location.x,
            "pos_y": ego_transform.location.y,
            "speed_mph": ego_speed_mph,
            "steer": ego_steer,
            "throttle": ego_throttle,
            "brake": ego_brake
        }
        self.log_status(lead_data, ego_data, distance, rel_speed)

        self.target_vehicle.apply_control(control)
        self._last_lead_steer = control.steer
        self._last_lead_throttle = control.throttle
        self._last_lead_brake = control.brake

    def ego_vehicle_cruise_and_follow(self):
        if self.player is None or self.target_vehicle is None:
            return
        map_inst = self.world.get_map()
        cur_location = self.player.get_location()
        cur_wp = map_inst.get_waypoint(cur_location, project_to_road=True, lane_type=carla.LaneType.Driving)
        if not cur_wp:
            return
        next_wps = cur_wp.next(10.0)
        if not next_wps:
            return
        next_wp = next_wps[0]

        vehicle_transform = self.player.get_transform()
        vehicle_yaw = vehicle_transform.rotation.yaw
        wp_yaw = next_wp.transform.rotation.yaw
        steer_angle = (wp_yaw - vehicle_yaw + 180) % 360 - 180
        steer = max(-1.0, min(1.0, steer_angle / 45.0))

        target_transform = self.target_vehicle.get_transform()
        player_transform = self.player.get_transform()
        distance = player_transform.location.distance(target_transform.location)

        velocity = self.player.get_velocity()
        speed = math.sqrt(velocity.x ** 2 + velocity.y ** 2 + velocity.z ** 2)
        speed_mph = speed * 2.23694
        target_velocity = self.target_vehicle.get_velocity()
        target_speed = math.sqrt(target_velocity.x ** 2 + target_velocity.y ** 2 + target_velocity.z ** 2)
        rel_speed = target_speed - speed

        control = carla.VehicleControl()
        # --- 내차 수동 브레이크 모드 ---
        if not self.ego_follow_mode:
            if self.ego_force_brake:
                control.throttle = 0.0
                control.brake = 1.0
            else:
                control.throttle = 0.0
                control.brake = 0.0
        else:
            # 평소/기존: 앞차와 거리 유지
            desired_distance = 20.0
            min_distance = 10.0
            max_distance = 30.0
            distance_error = distance - desired_distance
            speed_error = target_speed - speed
            acc = 0.25 * distance_error + 0.5 * speed_error

            if distance < min_distance:
                control.throttle = 0.0
                control.brake = 1.0
            elif acc > 0.2:
                control.throttle = min(1.0, acc / 4.0)
                control.brake = 0.0
            elif acc < -0.2:
                control.throttle = 0.0
                control.brake = min(1.0, -acc / 3.0)
            else:
                control.throttle = 0.0
                control.brake = 0.0
        control.steer = steer

        self._last_ego_steer = control.steer
        self._last_ego_throttle = control.throttle
        self._last_ego_brake = control.brake

        self.player.apply_control(control)

    def set_vehicle_lights(self):
        basic_lights = (
                carla.VehicleLightState.Position |
                carla.VehicleLightState.LowBeam |
                carla.VehicleLightState.HighBeam |
                carla.VehicleLightState.Interior |
                carla.VehicleLightState.Brake
        )
        if self.player:
            self.player.set_light_state(carla.VehicleLightState(basic_lights))
        if self.target_vehicle:
            self.target_vehicle.set_light_state(carla.VehicleLightState(basic_lights))

    def setup_follow_camera(self):
        try:
            camera_blueprint = self.world.get_blueprint_library().find('sensor.camera.rgb')
            camera_blueprint.set_attribute('image_size_x', '1280')
            camera_blueprint.set_attribute('image_size_y', '720')
            camera_blueprint.set_attribute('fov', '110')
            camera_transform = carla.Transform(carla.Location(x=-5.5, z=2.5), carla.Rotation(pitch=8.0))
            if self.player:
                self.follow_camera = self.world.spawn_actor(camera_blueprint, camera_transform, attach_to=self.player)
                if self.follow_camera:
                    self.sensors.append(self.follow_camera)
            else:
                print("[ERROR] Player vehicle not initialized, cannot attach follow camera.")
        except Exception as e:
            print(f"[ERROR] Exception occurred in setup_follow_camera: {e}")

    def destroy(self):
        for sensor in self.sensors:
            if sensor.is_alive:
                try:
                    sensor.destroy()
                except RuntimeError:
                    pass
        self.sensors = []
        if hasattr(self, 'target_vehicle') and self.target_vehicle:
            if self.target_vehicle.is_alive:
                try:
                    self.target_vehicle.destroy()
                except RuntimeError:
                    pass
        if hasattr(self, 'player') and self.player:
            if self.player.is_alive:
                try:
                    self.player.destroy()
                except RuntimeError:
                    pass
        try:
            super().destroy()
        except RuntimeError:
            pass

def game_loop(args):
    pygame.init()
    pygame.font.init()
    world = None
    original_settings = None
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
        world = FollowingWorld(client, sim_world, hud, args)
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
                elif event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_ESCAPE:
                        running = False
                        break
                    elif event.key == pygame.K_0:  # "0"키: 앞차 강제정지 + 내차 거리유지 해제
                        if world:
                            world.lead_force_stop = True
                            world.ego_follow_mode = False
                            print("[INFO] 앞차 강제정지(0키), 내차 거리유지 해제!")
                    elif event.key == pygame.K_s:  # "S"키: 내차 브레이크
                        if world:
                            world.ego_force_brake = True
                            print("[INFO] 내차 브레이크(S키)!")
                elif event.type == pygame.KEYUP:
                    if event.key == pygame.K_s:
                        if world:
                            world.ego_force_brake = False
                            print("[INFO] 내차 브레이크 해제(S키 뗌)")
                elif event.type == pygame.ACTIVEEVENT:
                    if event.gain == 0:
                        print("윈도우 포커스 잃음(비활성화)")
                    elif event.gain == 1:
                        print("윈도우 포커스 복귀(활성화)")
            if args.sync:
                sim_world.tick()
            clock.tick_busy_loop(60)
            world.tick(clock)
            world.render(display)
            pygame.display.flip()
            world.lead_vehicle_cruise_and_curve()
            world.ego_vehicle_cruise_and_follow()
    finally:
        if original_settings:
            sim_world.apply_settings(original_settings)
        if world is not None:
            world.destroy()
        pygame.quit()

def main():
    argparser = argparse.ArgumentParser(
        description='CARLA Manual Control Client with Following Vehicle')
    argparser.add_argument(
        '-v', '--verbose',
        action='store_true',
        dest='debug',
        help='print debug information')
    argparser.add_argument(
        '--host',
        metavar='H',
        default='127.0.0.1',
        help='IP of the host server (default: 127.0.0.1)')
    argparser.add_argument(
        '-p', '--port',
        metavar='P',
        default=2000,
        type=int,
        help='TCP port to listen to (default: 2000)')
    argparser.add_argument(
        '-a', '--autopilot',
        action='store_true',
        help='enable autopilot')
    argparser.add_argument(
        '--res',
        metavar='WIDTHxHEIGHT',
        default='1280x720',
        help='window resolution (default: 1280x720)')
    argparser.add_argument(
        '--filter',
        metavar='PATTERN',
        default='vehicle.*',
        help='actor filter (default: "vehicle.*")')
    argparser.add_argument(
        '--generation',
        metavar='G',
        default='2',
        help='restrict to certain actor generation (values: "1","2","All" - default: "2")')
    argparser.add_argument(
        '--rolename',
        metavar='NAME',
        default='hero',
        help='actor role name (default: "hero")')
    argparser.add_argument(
        '--gamma',
        default=2.2,
        type=float,
        help='Gamma correction of the camera (default: 2.2)')
    argparser.add_argument(
        '--sync',
        action='store_true',
        help='Activate synchronous mode execution')
    args = argparser.parse_args()
    args.width, args.height = [int(x) for x in args.res.split('x')]
    try:
        game_loop(args)
    except KeyboardInterrupt:
        print('\nCancelled by user. Bye!')

if __name__ == '__main__':
    main()