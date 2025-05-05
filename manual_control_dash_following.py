#!/usr/bin/env python

import glob
import os
import sys
import random
import argparse
import math
import carla
import pygame
from manual_control_dash import World, HUD, KeyboardControl, get_actor_blueprints


class FollowingWorld(World):
    def __init__(self, client, sim_world, hud, args):
        self.client = client  # client 객체 저장
        self.follow_camera = None  # Initialize follow_camera to None
        super().__init__(sim_world, hud, args)
        self.server_fps = 0  # Initialize server_fps with default value
        self.simulation_time = 0.0  # Initialize simulation_time

    def tick(self, clock):
        # Update server FPS
        fixed_delta_seconds = self.client.get_world().get_settings().fixed_delta_seconds
        if fixed_delta_seconds and fixed_delta_seconds > 0:  # Ensure fixed_delta_seconds is valid
            self.server_fps = 1.0 / fixed_delta_seconds
        else:
            self.server_fps = 0  # Fallback to 0 if fixed_delta_seconds is invalid

        # Update simulation time
        self.simulation_time = self.client.get_world().get_snapshot().timestamp.elapsed_seconds

        # Call parent tick method
        super().tick(clock)

    def restart(self):
        # Dodge Charger로 플레이어 차량 설정
        dodge_blueprint = self.world.get_blueprint_library().find('vehicle.dodge.charger_2020')
        dodge_blueprint.set_attribute('role_name', self.actor_role_name)
        if dodge_blueprint.has_attribute('color'):
            color = random.choice(dodge_blueprint.get_attribute('color').recommended_values)
            dodge_blueprint.set_attribute('color', color)
        self._actor_filter = 'vehicle.dodge.charger_2020'  # 필터를 Dodge Charger로 고정

        # 플레이어 차량 생성
        spawn_points = self.map.get_spawn_points()
        spawn_point = random.choice(spawn_points) if spawn_points else carla.Transform()
        self.player = self.world.try_spawn_actor(dodge_blueprint, spawn_point)
        if not self.player:
            print("Failed to spawn Dodge Charger.")
            return

        # 센서와 HUD 초기화
        super().restart()

        # 목표 차량 생성
        player_transform = self.player.get_transform()

        # 도로 진행 방향에 따라 목표 차량 위치 계산
        yaw = math.radians(player_transform.rotation.yaw)  # yaw를 라디안으로 변환
        offset_distance = 10.0  # 목표 차량과의 거리 (10미터 앞)
        target_location = carla.Location(
            x=player_transform.location.x + offset_distance * math.cos(yaw),
            y=player_transform.location.y + offset_distance * math.sin(yaw),
            z=player_transform.location.z
        )
        target_transform = carla.Transform(
            target_location,
            player_transform.rotation  # 동일한 방향 유지
        )

        target_blueprint = random.choice(
            get_actor_blueprints(self.world, 'vehicle.*', self._actor_generation)  # 모든 차량 블루프린트에서 선택
        )

        self.target_vehicle = self.world.try_spawn_actor(target_blueprint, target_transform)
        if self.target_vehicle:
            print(f"Target vehicle spawned successfully at: {target_transform}")

            # 자율 주행 활성화
            traffic_manager = self.client.get_trafficmanager()  # client에서 traffic_manager 가져오기
            self.target_vehicle.set_autopilot(True, traffic_manager.get_port())  # 목표 차량 자율주행 활성화

            # 속도 제한 설정 (TrafficManager 사용)
            traffic_manager.vehicle_percentage_speed_difference(self.target_vehicle, 0)  # 100% 속도 유지

        else:
            print("Failed to spawn target vehicle at:", target_transform)

        # 카메라를 플레이어 차량에 고정
        self.setup_follow_camera()

    def setup_follow_camera(self):
        """플레이어 차량 뒤에 카메라를 배치"""
        camera_blueprint = self.world.get_blueprint_library().find('sensor.camera.rgb')
        camera_blueprint.set_attribute('image_size_x', '1280')
        camera_blueprint.set_attribute('image_size_y', '720')
        camera_blueprint.set_attribute('fov', '110')  # 넓은 시야각 설정

        camera_transform = carla.Transform(
            carla.Location(x=-5.5, z=2.5),  # 플레이어 차량의 뒤쪽 위에 카메라 배치
            carla.Rotation(pitch=8.0)  # 약간 아래를 바라보도록 설정
        )
        if self.player:
            self.follow_camera = self.world.spawn_actor(camera_blueprint, camera_transform, attach_to=self.player)
            print("Follow camera attached to player vehicle.")

    def follow_target(self):
        """플레이어 차량이 목표 차량을 따라가도록 설정"""
        # 목표 차량이나 플레이어 차량이 없는 경우 반환
        if self.target_vehicle is None or self.player is None:
            print("Error: Target vehicle or Player vehicle is not initialized!")
            return

        # 수동 제어 비활성화
        self.player.set_autopilot(False)

        # 목표 차량과 플레이어 차량의 위치 가져오기
        target_location = self.target_vehicle.get_transform().location
        player_location = self.player.get_transform().location

        # 두 차량 간 거리 계산
        distance = player_location.distance(target_location)

        # 목표 차량의 속도 얻기
        target_velocity = self.target_vehicle.get_velocity()
        target_speed = (target_velocity.x ** 2 + target_velocity.y ** 2 + target_velocity.z ** 2) ** 0.5  # 속도(m/s)

        print(f"Distance to target: {distance:.2f} meters, Target speed: {target_speed:.2f} m/s")

        # VehicleControl 객체 생성
        control = carla.VehicleControl()
        safe_distance = 10.0  # 목표 차량과의 안전 거리

        # 10미터 간격 유지 로직
        if distance > safe_distance + 5.0:  # 너무 멀리 떨어졌을 경우
            control.throttle = 1.0  # 최대 가속
            control.brake = 0.0
        elif distance > safe_distance:  # 적정 거리 유지
            control.throttle = min(0.8, target_speed / 5.0 + 0.3)
            control.brake = 0.0
        elif distance < safe_distance - 2.0:  # 너무 가까운 경우 감속
            control.throttle = 0.0
            control.brake = 0.5
        else:  # 안정적인 거리 유지
            control.throttle = 0.5
            control.brake = 0.0
        # 핸드브레이크 해제
        control.hand_brake = False

        print(f"Throttle: {control.throttle:.2f}, Brake: {control.brake:.2f}")

        # 플레이어 차량에 제어 명령 적용
        self.player.apply_control(control)

    def destroy(self):
        """월드 종료 시 카메라와 차량 정리"""
        if hasattr(self, 'follow_camera') and self.follow_camera:  # Ensure follow_camera exists before destroying
            self.follow_camera.destroy()
            self.follow_camera = None
        super().destroy()


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

            traffic_manager = client.get_trafficmanager()
            #traffic_manager.set_synchronous_mode(True)
            traffic_manager.vehicle_percentage_speed_difference(self.target_vehicle, -30)  # Target vehicle moves 30% faster

        display = pygame.display.set_mode(
            (args.width, args.height),
            pygame.HWSURFACE | pygame.DOUBLEBUF)
        hud = HUD(args.width, args.height)
        world = FollowingWorld(client, sim_world, hud, args)  # client 객체 전달
        controller = KeyboardControl(world, args.autopilot)

        if args.sync:
            sim_world.tick()
        else:
            sim_world.wait_for_tick()

        clock = pygame.time.Clock()
        while True:
            if args.sync:
                sim_world.tick()
            clock.tick_busy_loop(60)

            # 이벤트 처리
            if controller.parse_events(client, world, clock, args.sync):
                return

            # HUD 및 렌더링
            world.tick(clock)
            world.render(display)
            pygame.display.flip()

            # 목표 차량 추적
            world.follow_target()

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