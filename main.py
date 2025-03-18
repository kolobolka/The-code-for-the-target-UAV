# main.py
import os
import time
import numpy as np
import pygame
import serial
import board
import busio
from crypto_lite import encrypt
from gps_module import GPSReader
from lidar import RPLidarA2
from thermal import AMG8833
from lora import LoRaRadio
from servo import SimpleServo

class BudgetCombatDrone:
    def __init__(self):
        try:
            self.gps = GPSReader('/dev/ttyAMA0')
        except serial.SerialException:
            self.gps = None
            
        try:
            self.lidar = RPLidarA2('/dev/ttyUSB0')
        except serial.SerialException:
            self.lidar = None
            
        try:
            self.thermal = AMG8833()
        except Exception as e:
            self.thermal = None
            print(f"Thermal init error: {e}")
            
        self.radio = LoRaRadio(433e6)
        self.servo = SimpleServo(18)
        self.position = np.array([0.0, 0.0, 100.0], dtype=np.float32)
        self.health = 1.0
        self.armed = False
        self.geofence = GeoFence()
        
        pygame.init()
        self.screen = pygame.display.set_mode((800, 600))
        self.clock = pygame.time.Clock()

    def update(self, dt: float):
        if self.gps:
            gps_pos = self.gps.get_position()
            if gps_pos is not None:
                self.position[:2] = gps_pos
                
        if self.lidar:
            obstacles = self.lidar.get_scan()
            if self.detect_obstacle(obstacles):
                self.avoid_collision()
                
        if self.thermal:
            if self.thermal.detect_human():
                self.servo.drop_medkit()
                
        self.position[2] += -5 * dt  # Постепенное снижение
        self.health -= 0.001 * dt
        
        if not self.geofence.check(self.position):
            self.radio.send_alert(b"OUT_OF_BOUNDS")
            
        if self.health < 0.3 and self.armed:
            self.self_destruct()
            
        return self.position

    def detect_obstacle(self, obstacles):
        if not obstacles:
            return False
        for angle, distance in obstacles:
            if distance < 15.0:  
                return True
        return False

    def avoid_collision(self):
        self.position[0] += 10.0  
        self.radio.send_alert(b"COLLISION_AVOIDANCE")

    def self_destruct(self):
        try:
            encrypted = encrypt(b"SELF_DESTRUCT_ACTIVATED")  
            self.radio.send(encrypted)
            self.servo.trigger_self_destruct()  
        except Exception as e:
            print(f"Self-destruct failed: {e}")
        finally:
            self.position = np.zeros(3)
            self.health = 0.0

    def simulate(self):
        running = True
        while running and self.health > 0:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False
                    
            dt = self.clock.tick(30)/1000  
            pos = self.update(dt)
            
            self.screen.fill((0,0,0))
            if self.position[2] > 0:
                pygame.draw.circle(self.screen, (255,0,0), 
                    (int(pos[0]/5 + 400), int(pos[2]/5 + 300)), 5)
            pygame.display.flip()
            
        pygame.quit()

class GeoFence:
    def __init__(self):
        self.allowed_area = {'min_x': 0, 'max_x': 500, 'min_z': 10}
        
    def check(self, position):
        return (self.allowed_area['min_x'] <= position[0] <= self.allowed_area['max_x'] and
                position[2] >= self.allowed_area['min_z'])
