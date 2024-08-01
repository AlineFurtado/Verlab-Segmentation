# -*- coding: utf-8 -*-
"""
Created on Thu Jul 18 14:45:51 2024

@author: User-Aline
"""

import numpy as np
import time
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation, PillowWriter
from shapely.geometry import Polygon, Point

class Robot:
    def __init__(self, x=0, y=0, th=0, goal=None, path=None):
        self.x = x
        self.y = y
        self.th = th
        self.goal = goal
        self.path = path if path is not None else []
        self.move_obstacles = []
        self.current_segment_index = 0
        self.current_path_index = 0
        self._radius = 0.5
        self.attempt_counter = 0
        self.max_attempts = 20  # Número máximo de tentativas
        self.interaction_start_time = None  # Variável para armazenar o tempo de início da interação
        self.smoothing_window = 5  # Número de posições anteriores para suavizar
        self.previous_positions = []  # Inicializa a lista para armazenar posições anteriores
        self.previous_directions = []
    # def move(self, combined_force, max_step=1.1): #VERSAO QUE GEROU O 28F ANIME
    #     if self.current_segment_index >= len(self.path):
    #         return

    #     segment = self.path[self.current_segment_index]
    #     print(f'segment:{segment}')
    #     if self.current_path_index >= len(segment):
    #         self.current_path_index = 0
    #         self.current_segment_index += 1
    #         if self.current_segment_index >= len(self.path):
    #             return
    #         segment = self.path[self.current_segment_index]

    #     next_goal = segment[self.current_path_index]
    #     print(f'next_goal:{next_goal}')
    #     attraction_force = self.calculate_attraction_force(next_goal)
        

    #     attempts = 0
    #     while True:
    #         total_force = attraction_force + combined_force
    #         print(f'total_force:{total_force}')
    #         print(f'posicao do robo:{[self.x, self.y]}')
    #         if np.linalg.norm(total_force) > max_step:
    #             total_force = max_step * total_force / np.linalg.norm(total_force)

    #         new_x = self.x + total_force[0]
    #         new_y = self.y + total_force[1]

    #         new_position = Point(new_x, new_y)
    #         collision = any(Polygon(obstacle).contains(new_position) for obstacle in self.move_obstacles)

    #         if not collision:
    #             self.x = new_x
    #             self.y = new_y
    #             break
    #         else:
    #             print("possível colisão com mov_obstacles")
    #             repulsion_force = self.avoid_move_obstacles(self.move_obstacles)
    #             combined_force += repulsion_force
    #             attraction_force = self.calculate_attraction_force(next_goal)
    #             max_step += 0.2
    #             print(f'max_step:{max_step}')

    #             attempts += 1
    #             if attempts >= self.max_attempts:
    #                 self.current_path_index += 1
    #                 if self.current_path_index >= len(segment):
    #                     self.current_path_index = 0
    #                     self.current_segment_index += 1
    #                     if self.current_segment_index >= len(self.path):
    #                         return
    #                 break

    #     if np.linalg.norm(attraction_force) > 0:
    #         self.th = np.arctan2(attraction_force[1], attraction_force[0])

    #     if np.linalg.norm(np.array([self.x, self.y]) - np.array(next_goal)) < 1.0:
    #         if self.current_path_index == len(segment) - 1:
    #             if self.interaction_start_time is None:
    #                 self.interaction_start_time = time.time()
    #                 print(f'tempo de interação:{self.interaction_start_time}')
    #             elif time.time() - self.interaction_start_time >= 120:  # 60 seconds for interaction
    #                 self.interaction_start_time = None
    #                 self.current_path_index = 0
    #                 self.current_segment_index += 1
    #                 if self.current_segment_index >= len(self.path):
    #                     return
    #         else:
    #             self.current_path_index += 1
    #             self.interaction_start_time = None  # Reset interaction time when moving to the next point
    #     else:
    #         self.attempt_counter += 1
    #         if self.attempt_counter >= self.max_attempts:
    #             self.current_path_index += 1
    #             self.attempt_counter = 0
    #             if self.current_path_index >= len(segment):
    #                 self.current_path_index = 0
    #                 self.current_segment_index += 1
    #                 if self.current_segment_index >= len(self.path):
    #                     return
    def move(self, combined_force, obstacles, max_step=1.0):
        if self.current_segment_index >= len(self.path):
            return
    
        segment = self.path[self.current_segment_index]
        if self.current_path_index >= len(segment):
            self.current_path_index = 0
            self.current_segment_index += 1
            if self.current_segment_index >= len(self.path):
                return
            segment = self.path[self.current_segment_index]
    
        next_goal = segment[self.current_path_index]
        print(f'next_goal:{next_goal}')
        attraction_force = self.calculate_attraction_force(next_goal)
    
        # Identificar se o ponto de destino está dentro de um obstáculo fixo
        destination_obstacle = None
        for obstacle in obstacles:
            if Polygon(obstacle).contains(Point(next_goal)):
                destination_obstacle = obstacle
                break
    
        # attempts = 0
        # while True:
        #     total_force = attraction_force + combined_force
        #     if np.linalg.norm(total_force) > max_step:
        #         total_force = max_step * total_force / np.linalg.norm(total_force)
    
        #     new_x = self.x + total_force[0]
        #     new_y = self.y + total_force[1]
    
        #     new_position = Point(new_x, new_y)
        #     print(f'robot position:{new_position}')
        #     # Verifica colisão com obstáculos móveis e obstáculos fixos exceto o de destino
        #     collision = any(Polygon(mov_obstacle).contains(new_position) for mov_obstacle in self.move_obstacles)
    
        #     if not collision:
        #         self.previous_positions.append((new_x, new_y))
        #         if len(self.previous_positions) > self.smoothing_window:
        #             self.previous_positions.pop(0)
        #         smoothed_x = np.mean([pos[0] for pos in self.previous_positions])
        #         smoothed_y = np.mean([pos[1] for pos in self.previous_positions])
        #         new_smoothposition = Point(smoothed_x, smoothed_y)
        #         sm_collision = any(Polygon(mov_obstacle).contains(new_smoothposition) for mov_obstacle in self.move_obstacles)
        #         if not sm_collision:
        #             self.x = smoothed_x
        #             self.y = smoothed_y
        #             break
        #         else:
        #             print("Possível colisão com obstáculos móveis")
        #             repulsion_force = self.avoid_move_obstacles(self.move_obstacles) + self.avoid_obstacles(obstacles)
        #             combined_force += repulsion_force
        #             attraction_force = self.calculate_attraction_force(next_goal)
        #             max_step += 0.05
        #     else:
        #         print("Possível colisão com obstáculos móveis")
        #         repulsion_force = self.avoid_move_obstacles(self.move_obstacles) + self.avoid_obstacles(obstacles)
        #         combined_force += repulsion_force
        #         attraction_force = self.calculate_attraction_force(next_goal)
        #         max_step += 0.05
    
        #         attempts += 1
        #         print(f'attempts:{attempts}')
        #         if attempts >= self.max_attempts:
        #             # Introduzir uma pequena mudança aleatória na direção para evitar ficar preso
        #             angle = np.random.uniform(-np.pi/4, np.pi/4)
        #             rotation_matrix = np.array([
        #                 [np.cos(angle), -np.sin(angle)],
        #                 [np.sin(angle), np.cos(angle)]
        #             ])
        #             combined_force = np.dot(rotation_matrix, combined_force)
        #             self.current_path_index += 1
        #             if self.current_path_index >= len(segment):
        #                 self.current_path_index = 0
        #                 self.current_segment_index += 1
        #                 if self.current_segment_index >= len(self.path):
        #                     return
        #             break
        attempts = 0
        stuck_counter = 0
        last_position = np.array([self.x, self.y])
        
        while True:
            total_force = attraction_force + combined_force
            if np.linalg.norm(total_force) > max_step:
                total_force = max_step * total_force / np.linalg.norm(total_force)
            
            new_x = self.x + total_force[0]
            new_y = self.y + total_force[1]
            
            new_position = Point(new_x, new_y)
            
            collision = any(Polygon(mov_obstacle).contains(new_position) for mov_obstacle in self.move_obstacles)
            
            if not collision:
                self.previous_positions.append((new_x, new_y))
                if len(self.previous_positions) > self.smoothing_window:
                    self.previous_positions.pop(0)
                smoothed_x = np.mean([pos[0] for pos in self.previous_positions])
                smoothed_y = np.mean([pos[1] for pos in self.previous_positions])
                new_smoothposition = Point(smoothed_x, smoothed_y)
                sm_collision = any(Polygon(mov_obstacle).contains(new_smoothposition) for mov_obstacle in self.move_obstacles)
                if not sm_collision:
                    self.x = smoothed_x
                    self.y = smoothed_y
                    break
            else:
                repulsion_force = self.avoid_move_obstacles(self.move_obstacles) + self.avoid_obstacles(obstacles)
                combined_force += repulsion_force
                attempts += 1
        
                # Verifica se o robô está preso em um loop
                current_position = np.array([self.x, self.y])
                if np.linalg.norm(current_position - last_position) < 0.1:
                    stuck_counter += 1
                else:
                    stuck_counter = 0
                
                if stuck_counter > 5:
                    # Introduz um deslocamento aleatório para tentar sair do ciclo
                    random_displacement = np.random.uniform(-0.5, 0.5, size=2)
                    combined_force += random_displacement
                    stuck_counter = 0  # Reinicia o contador de "preso"
        
                last_position = current_position
        
                if attempts >= self.max_attempts:
                    # Introduzir uma pequena mudança aleatória na direção para evitar ficar preso
                    angle = np.random.uniform(-np.pi/4, np.pi/4)
                    rotation_matrix = np.array([
                        [np.cos(angle), -np.sin(angle)],
                        [np.sin(angle), np.cos(angle)]
                    ])
                    combined_force = np.dot(rotation_matrix, combined_force)
                    attempts = 0  # Resetar o contador de tentativas
                    # O robô tentará continuar se movendo a partir da posição atual

    ####
        if np.linalg.norm(attraction_force) > 0:
            self.th = np.arctan2(attraction_force[1], attraction_force[0])
    
        # Verificar se atingiu o ponto de destino
        if np.linalg.norm(np.array([self.x, self.y]) - np.array(next_goal)) < 1.0:
            if self.current_path_index == len(segment) - 1:
                if self.interaction_start_time is None:
                    self.interaction_start_time = time.time()
                elif time.time() - self.interaction_start_time >= 120:
                    self.interaction_start_time = None
                    self.current_path_index = 0
                    self.current_segment_index += 1
                    if self.current_segment_index >= len(self.path):
                        return
            else:
                self.current_path_index += 1
                self.interaction_start_time = None
        else:
            self.attempt_counter += 1
            if self.attempt_counter >= self.max_attempts:
                self.current_path_index += 1
                self.attempt_counter = 0
                if self.current_path_index >= len(segment):
                    self.current_path_index = 0
                    self.current_segment_index += 1
                    if self.current_segment_index >= len(self.path):
                        return

    def calculate_attraction_force(self, goal, katt=1.0):
        return katt * (np.array(goal) - np.array([self.x, self.y]))

    def draw(self, ax):
        # Desenha o corpo do robô
        robot_body = plt.Circle((self.x, self.y), radius=self._radius, fill=True, color='blue', label='Robô')
        ax.add_patch(robot_body)
        
        # Marca a posição base com um "X"
        ax.plot(0, 0, 'rx', label='Posição base')
        
        # Adiciona a legenda
        ax.legend()

    def avoid_people(self, people, rep_range=2.0, krep=3.0):
        repulsion = np.array([0.0, 0.0])
        for person in people:
            distance = np.linalg.norm(np.array([self.x, self.y]) - np.array([person.x, person.y]))
            print(f'distance_people:{distance}')
            if 0 < distance < rep_range:
                repulsion += krep * (1/distance - 1/rep_range) * (1/distance**2) * (np.array([self.x, self.y]) - np.array([person.x, person.y])) / distance
        print(f'repulsion_people:{repulsion}')
        return repulsion

    def avoid_obstacles(self, obstacles, rep_range=2.0, krep=1.5):
        repulsion = np.array([0.0, 0.0])
        point = Point([self.x, self.y])
        for obstacle in obstacles:
            polygon = Polygon(obstacle)
            if polygon.contains(point):
                continue
            distance = point.distance(polygon)
            print(f'distance_obstacle:{distance}')
            if 0 < distance < rep_range:
                nearest_point = np.array(polygon.exterior.interpolate(polygon.exterior.project(point)).coords[0])
                repulsion += krep * (1/distance - 1/rep_range) * (1/distance**2) * (np.array([self.x, self.y]) - nearest_point) / distance
        print(f'repulsion_obstacle:{repulsion}')
        return repulsion

    def avoid_move_obstacles(self, move_obstacles, rep_range=4.0, krep=2.0):
        # Inicializa a força de repulsão como um vetor nulo.
        repulsion = np.array([0.0, 0.0])
        # Cria um ponto com a posição atual do robô.
        point = Point([self.x, self.y])
        
        # Itera sobre cada obstáculo móvel.
        for obstacle in move_obstacles:
            polygon = Polygon(obstacle)
            
            # Calcula a distância do ponto até o polígono.
            distance = point.distance(polygon)
            print(f'distance_move_obstacle:{distance}')
            
            # Se o ponto estiver dentro do polígono, aplica uma força de repulsão forte.
            if polygon.contains(point):
                # Força de repulsão quando o robô está dentro do obstáculo
                nearest_point = np.array(polygon.exterior.interpolate(polygon.exterior.project(point)).coords[0])
                repulsion += krep * (np.array([self.x, self.y]) - nearest_point) / max(distance, 0.1)
            elif 0 < distance < rep_range:
                # Se a distância estiver dentro do alcance de repulsão, calcula a força de repulsão.
                nearest_point = np.array(polygon.exterior.interpolate(polygon.exterior.project(point)).coords[0])
                dynamic_krep = krep * (rep_range - distance) / rep_range  # Ajusta krep dinamicamente
                repulsion += dynamic_krep * (1/distance - 1/rep_range) * (1/distance**2) * (np.array([self.x, self.y]) - nearest_point) / distance
        
        print(f'repulsion_mov_obstacle:{repulsion}')
        # Retorna a força de repulsão total.
        return repulsion


    


