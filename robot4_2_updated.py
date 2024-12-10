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
    def __init__(self, x=0, y=0, th=0, goal=None, path=None, budget=30.0):
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
        self.max_attempts = 20
        self.interaction_start_time = None
        self.smoothing_window = 5
        self.previous_positions = []
        self.budget = budget  # Total allowed distance
        self.remaining_budget = budget  # Remaining distance
        # Métricas de navegação
        self.path_length = 0.0
        self.collision_detected = False
        self.total_time = 0.0
        self.interactions = []
        self.coordinates_visited = []  # Lista para armazenar as coordenadas visitadas

    def move(self, combined_force, obstacles, max_step=1.0):
        def return_to_base():
            base_position = np.array([0, 0])
            while True:
                attraction_force = self.calculate_attraction_force(base_position)
                combined_force = attraction_force + self.avoid_obstacles(obstacles) + self.avoid_move_obstacles(self.move_obstacles)
                if np.linalg.norm(combined_force) > max_step:
                    combined_force = max_step * combined_force / np.linalg.norm(combined_force)

                self.x += combined_force[0]
                self.y += combined_force[1]
                step_distance = np.linalg.norm(combined_force)
                self.path_length += step_distance
                self.remaining_budget -= step_distance

                if np.linalg.norm(np.array([self.x, self.y]) - base_position) < 0.1 or self.remaining_budget < 0:
                    break
        
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
        attraction_force = self.calculate_attraction_force(next_goal)

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

            # Verifica colisão
            collision = any(Polygon(mov_obstacle).contains(new_position) for mov_obstacle in self.move_obstacles)
            if collision:
                self.collision_detected = True

            if not collision:
                # Atualiza o comprimento do caminho
                self.path_length += np.linalg.norm([new_x - self.x, new_y - self.y])
                
                # Atualiza posição
                self.previous_positions.append((new_x, new_y))
                if len(self.previous_positions) > self.smoothing_window:
                    self.previous_positions.pop(0)
                smoothed_x = np.mean([pos[0] for pos in self.previous_positions])
                smoothed_y = np.mean([pos[1] for pos in self.previous_positions])
                self.x = smoothed_x
                self.y = smoothed_y

                # Adiciona as coordenadas visitadas
                self.coordinates_visited.append((self.x, self.y))
                break
            else:
                # Aplica força de repulsão para evitar obstáculos
                repulsion_force = self.avoid_move_obstacles(self.move_obstacles) + self.avoid_obstacles(obstacles)
                combined_force += repulsion_force
                attempts += 1

                # Verifica se o robô está preso
                current_position = np.array([self.x, self.y])
                if np.linalg.norm(current_position - last_position) < 0.1:
                    stuck_counter += 1
                else:
                    stuck_counter = 0

                if stuck_counter > 5:
                    random_displacement = np.random.uniform(-0.5, 0.5, size=2)
                    combined_force += random_displacement
                    stuck_counter = 0

                last_position = current_position

                if attempts >= self.max_attempts:
                    angle = np.random.uniform(-np.pi/4, np.pi/4)
                    rotation_matrix = np.array([
                        [np.cos(angle), -np.sin(angle)],
                        [np.sin(angle), np.cos(angle)]
                    ])
                    combined_force = np.dot(rotation_matrix, combined_force)
                    attempts = 0

        # Verificar se atingiu o ponto de destino
        if np.linalg.norm(np.array([self.x, self.y]) - np.array(next_goal)) < 1.0:
            # Verifica se o último objetivo é o ponto de origem
            if next_goal == (0, 0) and self.current_segment_index == len(self.path) - 1:
                # Último ponto é a origem, fim da navegação, não é uma interação
                self.current_path_index = 0
                self.current_segment_index += 1
                if self.current_segment_index >= len(self.path):
                    return
            else:
                # Interação se não for o ponto de origem
                if self.current_path_index == len(segment) - 1:
                    if self.interaction_start_time is None:
                        self.interaction_start_time = time.time()
                    elif time.time() - self.interaction_start_time >= 120:
                        interaction_duration = time.time() - self.interaction_start_time
                        interaction_distance = np.linalg.norm(np.array([self.x, self.y]) - np.array(next_goal))
                        self.interactions.append((interaction_duration, interaction_distance, True))  # Interação bem-sucedida
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

    # def draw(self, ax):
    #     # Desenha o corpo do robô
    #     robot_body = plt.Circle((self.x, self.y), radius=self._radius, fill=True, color='blue', label='Robot')
    #     ax.add_patch(robot_body)
        
    #     # Marca a posição base com um "X"
    #     ax.plot(0, 0, 'rx', label='Base Position')
        
    #     # Adiciona a legenda
    #     ax.legend()

    def draw(self, ax):
        # Desenha o corpo do robô
        robot_body = plt.Circle((self.x, self.y), radius=self._radius, fill=True, color='blue', label='Robot')
        ax.add_patch(robot_body)
    
        # Marca a posição base com um "X"
        ax.plot(0, 0, 'rx', label='Base Position')

        # Desenha a trilha do robô com uma linha para mostrar seu movimento
        #if len(self.previous_positions) > 1:
        #    x_vals, y_vals = zip(*self.previous_positions)
        #    ax.plot(x_vals, y_vals, 'g--', label='Robot Path')  # Linha tracejada verde para os movimentos anteriores

        # Marca os pontos onde o robô parou para interação
        if self.interactions:
            for interaction in self.interactions:
                ax.plot(interaction[0], interaction[1], 'bo', markersize=10)#, label='Pause for Interaction')  # Marca azul para pausas

        # Adiciona a legenda
        ax.legend()


    def avoid_people(self, people, rep_range=2.0, krep=3.0):
        repulsion = np.array([0.0, 0.0])
        for person in people:
            distance = np.linalg.norm(np.array([self.x, self.y]) - np.array([person.x, person.y]))
            #print(f'distance_people:{distance}')
            if 0 < distance < rep_range:
                repulsion += krep * (1/distance - 1/rep_range) * (1/distance**2) * (np.array([self.x, self.y]) - np.array([person.x, person.y])) / distance
        #print(f'repulsion_people:{repulsion}')
        return repulsion

    def avoid_obstacles(self, obstacles, rep_range=2.0, krep=1.5):
        repulsion = np.array([0.0, 0.0])
        point = Point([self.x, self.y])
        for obstacle in obstacles:
            polygon = Polygon(obstacle)
            if polygon.contains(point):
                continue
            distance = point.distance(polygon)
            #print(f'distance_obstacle:{distance}')
            if 0 < distance < rep_range:
                nearest_point = np.array(polygon.exterior.interpolate(polygon.exterior.project(point)).coords[0])
                repulsion += krep * (1/distance - 1/rep_range) * (1/distance**2) * (np.array([self.x, self.y]) - nearest_point) / distance
        #print(f'repulsion_obstacle:{repulsion}')
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
            #print(f'distance_move_obstacle:{distance}')
            
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
        
        #print(f'repulsion_mov_obstacle:{repulsion}')
        # Retorna a força de repulsão total.
        return repulsion


    def print_summary(self):
           print("=== Navegação Finalizada ===")
           print(f"Tempo total: {self.total_time:.2f} segundos")
           print(f"Comprimento total do caminho: {self.path_length:.2f} unidades")
           print(f"Orçamento total: {self.budget:.2f} unidades")
           print(f"Orçamento restante: {self.remaining_budget:.2f} unidades")
           print(f"Colisão detectada: {'Sim' if self.collision_detected else 'Não'}")
    
           for idx, interaction in enumerate(self.interactions):
               duration, distance, success = interaction
               print(f"Interação {idx + 1}:")
               print(f"  - Duração: {duration:.2f} segundos")
               print(f"  - Distância ao alvo: {distance:.2f} unidades")
               print(f"  - Bem-sucedida: {'Sim' if success else 'Não'}")
        
        # Imprimir as coordenadas visitadas
               print("\n=== Coordenadas Visitadas ===")
               for idx, coord in enumerate(self.coordinates_visited):
                   print(f"Passo {idx + 1}: {coord}")



