# -*- coding: utf-8 -*-
"""
Created on Wed Jul 10 14:33:37 2024

@author: User-Aline
"""

import numpy as np
from shapely.geometry import Polygon, Point
from person import Person
from robot4_2 import *

# Definição de uma função geradora que irá iterar enquanto a condição de parada não for satisfeita
# Define um número maior de frames para garantir que o robô percorra todo o caminho
def frame_generator(robot, people, goals, obstacles):
    while True:
        yield None
        if robot.current_segment_index >= len(robot.path):
            break
def move_robot(robot, people, obstacles, move_obstacles, rep_range_people=2.0, krep_people=1.5, max_step=0.8):
    is_last_point = (robot.current_path_index == len(robot.path[robot.current_segment_index]) - 1)
    
    # Calcular forças de repulsão
    repulsion_people = rep_force_robot(np.array([robot.x, robot.y]), people, rep_range_people, krep_people)
    repulsion_move_obstacles = robot.avoid_move_obstacles(move_obstacles)
    repulsion_obstacles = robot.avoid_obstacles(obstacles)

    if is_last_point:
        # Identificar se o ponto de destino está dentro de um obstáculo fixo
        next_goal = robot.path[robot.current_segment_index][robot.current_path_index]
        destination_obstacle = None
        for obstacle in obstacles:
            if Polygon(obstacle).contains(Point(next_goal)):
                destination_obstacle = obstacle
                break
        
        # Ignorar repulsão do obstáculo de destino se estiver no último ponto
        if destination_obstacle:
            combined_forces = repulsion_people + repulsion_move_obstacles
        else:
            combined_forces = repulsion_people + repulsion_obstacles + repulsion_move_obstacles
    else:
        combined_forces = repulsion_people + repulsion_obstacles + repulsion_move_obstacles

    print(f'combined_forces:{combined_forces}')
    robot.move(combined_forces, obstacles)


# def move_robot(robot, people, obstacles, move_obstacles, WORLD_SIZE, katt=1.0, krep=1.5, rep_range=2.0, rep_range_people=2.0, krep_people=1.5, max_step=0.8):
#     is_last_point = (robot.current_path_index == len(robot.path[robot.current_segment_index]) - 1)
    
#     #repulsion_people = robot.avoid_people(people)
#     repulsion_people = rep_force_robot(np.array([robot.x, robot.y]), people, WORLD_SIZE, rep_range, krep, rep_range_people,krep_people)
#     repulsion_move_obstacles = robot.avoid_move_obstacles(move_obstacles)
#     if is_last_point:
#         combined_forces = repulsion_people + repulsion_move_obstacles
#         #combined_forces = repulsion_move_obstacles
#     else:
#         repulsion_obstacles = robot.avoid_obstacles(obstacles)
#         combined_forces = repulsion_people + repulsion_obstacles + repulsion_move_obstacles
#         #combined_forces = repulsion_obstacles + repulsion_move_obstacles
#         print(f'combined_forces:{combined_forces}')
#     robot.move(combined_forces, obstacles)

def move_towards_goals(people, goals, obstacles, max_people_per_goal=6, katt=0.1, krep=2.0, rep_range=4.0, max_step=0.8):
    people_at_goals = {tuple(goal): [] for goal in goals}
    for person in people:
        if hasattr(person, 'goal') and any(np.array_equal(person.goal, g) for g in goals):
            closest_goal = person.goal
        else:
            position = np.array([person.x, person.y])
            closest_goal = min(goals, key=lambda g: np.linalg.norm(position - g))
            person.goal = closest_goal
        people_at_goals[tuple(closest_goal)].append(person)

    for person in people:
        position = np.array([person.x, person.y])
        if len(people_at_goals[tuple(person.goal)]) > max_people_per_goal:
            alternative_goals = [g for g in goals if not np.array_equal(g, person.goal) and len(people_at_goals[tuple(g)]) < max_people_per_goal]
            if alternative_goals:
                person.goal = min(alternative_goals, key=lambda g: np.linalg.norm(position - g))
            else:
                continue

        attraction = att_force(position, person.goal, katt)
        repulsion_people = rep_force(position, [p.get_coords()[:2] for p in people if p != person], rep_range, krep)
        repulsion_obstacles = rep_force_obstacles(position, obstacles, rep_range, krep)

        total_force = attraction + repulsion_people + repulsion_obstacles
        if np.linalg.norm(total_force) > max_step:
            total_force = max_step * total_force / np.linalg.norm(total_force)
        person.x += total_force[0]
        person.y += total_force[1]
        if np.linalg.norm(attraction) > 0:
            person.th = np.arctan2(attraction[1], attraction[0])

def change_person_goal(people, goals, obstacles, rep_range=2.0, min_distance=2.0):
    for person in people:
        current_goal = np.array(person.goal)
        position = np.array([person.x, person.y])
        if np.linalg.norm(position - current_goal) < min_distance:
            # Encontrar objetivos alternativos válidos
            valid_alternatives = [g for g in goals if np.linalg.norm(position - g) > rep_range]
            if not valid_alternatives:
                continue
            valid_alternatives = np.array(valid_alternatives)  # Garantir que é um array 1-dimensional
            new_goal = valid_alternatives[np.random.choice(len(valid_alternatives))]
            person.goal = new_goal


def att_force(q, goal, katt=0.1):
    return katt * (goal - q)

def rep_force(q, others, rep_range, krep):
    repulsion = np.array([0.0, 0.0])
    for other in others:
        distance = np.linalg.norm(q - other)
        if 0 < distance < rep_range:
            repulsion += krep * (1/distance - 1/rep_range) * (1/distance**2) * (q - other) / distance
    return repulsion

def rep_force_robot(q, people, rep_range_people=2.0, krep_people=1.5):
    repulsion = np.array([0.0, 0.0])

    # Repulsão pelas outras pessoas
    for person in people:
        person_position = np.array([person.x, person.y])
        distance = np.linalg.norm(q - person_position)
        print(f'distance_person:{distance}')
        if 0 < distance < rep_range_people:
            repulsion += krep_people * (1 / distance - 1 / rep_range_people) * (1 / distance ** 2) * (q - person_position) / distance
    
    return repulsion

def rep_force_obstacles(q, obstacles, rep_range, krep):
    repulsion = np.array([0.0, 0.0])
    point = Point(q)
    for obstacle in obstacles:
        polygon = Polygon(obstacle)
        if polygon.contains(point):
            continue  # Ignorar se o ponto já está dentro do obstáculo
        distance = point.distance(polygon)
        if 0 < distance < rep_range:
            nearest_point = np.array(polygon.exterior.interpolate(polygon.exterior.project(point)).coords[0])
            repulsion += krep * (1/distance - 1/rep_range) * (1/distance**2) * (q - nearest_point) / distance
    return repulsion

# def generate_people(n, world_size, obstacles, safe_distance=2):
#     people = []
#     for i in range(n):
#         while True:
#             x = np.random.uniform(0, world_size-1)
#             y = np.random.uniform(0, world_size-1)
#             th = np.random.uniform(-np.pi, np.pi)
#             if x == 0 and y == 0:
#                 continue
#             new_person = Person(x=x, y=y, th=th, id_node=i)
#             new_point = Point([x, y])
#             if not any(Polygon(obstacle).buffer(safe_distance).contains(new_point) for obstacle in obstacles):
#                 people.append(new_person)
#                 break
#     return people
# def generate_people(n, world_size, obstacles, safe_distance=5):
#     people = []
#     for i in range(n):
#         while True:
#             x = np.random.uniform(-2, world_size-1)
#             y = np.random.uniform(-2, world_size-1)
#             th = np.random.uniform(-np.pi, np.pi)
#             new_person = Person(x=x, y=y, th=th, id_node=i)
#             new_point = Point([x, y])
#             if not any(Polygon(obstacle).buffer(safe_distance).contains(new_point) for obstacle in obstacles):
#                 people.append(new_person)
#                 break
#     return people
def generate_people(n, world_size, obstacles, safe_distance=5):
    people = []
    for i in range(n):
        while True:
            x = np.random.uniform(-2, world_size-1)
            y = np.random.uniform(-2, world_size-1)
            th = np.random.uniform(-np.pi, np.pi)
            new_person = Person(x=x, y=y, th=th, id_node=i)
            new_point = Point([x, y])
            distance_from_origin = np.sqrt(x**2 + y**2)
            if distance_from_origin >= 5.0 and not any(Polygon(obstacle).buffer(safe_distance).contains(new_point) for obstacle in obstacles):
                people.append(new_person)
                break
    return people

def generate_goals(n, world_size, obstacles, people, min_distance, safe_distance=5):
    goals = []
    people_positions = [np.array([person.x, person.y]) for person in people]
    while len(goals) < n:
        new_goal = np.random.uniform(-1, world_size, 2)
        new_point = Point(new_goal)
        if all(np.linalg.norm(new_goal - existing_goal) >= min_distance for existing_goal in goals) and \
           all(np.linalg.norm(new_goal - person_pos) >= min_distance for person_pos in people_positions) and \
           not any(Polygon(obstacle).buffer(safe_distance).contains(new_point) for obstacle in obstacles):
            goals.append(new_goal)
    return goals

def extract_move_obstacles(segments_samples):
    obstacles = []
    buffer_distance = 1.0  # Distância do buffer

    for cluster_idx, data in segments_samples.items():
        points = []

        for person, segments in data['prohibited_approach'].items():
            points.extend(segments)
        for person, segments in data['approach_segments'].items():
            points.extend(segments)
        for person, segments in data['better_approach_segments'].items():
            points.extend(segments)

        if points:
            # Cria um polígono a partir dos pontos
            polygon = Polygon(points)
            # Corrige o polígono para garantir que ele seja convexo, se necessário
            if not polygon.is_valid or polygon.is_empty:
                polygon = polygon.convex_hull

            # Adiciona um buffer ao polígono
            buffered_polygon = polygon.buffer(buffer_distance)
            obstacles.append(buffered_polygon)
    print(f'move_obstacles:{obstacles}')
    return obstacles

