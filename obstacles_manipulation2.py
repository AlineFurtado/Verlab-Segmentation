# -*- coding: utf-8 -*-
"""
Created on Thu Jun  6 17:35:11 2024

@author: User-Aline
"""

import matplotlib.pyplot as plt
import numpy as np
from shapely.geometry import Polygon, Point

def extract_polygons(segments_samples):
    obstacles = []
    buffer_distance = 0.5  # Distância do buffer

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

    return obstacles


# def replan_endpoint(point, obstacles, min_distance=0.5, buffer_size=0.2):
#     original_point = Point(point)
#     for poly in obstacles:
#         #for poly in handle_multipolygon(obstacle):
#         if poly.contains(original_point) or poly.buffer(buffer_size).contains(original_point):
#                 distance = min_distance
#                 angle = 0
#                 while angle < 2 * np.pi:
#                     new_x = point[0] + distance * np.cos(angle)
#                     new_y = point[1] + distance * np.sin(angle)
#                     new_point = Point(new_x, new_y)
#                     if not poly.contains(new_point) and not poly.buffer(buffer_size).contains(new_point):
#                         return (new_x, new_y)
#                     angle += np.pi / 18
#                 distance += 0.1
#                 if distance > buffer_size:
#                     break
#     return point
def is_parallel(point, segment):
    """Check if the point is parallel to the segment."""
    # Calculate vectors
    px, py = point
    (x1, y1), (x2, y2) = segment
    segment_vector = (x2 - x1, y2 - y1)
    point_vector = (px - x1, py - y1)
    cross_product = np.cross(segment_vector, point_vector)
    return np.isclose(cross_product, 0)

def replan_endpoint(point, obstacles, data, min_distance=1.0, buffer_size=0.2):
    original_point = Point(point)
    
    for poly in obstacles:
        if poly.contains(original_point) or poly.buffer(buffer_size).contains(original_point):
            distance = min_distance
            angle = 0
            while angle < 2 * np.pi:
                new_x = point[0] + distance * np.cos(angle)
                new_y = point[1] + distance * np.sin(angle)
                new_point = Point(new_x, new_y)
                if not poly.contains(new_point) and not poly.buffer(buffer_size).contains(new_point):
                    # Verificar paralelismo com segmentos em 'better_approach_segments'
                    #is_parallel_flag = False
                    #for cluster_idx, dados in data.items():
                    #    points = []
                    #    for person, segments in dados['prohibited_approach'].items():
                    #        points.extend(segments)
                    #        for segment in points:
                    #            if is_parallel((new_x, new_y), segment):
                    #                is_parallel_flag = True
                    #                break
                    #        if is_parallel_flag:
                    #            break
                    #if not is_parallel_flag:
                    return (new_x, new_y)
                angle += np.pi / 18
            distance += 0.05
            if distance > buffer_size:
                break
    
    return point
#recebe uma tupla, um par de pontos route=((x_start,y_start),(x_end,y_end))
def process_route(route, obstacles,  data):
    processed_route = []
    for start_point, end_point in route:
        if not np.array_equal(start_point, (0, 0)):
            new_start_point = replan_endpoint(start_point, obstacles, data)
        else:
            new_start_point = start_point
        
        if not np.array_equal(end_point, (0, 0)) and not np.array_equal(end_point, start_point):
            new_end_point = replan_endpoint(end_point, obstacles, data)
        else:
            new_end_point = end_point
        
        processed_route.append((new_start_point, new_end_point))
    return processed_route

def plot_polygons(polygons):
    fig, ax = plt.subplots()
    for polygon in polygons:
        x, y = polygon.exterior.xy
        ax.plot(x, y, label='Polygon')
        ax.fill(x, y, alpha=0.5)

    ax.set_title('Polygons')
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.legend()
    plt.show()
