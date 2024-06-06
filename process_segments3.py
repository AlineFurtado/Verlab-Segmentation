# -*- coding: utf-8 -*-
"""
Created on Thu Jun  6 08:49:45 2024

@author: User-Aline
"""
import numpy as np
from scipy.spatial import ConvexHull

# Função para calcular o ponto central de um segmento
def calculate_central_point(segment):
    mid_index = len(segment) // 2
    if len(segment) % 2 == 0:
        return segment[mid_index - 1]
    return segment[mid_index]

# Função para calcular o lucro de um segmento
def calculate_profit(segment_points, num_people, min_profit, max_profit, last_color):
    base_profit = num_people * 10
    if last_color == 'green':
        return base_profit + max_profit
    elif last_color == 'yellow':
        return base_profit + min_profit
    return base_profit

# Função para obter a cor de um ponto
def get_color(point, cluster_data):
    for points in cluster_data['better_approach_segments'].values():
        if any(np.array_equal(point, p) for p in points):
            return 'green'
    for points in cluster_data['approach_segments'].values():
        if any(np.array_equal(point, p) for p in points):
            return 'yellow'
    for points in cluster_data['prohibited_approach'].values():
        if any(np.array_equal(point, p) for p in points):
            return 'red'
    return 'unknown'

#def order_points_anticlockwise(all_points):
#    # Converter a lista de pontos para um array NumPy
#    points = np.array(all_points)
    
#    # Encontrar a casca convexa dos pontos
#    hull = ConvexHull(points)
    
#    # Obter os pontos que formam a casca convexa
#    convex_hull_points = points[hull.vertices]
    
#    # Calcular o centroide dos pontos da casca convexa
#    centroid = np.mean(convex_hull_points, axis=0)
    
#    # Calcular os ângulos dos pontos da casca convexa em relação ao centroide
#    hull_angles = np.arctan2(convex_hull_points[:,1] - centroid[1], convex_hull_points[:,0] - centroid[0])
    
#    # Ordenar os pontos da casca convexa com base nos ângulos
#    sorted_hull_indices = np.argsort(hull_angles)
#    ordered_hull_points = convex_hull_points[sorted_hull_indices]
    
#    # Ordenar os pontos internos com base na proximidade ao ponto mais próximo da casca convexa
#    internal_points = [pt for pt in points if pt.tolist() not in ordered_hull_points.tolist()]
    
#    # Lista para armazenar todos os pontos ordenados
#    combined_points = list(ordered_hull_points)
    
#    # Inserir os pontos internos de forma a preservar a ordem
#    for pt in internal_points:
#        combined_points.append(pt)
    
#    # Converter a lista para um array NumPy
#    combined_points = np.array(combined_points)
    
#    return combined_points
def order_points_anticlockwise(points):
    points = np.array(points)  # Certificar-se de que 'points' é um array NumPy
    
    # Calcular o centroide dos pontos
    center = np.mean(points, axis=0)
    
    # Calcular os ângulos dos pontos em relação ao centroide
    angles = np.arctan2(points[:, 1] - center[1], points[:, 0] - center[0])
    
    # Ordenar os pontos pelos ângulos em sentido anti-horário
    sorted_indices = np.argsort(angles)
    ordered_points = points[sorted_indices]
    
    return ordered_points

def compare_points_lists(all_points, ordered_points):
    # Converter listas de pontos para arrays numpy para facilitar a comparação
    all_points_array = np.array(all_points)
    ordered_points_array = np.array(ordered_points)
    
    # Converter arrays para sets de tuplas para a comparação
    set_all_points = set(map(tuple, all_points_array))
    set_ordered_points = set(map(tuple, ordered_points_array))
    
    # Verificar se todos os pontos de all_points estão em ordered_points
    if set_all_points == set_ordered_points:
        return True
    else:
        missing_points = set_all_points - set_ordered_points
        for point in missing_points:
            print(f"Ponto {point} em all_points não foi encontrado na lista ordered_points")
        return False
    
def compare_points_with_tolerance(all_points, ordered_points, tolerance=1e-9):
    all_points_array = np.array(all_points)
    ordered_points_array = np.array(ordered_points)
    
    all_points_set = set(map(tuple, all_points_array))
    ordered_points_set = set(map(tuple, ordered_points_array))
    
    missing_points = set()
    
    for point in all_points_set:
        found = any(np.allclose(point, op, atol=tolerance) for op in ordered_points_set)
        if not found:
            missing_points.add(point)
    
    if not missing_points:
        return True
    else:
        for point in missing_points:
            print(f"Ponto {point} em all_points não foi encontrado na lista ordered_points")
        return False

# Função para processar os clusters
def process_clusters(data, min_profit, max_profit):
    processed_segments = {}
    id_segment = 1

    for cluster_idx, cluster_data in data.items():
        num_people = cluster_data['num_people']
        processed_segments[cluster_idx] = {'approach_segments_samples': []}
        persons = cluster_data['persons']
        virtual_persons = cluster_data['virtual_persons']

        # Extrair pontos dos segmentos
        prohibited_approach_points = [point for sublist in cluster_data['prohibited_approach'].values() for point in sublist]
        #print(f'prohibited_approach_points:{prohibited_approach_points}')
        
        approach_segments_points = [point for sublist in cluster_data['approach_segments'].values() for point in sublist]
        #print(f'approach_segments_points:{approach_segments_points}')
        
        better_approach_segments_points = [point for sublist in cluster_data['better_approach_segments'].values() for point in sublist]
        #print(f'better_approach_segments_points:{better_approach_segments_points}')

        # Concatenar todos os pontos em uma lista para formar uma curva fechada
        all_points = prohibited_approach_points + approach_segments_points + better_approach_segments_points
        #print(f'pontos em prohibited_approach_points:{len(prohibited_approach_points)}')
        #print(f'pontos em approach_segments_points :{len(approach_segments_points)}')
        #print(f'pontos em better_approach_segments_points:{len(better_approach_segments_points)}')
        print(f'total de pontos:{len(all_points)}')

        # Ordenar os pontos em sentido anti-horário
        ordered_points = order_points_anticlockwise(all_points)
        print(f'n.ordered_points:{len(ordered_points)}')
        current_segment = []
        current_color = 'unknown'
        last_color = 'unknown'

###################################################################################
        #teste necessário
        # Comparar as listas de pontos com uma tolerância
#        result = compare_points_with_tolerance(all_points, ordered_points)
#        if result:
#            print("Todos os pontos em all_points estão presentes em ordered_points.")
#        else:
#            print("Alguns pontos em all_points não foram encontrados em ordered_points.")
#######################################################################################
        # Laço para percorrer cada ponto da curva fechada
        for point in ordered_points:
            color = get_color(point, cluster_data)
            #print(f'color:{color}')
            #print(f'last_color:{last_color}')
            if color == last_color:
                if color != 'red':
                    current_segment.append(point)
                else:
                    continue
            else:
                if current_segment:
                    median_point = calculate_central_point(current_segment)
                    profit = calculate_profit(current_segment, num_people, min_profit, max_profit, last_color)
                    processed_segments[cluster_idx]['approach_segments_samples'].append({
                        'id_segment': id_segment,
                        'median_point': median_point,
                        'profit': profit,
                        'color': last_color,
                        'segment_size': len(current_segment)
                    })
                    id_segment += 1
                if color == 'red':
                    current_segment = []
                else:
                    current_segment = [point]
            last_color = color

        if current_segment:
            median_point = calculate_central_point(current_segment)
            profit = calculate_profit(current_segment, num_people, min_profit, max_profit, last_color)
            processed_segments[cluster_idx]['approach_segments_samples'].append({
                'id_segment': id_segment,
                'median_point': median_point,
                'profit': profit,
                'color': last_color,
                'segment_size': len(current_segment)
            })

    return processed_segments


