# -*- coding: utf-8 -*-
"""
Created on Mon May 20 09:01:12 2024

@author: User-Aline
"""
from person import Person
from overall_density8_1 import OverallDensity
import numpy as np
import matplotlib.pyplot as plt
from shapely.geometry import Point, Polygon, MultiPolygon, LineString
from shapely.ops import unary_union
from obstacles_manipulation2 import process_route, replan_endpoint


def calculate_comprimento_path(path):
    """
    Calcula o comprimento total de um caminho dado uma lista de pontos.
    :param path: Lista de tuplas (x, y) representando o caminho.
    :return: Comprimento total do caminho.
    """
    comprimento_total = 0.0
    for i in range(len(path) - 1):
        ponto_inicial = np.array(path[i])
        ponto_final = np.array(path[i + 1])
        comprimento_total += np.linalg.norm(ponto_final - ponto_inicial)
    return comprimento_total

def calculate_repulsion_force(current_point, obstacles, repulsion_distance, repulsion_strength):
    repulsion_force = np.zeros(2)
    current_point = np.array(current_point)
    #print("Calculating repulsion force for point:", current_point)
    for poly in obstacles:
        closest_point = poly.exterior.interpolate(poly.exterior.project(Point(current_point))).coords[0]
        obstacle_vector = current_point - np.array(closest_point)
        obstacle_distance = np.linalg.norm(obstacle_vector)
        #print(f"Obstacle distance: {obstacle_distance}, Repulsion distance: {repulsion_distance}")
        if 0 < obstacle_distance <= repulsion_distance:
            force_magnitude = (repulsion_distance - obstacle_distance) / obstacle_distance * repulsion_strength
            repulsion_force += (obstacle_vector * force_magnitude)
            #print(f"Repulsion force added: {obstacle_vector * force_magnitude}")
    #print(f"Total repulsion force: {repulsion_force}")
    return repulsion_force


def find_alternative_path(current_point, end_point, obstacles, attraction_strength, repulsion_distance, repulsion_force, angle_increment=np.pi/18, max_attempts=20):
    direction_to_goal = np.array(end_point) - np.array(current_point)
    direction_to_goal /= np.linalg.norm(direction_to_goal) if np.linalg.norm(direction_to_goal) > 0 else 1
    best_point = None
    best_distance = np.inf

    extra_attempts = max_attempts * 2 if np.linalg.norm(np.array(current_point) - np.array(end_point)) <= repulsion_distance else max_attempts
    for attempt in range(extra_attempts):
        angle = angle_increment * (attempt - extra_attempts // 2)
        rotation_matrix = np.array([[np.cos(angle), -np.sin(angle)], [np.sin(angle), np.cos(angle)]])
        new_direction = rotation_matrix.dot(direction_to_goal)

        for step_multiplier in np.linspace(0.1, 2.0, num=20):
            total_force = repulsion_force +  new_direction * attraction_strength #* step_multiplier
            test_point = np.array(current_point) + total_force
            test_point = tuple(test_point)
            
            if within_grid_limits(test_point) and is_path_clear(current_point, test_point, obstacles):
                distance_to_end = np.linalg.norm(np.array(end_point) - np.array(test_point))
                if distance_to_end < best_distance:
                    best_distance = distance_to_end
                    best_point = test_point
                    #print(f'test_point:{best_point}')

    if best_point is None:  # Se não encontrar um ponto viável, tente com direções aleatórias
        for attempt in range(max_attempts):
            random_direction = np.random.rand(2) - 0.5
            random_direction /= np.linalg.norm(random_direction)
            candidate_point = current_point + random_direction * repulsion_distance
            if within_grid_limits(candidate_point) and is_path_clear(current_point, candidate_point, obstacles):
                return candidate_point
                #print(f'candidate_point:{candidate_point}')

    return best_point


def within_grid_limits(point, grid_min_x=-15, grid_max_x=30, grid_min_y=-15, grid_max_y=30):
    x, y = point
    return grid_min_x <= x <= grid_max_x and grid_min_y <= y <= grid_max_y


def is_path_clear(start_point, end_point, obstacles):
    """
    Verifica se existe um caminho livre de obstáculos entre dois pontos.
    
    Args:
    - start_point: Tuple (x, y) representando o ponto de início.
    - end_point: Tuple (x, y) representando o ponto final.
    - obstacles: Lista de objetos shapely (Polygon ou MultiPolygon) representando os obstáculos.

    Returns:
    - Boolean: True se o caminho está livre, False caso contrário.
    """
    # Cria uma linha entre o ponto inicial e final
    path_line = LineString([start_point, end_point])

    # Unifica todos os obstáculos em uma única forma geométrica para otimizar a verificação de interseção
    unified_obstacles = unary_union([obstacle for obstacle in obstacles])

    # Verifica se a linha do caminho intersecta com algum dos obstáculos
    if path_line.intersects(unified_obstacles):
        return False  # Existe interseção, caminho não está livre
    else:
        return True  # Não existe interseção, caminho está livre



#def print_debug_info(iteration, current_point, total_force, repulsion_force, attraction_strength):
    #print(f"Iteração: {iteration}")
    #print(f"Ponto Atual: {current_point}")
    #print(f"Força Total: {total_force}")
    #print(f"Força de Repulsão: {repulsion_force}")

def replan_within_obstacle(current_point, end_point, obstacles, attraction_strength, repulsion_force, angle_increment=np.pi/18, max_attempts=30):
    """
    Tenta encontrar um caminho direcionado para o ponto final que esteja fora dos obstáculos.
    :param current_point: Ponto atual onde o obstáculo foi encontrado.
    :param end_point: Ponto de destino do segmento.
    :param obstacles: Lista de obstáculos no ambiente.
    :param angle_increment: Incremento do ângulo a ser testado a cada iteração (padrão de 10 graus).
    :param max_attempts: Número máximo de tentativas para encontrar um caminho alternativo.
    :return: Retorna o primeiro ponto de teste que está fora dos obstáculos e dentro dos limites ou None se nenhum for encontrado.
    """

    direction_to_goal = np.array(end_point) - np.array(current_point)
    direction_to_goal /= np.linalg.norm(direction_to_goal)
    best_point = None
    best_distance = np.inf

    for attempt in range(max_attempts):
        angle = angle_increment * attempt
        rotation_matrix = np.array([[np.cos(angle), -np.sin(angle)],
                                    [np.sin(angle), np.cos(angle)]])
        new_direction = rotation_matrix.dot(direction_to_goal)

        for step_multiplier in np.linspace(0.1, 2.0, num=20):  # Padronização do step_multiplier
            test_point = np.array(current_point) + repulsion_force + new_direction * attraction_strength #new_direction * attraction_strength * step_multiplier
            test_point = tuple(test_point)

            if within_grid_limits(test_point) and is_path_clear(current_point, test_point, obstacles):
                distance_to_end = np.linalg.norm(np.array(end_point) - np.array(test_point))
                if distance_to_end < best_distance:
                    best_distance = distance_to_end
                    best_point = test_point

            # Projeção
            if best_point is None:
                for poly in obstacles:
                    closest_point = poly.exterior.interpolate(poly.exterior.project(Point(current_point))).coords[0]
                    if is_path_clear(current_point, closest_point, obstacles):
                        return closest_point
    return None

def plot_current_segment(original_segment, current_adjusted_path, people, G, obstacles):
    plot_path(original_segment, current_adjusted_path, people, G, obstacles)

def plot_path(original_path, adjusted_path, people, G, obstacles):
    """
    Plota o caminho original e ajustado, e os obstáculos.
    :param original_path: Lista de tuplas (x, y) do caminho original.
    :param adjusted_path: Lista de tuplas (x, y) do caminho ajustado.
    :param obstacles: Lista de objetos Shapely Polygon representando obstáculos.
    """
    if adjusted_path is not None:
        fig, ax = plt.subplots()
        
        #plot original_path
        #flat_coords_result = [point for segment_group in original_path for point in segment_group]
        coords_x, coords_y = original_path
        #print(f'original_path:{original_path}')
        ax.plot(coords_x, coords_y, 'bx', label = 'Goals')
        #print(f'original_path:{original_path}')
        # Plot adjusted path
        axx, axy = zip(*adjusted_path)
        ax.plot(axx, axy, 'bo-', label='Social Path')
        #print(f'adjusted_path:{adjusted_path}')
        for obstacle in obstacles:
            if isinstance(obstacle, Polygon):
                x, y = obstacle.exterior.xy
                legend = ax.get_legend()
                if legend is None or 'Obstacle' not in [text.get_text() for text in legend.get_texts()]:
                    label = 'Obstacle'
                else:
                    label = ""
                ax.fill(x, y, alpha=0.8, fc='gray', ec='none', label=None)
            elif isinstance(obstacle, MultiPolygon):
                for poly in obstacle:
                    x, y = poly.exterior.xy
                    legend = ax.get_legend()
                    if legend is None or 'Obstacle' not in [text.get_text() for text in legend.get_texts()]:
                        label = 'Obstacle'
                    else:
                        label = ""
                    ax.fill(x, y, alpha=0.8, fc='gray', ec='none', label=None)
    
        # Desenha as pessoas
        for person in people:
            person.draw(ax)
    
        # Preparar o ambiente para desenho
        G.make_graph()
        G.boundary_estimate()
        # Desenha os detalhes do ambiente utilizando o objeto OverallDensity
        G.draw_overall(drawDensity=False, drawCluster=True, drawGraph=False, drawSegment=True, people=people)
    
        # Plot line from original goal to the end point of the social path
        original_goal = original_path  # Assume que original_path é um ponto (x, y)
        end_point = adjusted_path[-1]  # Último ponto de adjusted_path
        ax.plot([original_goal[0], end_point[0]], [original_goal[1], end_point[1]], 'b-')
    
        ax.set_aspect('equal', 'datalim')
        ax.legend()
        plt.show()

#modificações na smooth pra detectar obstaculos
def smooth_path(path, weight_data=0.5, weight_smooth=0.1, tolerance=0.00001, obstacles=None):
    new_path = np.copy(path)
    change = tolerance
    while change >= tolerance:
        change = 0.0
        for i in range(1, len(path)-1):
            for j in range(len(path[i])):
                aux = new_path[i][j]
                # Cálculo suave considerando o ponto anterior e o próximo
                new_x = new_path[i][j] + weight_data * (path[i][j] - new_path[i][j])
                new_x += weight_smooth * (new_path[i-1][j] + new_path[i+1][j] - (2.0 * new_path[i][j]))
                
                # Checa se o novo ponto ajustado intercepta algum obstáculo
                # Verificação de colisão
                if obstacles and is_path_clear(new_path[i-1], (new_x, new_path[i][1-j]), obstacles):
                    new_path[i][j] = new_x
                    change += abs(aux - new_x)
                else:
                    # Se interceptar, mantém o ponto antigo para evitar colisão
                    change += 0  # Não houve mudança neste passo, já que o ajuste foi impedido
    return new_path


# def integrated_elastic_band_algorithm(people, G, path, obstacles,  data, repulsion_distance=2.0, attraction_strength=1.0):
#     start, end = path
#     #print(f"path: {path}")
#     original_end_point = end
#     replanned_path = process_route([(start, end)], obstacles, data)
#     #print(f"replanned_path: {replanned_path}")
#     target_points = [end_point[1] for _, end_point in replanned_path]

#     # complete_adjusted_path = []
#     point_history = []

#     for segment_index, segment in enumerate(replanned_path):
#         start_point, end_point = map(np.array, segment)
#         # Criando um "corredor" inicial
#         corridor_length = calculate_comprimento_path(replanned_path)#0.5  # Define o comprimento do corredor
#         corridor_direction = (end_point - start_point) / np.linalg.norm(
#             end_point - start_point
#         )
#         corridor_point = start_point + corridor_direction * corridor_length
#         corridor_path = [(start_point), (corridor_point)]

#         # Verificando se o corredor intersecta algum obstáculo
#         if is_path_clear(start_point, corridor_point, obstacles):
#             # Se o corredor estiver livre, define o ponto final do corredor como o ponto inicial
#             current_point = corridor_point
#         else:
#             # Se o corredor intersecta, replaneja o ponto inicial
#             current_point = replan_endpoint(start_point, obstacles) 

#         segment_adjusted_path = [tuple(current_point)]  # Inicializa o caminho ajustado com o novo ponto inicial
#         point_history.append(current_point)
#         direction = (end_point - current_point) / np.linalg.norm(
#             end_point - current_point
#         )
#         max_iterations = 1000
#         iterations = 0
#         distance_to_end_threshold = 0.05#np.linalg.norm(end_point - start_point) * 0.01

#         #print(f"segment atual processado: {segment}")
#         #print(f"distance_to_end_threshold: {distance_to_end_threshold}")

#         # Força de Repulsão Ajustada
#         repulsion_strength = 1.5  # Aumentar a força de repulsão no início
#         while (
#             np.linalg.norm(end_point - current_point) > distance_to_end_threshold
#             and iterations < max_iterations
#         ):
#             iterations += 1
#             #print(f"Iteração: {iterations}")
#             is_close_to_target = np.any(
#                 [
#                     np.linalg.norm(np.array(target) - current_point)
#                     < 0.5
#                     for target in target_points
#                 ]
#             )
#             repulsion_strength = (
#                 0.5 if is_close_to_target else repulsion_strength
#             )  # Reduzir a força de repulsão ao se aproximar do objetivo
#             effective_repulsion_distance = (
#                 repulsion_distance if not is_close_to_target else 0.5
#             )
#             repulsion_force = calculate_repulsion_force(
#                 current_point, obstacles, effective_repulsion_distance, repulsion_strength
#             )
#             total_force = repulsion_force + direction * attraction_strength
#             next_point = current_point + total_force

#             #print(f"Ponto Atual: {current_point}")
#             #print(f"Força Total: {total_force}")
#             #print(f"Força de Repulsão: {repulsion_force}")

#             if is_path_clear(current_point, next_point, obstacles):
#                 #print(f"caminho está livre entre :{current_point} e {next_point}")
#                 if not any(
#                     p.contains(Point(next_point))
#                     for p in obstacles):
#                         #print(f"Nenhum obstáculo contém: {next_point}")
#                         if (
#                             np.linalg.norm(next_point - end_point)
#                             <= np.linalg.norm(current_point - end_point) 
#                         ):
#                             d = np.linalg.norm(next_point - end_point)
#                             #print(
#                              #   f"{next_point} ha uma distancia:{d}, ponto será adicionado ao caminho"
#                             #)
#                             current_point = next_point
#                         else:
#                             #verifique se a linha reta entre current_point e end_point intersectam obstaculo
#                             if not is_path_clear(current_point, end_point, obstacles):
#                             #se interceptou calcula alternative point
#                              #   print(f'caminho obstruido entre:{current_point} e {end_point}')
#                                 alternative_point = find_alternative_path(
#                                         current_point, end_point, obstacles, attraction_strength, repulsion_distance, repulsion_force
#                                     )
#                                 if alternative_point is not None:
#                                     if (
#                                         within_grid_limits(alternative_point)
#                                         and is_path_clear(
#                                             current_point, alternative_point, obstacles
#                                         )
#                                     ):
#                                         current_point = alternative_point
#                               #          print(f'novo ponto adicionado:{alternative_point}')
#                             else:
#                                 current_point = end_point
#                                # print(
#   #                              f"ponto:{next_point} não foi adicionado, caminho muito próximo do end_point d:{d}"
#                             #)
#                         segment_adjusted_path.append(tuple(current_point))
#             else:
#                 #print(
#                  #   f"ponto {next_point} está dentro do obstáculo, vamos calcular caminho alternativo"
#                 #)
#                 alternative_point = find_alternative_path(
#                     current_point, end_point, obstacles, attraction_strength, repulsion_distance, repulsion_force
#                 )
#                 if alternative_point is not None:
#                     if (
#                         within_grid_limits(alternative_point)
#                         and is_path_clear(
#                             current_point, alternative_point, obstacles
#                         )
#                     ):
#                  #       print(
#                  #           f"Ponto alternativo encontrado: {alternative_point}"
#                  #       )
#                         segment_adjusted_path.append(tuple(alternative_point))
#                         current_point = alternative_point
#                     else:
#                         if not is_path_clear(
#                             current_point, next_point, obstacles
#                         ):
#                             current_point = replan_within_obstacle(
#                                 current_point, end_point, obstacles, attraction_strength, repulsion_force 
#                             )
#                             if current_point is None:
#                                 print(
#                                     "Falha ao replanejar. Tentando um ponto anterior no caminho."
#                                 )
#                                 if point_history:
#                                     current_point = point_history.pop()
#                                 else:
#                                     print(
#                                         "Sem pontos no histórico para retornar."
#                                     )
#                                     final_path = None
#                                     break
#                             else:
#                                 point_history.append(current_point)
#                                 segment_adjusted_path.append(tuple(current_point))
#                         else:
#                             point_history.append(current_point)
#                             segment_adjusted_path.append(tuple(current_point))
#                 else:
#                     print("Não foi possível encontrar um ponto alternativo.")
#                     if point_history:
#                         current_point = point_history.pop()
#                         continue
#                     else:
#                         print("Sem pontos no histórico para retornar.")
#                         final_path= None
#                         break
def integrated_elastic_band_algorithm(people, G, path, obstacles, data, repulsion_distance=2.0, attraction_strength=1.0):
    start, end = path
    original_end_point = end
    replanned_path = process_route([(start, end)], obstacles, data)
    target_points = [end_point[1] for _, end_point in replanned_path]

    point_history = []
    final_path = []

    for segment_index, segment in enumerate(replanned_path):
        start_point, end_point = map(np.array, segment)
        corridor_length = calculate_comprimento_path(replanned_path)
        corridor_direction = (end_point - start_point) / np.linalg.norm(end_point - start_point)
        corridor_point = start_point + corridor_direction * corridor_length
        corridor_path = [(start_point), (corridor_point)]

        if is_path_clear(start_point, corridor_point, obstacles):
            current_point = corridor_point
        else:
            current_point = replan_endpoint(start_point, obstacles)

        segment_adjusted_path = [tuple(current_point)]
        point_history.append(current_point)
        direction = (end_point - current_point) / np.linalg.norm(end_point - current_point)
        max_iterations = 1000
        iterations = 0
        distance_to_end_threshold = 0.05

        repulsion_strength = 1.5
        while (
            np.linalg.norm(end_point - current_point) > distance_to_end_threshold
            and iterations < max_iterations
        ):
            iterations += 1
            is_close_to_target = np.any(
                [
                    np.linalg.norm(np.array(target) - current_point)
                    < 0.5
                    for target in target_points
                ]
            )
            repulsion_strength = 0.5 if is_close_to_target else repulsion_strength
            effective_repulsion_distance = repulsion_distance if not is_close_to_target else 0.5
            repulsion_force = calculate_repulsion_force(
                current_point, obstacles, effective_repulsion_distance, repulsion_strength
            )
            total_force = repulsion_force + direction * attraction_strength
            next_point = current_point + total_force

            if is_path_clear(current_point, next_point, obstacles):
                if not any(p.contains(Point(next_point)) for p in obstacles):
                    if np.linalg.norm(next_point - end_point) <= np.linalg.norm(current_point - end_point):
                        current_point = next_point
                    else:
                        if not is_path_clear(current_point, end_point, obstacles):
                            alternative_point = find_alternative_path(
                                current_point, end_point, obstacles, attraction_strength, repulsion_distance, repulsion_force
                            )
                            if alternative_point is not None:
                                if within_grid_limits(alternative_point) and is_path_clear(current_point, alternative_point, obstacles):
                                    current_point = alternative_point
                            else:
                                current_point = end_point
                        else:
                            current_point = end_point
                    segment_adjusted_path.append(tuple(current_point))
            else:
                alternative_point = find_alternative_path(
                    current_point, end_point, obstacles, attraction_strength, repulsion_distance, repulsion_force
                )
                if alternative_point is not None:
                    if within_grid_limits(alternative_point) and is_path_clear(current_point, alternative_point, obstacles):
                        segment_adjusted_path.append(tuple(alternative_point))
                        current_point = alternative_point
                    else:
                        if not is_path_clear(current_point, next_point, obstacles):
                            current_point = replan_within_obstacle(current_point, end_point, obstacles, attraction_strength, repulsion_force)
                            if current_point is None:
                                if point_history:
                                    current_point = point_history.pop()
                                else:
                                    final_path = None
                                    break
                            else:
                                point_history.append(current_point)
                                segment_adjusted_path.append(tuple(current_point))
                        else:
                            point_history.append(current_point)
                            segment_adjusted_path.append(tuple(current_point))
                else:
                    if point_history:
                        current_point = point_history.pop()
                        continue
                    else:
                        final_path = None
                        break

        if iterations >= max_iterations and not np.array_equal(current_point, end_point):
            final_path = None
            break

        if final_path is None:
            break
        else:
            point_history.append(current_point)

    if final_path is not None:
        final_path = segment_adjusted_path
    else:
        final_path = None
    
    #print_debug_info(
    #    iterations, current_point, total_force, repulsion_force, attraction_strength
    #)
    #print(f"final_path: {final_path}")
    #plot_current_segment(original_end_point, final_path, people, G, obstacles)

    return final_path

            
            # print_debug_info(
            #     iterations, current_point, total_force, repulsion_force, attraction_strength
            # )

        #deforma o caminho se o elastico cruza algum obstaculo:
        #teste antes do deformed:
        #print("plot do caminho sem usar deforme_path")
        #plot_current_segment(original_end_point, segment_adjusted_path, people, G, obstacles)
        #path_deformed = deforma_elastic(segment_adjusted_path, obstacles, repulsion_distance=1.0, repulsion_strength=1.0)
        #final_path = smooth_path(path_deformed)
#        final_path = smooth_path(segment_adjusted_path)
#        print(f"final_path: {final_path}")
#        plot_current_segment(original_end_point, final_path, people, G, obstacles)

#    return final_path