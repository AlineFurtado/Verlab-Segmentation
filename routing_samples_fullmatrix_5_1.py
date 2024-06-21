# -*- coding: utf-8 -*-
"""
Created on Thu Apr  4 13:47:12 2024

@author: User-Aline

Gera um arquivo no formato COPS com os segmentos classificados em  G.discretize_clusters_to_approach_regions(people)
porém, não usa todos os segmentos, mas seleciona amostras 1 ou 2 no máximo em cada segmento.

inclui função para plotar a rota obtida (desconsidera as restrições sociais na plotagem)

"""

import os
import numpy as np
#from path_planing6 import *
from path_planing2_2_21 import *


def routing(people, G, segments_samples, obstacles, data):
    tmax = 100
    gtsp_subgroup_section = [(0, 0, 0)]
    gtsp_cluster_section = [(0, 0)]
    
    segments = []
    segment_cluster_map = {}
    segment_index = 0

    # Inicialização dos segmentos e mapeamento
    for cluster_id, cluster_data in segments_samples.items():
        #print(f'cluster_id:{cluster_id}')
        for segment in cluster_data['approach_segments_samples']:
            segments.append(segment)
            segment_cluster_map[segment_index] = cluster_id
            segment_index += 1

    num_segments = len(segments)
    #print(f'num_segments:{num_segments}')
    full_matrix = np.zeros((num_segments + 1, num_segments + 1))
    path_storage = {}
    
    # Preenchimento da matriz de distâncias
    for i in range(num_segments + 1):
        print(f'i:{i}')
        for j in range(num_segments + 1):  # (i, num_segments +1)Ajuste para começar de i e evitar redundâncias
            print(f'j:{j}')
            if not full_matrix[i, j]: #se o elemento da matriz ainda não foi calculado
                if i == j:
                    full_matrix[i, j] = 0
                    full_matrix[j, i] = 0
                elif i == 0 or j == 0:
                    start = (0, 0)
                    end = segments[max(i, j) - 1]['median_point']
                    print(f'start:{start}')
                    print(f'end:{end}')
                    if (j, i) not in path_storage:
                        path_storage[(i, j)] = []
                        path_storage[(j, i)] = []
                        path = integrated_elastic_band_algorithm(people, G, (start, end), obstacles, data, repulsion_distance=2.0, attraction_strength=1.0)
                        if path is not None:
                            comprimento = calculate_comprimento_path(path)
                            path_storage[(i, j)].append(start)
                            path_storage[(i, j)].extend(path)
                            path_storage[(i, j)].append(end) # Armazenamento em ambas direções
                        else: 
                            comprimento = float('inf')
                            path_storage[(i, j)] = None
                        full_matrix[i, j] = comprimento
                        full_matrix[j, i] = comprimento  # Simetria da matriz
                        path_storage[(j,i)] =path_storage[(i,j)]
                else:
                    segment_i = segments[i - 1]
                    segment_j = segments[j - 1]
                    if segment_cluster_map[i - 1] == segment_cluster_map[j - 1]:
                        full_matrix[i, j] = float('inf')
                        full_matrix[j, i] = float('inf')
                    elif (j, i) not in path_storage:
                        path_storage[(i, j)] = []
                        path_storage[(j, i)] = []
                        start = segment_i['median_point']
                        end = segment_j['median_point']
                        print(f'start:{start}')
                        print(f'end:{end}')
                        path = integrated_elastic_band_algorithm(people, G, (start, end), obstacles, data)
                        if path is not None:
                            comprimento = calculate_comprimento_path(path)
                            path_storage[(i, j)].append(start)
                            path_storage[(i, j)].extend(path)
                            path_storage[(i, j)].append(end)
                        else:
                            comprimento = float('inf')
                            path_storage[(i, j)]=None
                        full_matrix[i, j] = comprimento
                        full_matrix[j, i] = comprimento  # Simetria da matriz
                        path_storage[(j,i)] =path_storage[(i,j)]
            else:
                print(f'elemento:[{i},{j}] já calculado')
            print(f'full_matrix[{i},{j}]:{full_matrix[i,j]}')
    inf_value = 999999
    # Formatando a matriz full_matrix para preservação de precisão
    full_matrix_str = '\n'.join('  '.join(f"{inf_value if val == float('inf') else val:.12f}" for val in row) for row in full_matrix)

    id_vertex = 1
    for cluster_idx, cluster_data in segments_samples.items():
        for segment in cluster_data['approach_segments_samples']:
            id_segment = segment['id_segment']
            #print(f'id_segment :{id_segment }')
            profit = segment['profit']
            #print(f'profit:{profit}')
            #print(f'cluster_idx:{cluster_idx}')
            #print(f'id_vertex:{id_vertex}')
            gtsp_cluster_section.append((cluster_idx, id_vertex))
            gtsp_subgroup_section.append((id_vertex, profit, id_segment))
            id_vertex += 1
    #print(f'gtsp_cluster_section:{gtsp_cluster_section}')
    # Reorganiza clusters
    # Corrigido para gerar listas de subgrupos corretamente
    id_subgroup_list = [[] for _ in range(max(cluster_id for cluster_id, _ in gtsp_cluster_section) + 1)]
    for cluster_idx, id_vertex in gtsp_cluster_section:
        id_subgroup_list[cluster_idx].append(id_vertex)
    # Geração do arquivo .COPS
    file_path = os.path.join("datasets", "output7.COPS")
    with open(file_path, "w") as f:
        f.write("NAME: COPS for social segmentation routing\n")
        f.write("TYPE: OP\n")
        f.write("COMMENT: Cada segmento classificado equivale a um vértice, e cada vértice é um subgrupo\n")
        f.write(f"DIMENSION: {num_segments + 1}\n")
        f.write(f"TMAX: {tmax}\n")
        f.write("START_CLUSTER: 0\n")
        f.write("END_CLUSTER: 0\n")
        f.write(f"CLUSTERS: {len(G.cluster) + 1}\n")
        f.write(f"SUBGROUPS: {num_segments + 1}\n")
        f.write("EDGE_WEIGHT_TYPE: EXPLICIT\n")
        f.write("EDGE_WEIGHT_FORMAT: FULL_MATRIX\n")
        f.write("EDGE_WEIGHT_SECTION\n")
        f.write(full_matrix_str + '\n')
        f.write("GTSP_SUBGROUP_SECTION: subgroup_id subgroup_profit id-vertex-list\n")
        for subgroup_info in gtsp_subgroup_section:
            f.write(" ".join(map(str, subgroup_info)) + "\n")
        f.write("GTSP_CLUSTER_SECTION: cluster_id id-subgroup-list\n")
        for idx, subgroup_list in enumerate(id_subgroup_list):
            f.write(f"{idx} {' '.join(map(str, subgroup_list))}\n")

    print(f'path_storage:{path_storage}')
    return path_storage

def extract_final_tour(path_storage, route_result):
    final_tour = []

    for (start, end) in route_result['route']:
        if (start, end) in path_storage:
            path_segment = path_storage[(start, end)]
            final_tour.append(path_segment)
        else:
            print(f'Path segment ({start}, {end}) not found in path_storage')
    
    return final_tour

def plot_tour(final_tour, people, G):

    fig, ax = plt.subplots()
        
    for segment in final_tour:
        if segment:  # Certifique-se de que o segmento não está vazio
            axx, axy = zip(*segment)
            ax.plot(axx, axy, 'bo-', label='Social Path')
            
        # Desenha as pessoas
        for person in people:
            person.draw(ax)

        # Preparar o ambiente para desenho
        G.make_graph()
        G.boundary_estimate()
        G.discretize_clusters_to_approach_regions(people)
        # Desenha os detalhes do ambiente utilizando o objeto OverallDensity
        G.draw_overall(drawDensity=False, drawCluster=True, drawGraph=False, drawSegment=True, people=people)

    ax.set_aspect('equal', 'datalim')
    ax.legend()
    plt.show()
