# -*- coding: utf-8 -*-
"""
Created on Mon Mar 18 15:45:47 2024

@author: User-Aline
"""

import matplotlib.pyplot as plt
from overall_density8_1 import OverallDensity
from person import Person
from F_formation import F_formation
from obstacles_manipulation2 import *
import time
import os
from force_and_movements4_1 import *
from robot4_2 import *
from process_segments3 import process_clusters
from routing_samples_fullmatrix_5_1 import routing, extract_final_tour, plot_tour
from path_planing2_2_21 import *
from grafo import COPS  # Importe a classe COPS do seu módulo
from tabu_search import receive_data, spreadsheet_header, main
from matplotlib.animation import FuncAnimation, PillowWriter
from shapely.geometry import Point, Polygon
import numpy as np

# Elementos da parte estática do ambiente
f_formation = F_formation()
static_people = []
# group1 = f_formation.Face_to_face(x1=10, y1=7, th1=np.deg2rad(0), x2=12.2, y2=7, th2=np.deg2rad(180))
# p, xc, yc, rc = group1
# for sperson in p:
#     static_people.append(sperson)
p1 = Person(3, 10, np.deg2rad(0), id_node=0)
static_people.append(p1)
# group= f_formation.semi_circle(0, 12, 1.2)
# p_, xc, yc, rc = group
# for sperson in p_:
#     static_people.append(sperson)
# # Inicialização do grafo
Gstatico = OverallDensity(person=static_people, zone='Social', map_resolution=400, window_size=1)
Gstatico.make_graph()
Gstatico.boundary_estimate()
data = Gstatico.discretize_clusters_to_approach_regions(static_people)

# # Pré-processamento do ambiente estático
# min_profit = 50
# max_profit = 100
# processed_segments = process_clusters(data, min_profit, max_profit)
# # # Cria os obstáculos
obstacles = extract_polygons(data)
#path_storage = routing(static_people, Gstatico, processed_segments, obstacles, data)

# # # # Chamada para obter o resultado da rota
# dir = fr"{os.getcwd()}\datasets"
# problem = "output3"
# # Getting parsed problem
# args = receive_data()
# if args.path:
#     dir = os.path.dirname(args.path)
#     problem = os.path.basename(args.path).split('.')[0]
# results_path = fr"{dir}\results"
# if not os.path.exists(results_path):
#     os.makedirs(results_path)
# results_file = fr"{results_path}\{problem}.csv"
# spreadsheet_header(results_file)
# route_result = main(dir, problem, results_file)
# print(f'route_result:{route_result}')

# final_tour = extract_final_tour(path_storage, route_result)
# plot_tour(final_tour, static_people, Gstatico, obstacles)
# print(f'final_tour:{final_tour}')
#final_tour=[[(0.0, 0.0), (0.0, 0.0), (-0.06605455157002893, 0.9978160132092903), (-0.13210910314005786, 1.9956320264185805), (-0.1981636547100868, 2.9934480396278706), (-0.2642182062801157, 3.991264052837161), (-0.33027275785014465, 4.989080066046451), (-0.6402813440223815, 9.67205079497626), (0.002506265664159457, 10.438095238095237)], [(0.002506265664159457, 10.438095238095237), (-0.6402813440223815, 9.67205079497626), (-2.63833691355985, 4.939060824543208), (-1.728950711057021, 4.523108021093725), (-0.7448790046668972, 4.2796469066756115), (0.16450719783593137, 4.2726117935477435), (1.07389340033876, 4.28668201980348), (1.9832796028415884, 4.258541567292007), (2.892665805344417, 4.314822472314952), (3.8020520078472453, 4.2022606622690635), (4.412931468962863, 4.715133465716752), (5.038068060955526, 4.6647941942733935), (5.947454263458354, 4.24884139082391), (6.856840465961183, 3.832888587374426), (7.766226668464011, 3.4169357839249423), (8.67561287096684, 3.0009829804754586), (9.584999073469668, 2.585030177025975), (10.40834020986233, 4.6184167097632285), (11.05112781954887, 5.3844611528822055)], [(11.05112781954887, 5.3844611528822055), (10.40834020986233, 4.6184167097632285), (8.995161621673546, 3.8367499323832206), (8.226506059646471, 3.6502873928774315), (7.312449830796863, 3.244699904779939), (6.398393601947255, 2.839112416682447), (5.484337373097647, 2.4335249285849545), (4.570281144248039, 2.027937440487462), (3.6562249153984308, 1.6223499523899698), (2.742168686548823, 1.2167624642924775), (1.8281124576992154, 0.8111749761949849), (0.9140562288496077, 0.40558748809749245), (0.0, 0.0), (0.0, 0.0)]]
###para testes no dinamico
#cenario 1 person: 
final_tour=[[(0.0, 0.0), (0.0, 0.0), (0.46122657345899426, 0.8872823946947641), (0.9224531469179885, 1.7745647893895282), (1.3836797203769828, 2.6618471840842926), (1.844906293835977, 3.5491295787790564), (2.3061328672949712, 4.43641197347382), (2.7673594407539657, 4.669076407957854), (3.22858601421296, 4.5527441907158375), (3.6898125876719545, 4.610910299336846), (4.1510391611309485, 4.581827245026341), (4.703418163810947, 4.613239607717829), (5.466628992762149, 4.905172285623128), (6.131953319435037, 5.640097214480681), (6.593179892894031, 6.527379609175445), (7.054406466353025, 7.414662003870209), (7.515633039812019, 8.301944398564974), (5.192982456140351, 9.989974937343359), (4.192982456140351, 9.989974937343359)], [(4.192982456140351, 9.989974937343359), (5.192982456140351, 9.989974937343359), (7.515633039812019, 8.301944398564974), (7.054406466353025, 7.414662003870209), (6.593179892894031, 6.527379609175445), (6.131953319435037, 5.640097214480681), (5.466628992762149, 4.905172285623128), (4.703418163810947, 4.613239607717829), (4.1510391611309485, 4.581827245026341), (3.6898125876719545, 4.610910299336846), (3.22858601421296, 4.5527441907158375), (2.7673594407539657, 4.669076407957854), (2.3061328672949712, 4.43641197347382), (1.844906293835977, 3.5491295787790564), (1.3836797203769828, 2.6618471840842926), (0.9224531469179885, 1.7745647893895282), (0.46122657345899426, 0.8872823946947641), (0.0, 0.0), (0.0, 0.0)]]
# cenario 2 people final_tour=[[(0.0, 0.0), (0.0, 0.0), (0.9145266388250867, 0.40452568135940337), (1.8290532776501733, 0.8090513627188067), (2.74357991647526, 1.21357704407821), (3.6581065553003467, 1.6181027254376135), (4.572633194125434, 2.0226284067970166), (5.48715983295052, 2.4271540881564198), (6.401686471775607, 2.831679769515823), (7.316213110600693, 3.236205450875226), (8.23073974942578, 3.640731132234629), (8.984414043574418, 3.8229589841355116), (10.44443043542624, 4.619920469161725), (11.087218045112781, 5.385964912280702)], [(11.087218045112781, 5.385964912280702), (10.44443043542624, 4.619920469161725), (8.984414043574418, 3.8229589841355116), (8.23073974942578, 3.640731132234629), (7.316213110600693, 3.236205450875226), (6.401686471775607, 2.831679769515823), (5.48715983295052, 2.4271540881564198), (4.572633194125434, 2.0226284067970166), (3.6581065553003467, 1.6181027254376135), (2.74357991647526, 1.21357704407821), (1.8290532776501733, 0.8090513627188067), (0.9145266388250867, 0.40452568135940337), (0.0, 0.0), (0.0, 0.0)]]
# cenario 3 output1 final_tour=[[(0.0, 0.0), (0.0, 0.0), (0.9138747304856061, 0.4059962770517247), (1.8277494609712122, 0.8119925541034494), (2.7416241914568182, 1.217988831155174), (3.6554989219424243, 1.6239851082068988), (4.56937365242803, 2.0299813852586235), (5.4832483829136365, 2.435977662310348), (6.3971231133992426, 2.841973939362073), (7.310997843884849, 3.2479702164137976), (8.224872574370455, 3.6539664934655223), (8.974603658775866, 3.8213345684099265), (10.427387828909948, 4.632451797482527), (11.07017543859649, 5.398496240601504)], [(11.07017543859649, 5.398496240601504), (10.427387828909948, 4.632451797482527), (9.860635740816674, 2.814053463522712), (9.159427173801552, 3.5270096652241905), (8.223702595947872, 3.522742960242523), (7.455476895278628, 4.13090873477574), (6.727050814690207, 4.820394632879249), (6.025842247675085, 5.533350834580727), (5.324633680659963, 6.246307036282205), (5.6364745926548965, 6.9592632379836825), (5.48055413665743, 7.67221943968516), (5.438602141738484, 8.44790108305405), (5.582533982793824, 8.500111147536815), (5.65519003267713, 8.67076812993434), (5.795081541766637, 8.85765446585401), (5.9437301357839845, 9.143331737917899), (6.105577648968426, 9.586386765447878), (5.172932330827067, 9.974937343358395), (4.172932330827067, 9.974937343358395)], [(4.172932330827067, 9.974937343358395), (5.172932330827067, 9.974937343358395), (7.019825345740742, 9.2158187008755), (6.716156685694232, 8.280034974581167), (6.8621577759568435, 7.389870852293591), (6.5825348442318505, 6.510872070146305), (6.122165596053272, 5.623144547011255), (5.451140324378421, 4.895720674484185), (4.6904446882717625, 4.6118180266956585), (4.143323233607207, 4.582244552939109), (3.6829539854286284, 4.610965940391881), (3.22258473725005, 4.553523165486337), (2.762215489071471, 4.668408715297425), (2.3018462408928926, 4.4386376156752485), (1.8414769927143142, 3.5509100925401986), (1.3811077445357356, 2.6631825694051487), (0.9207384963571571, 1.7754550462700993), (0.46036924817857855, 0.8877275231350497), (0.0, 0.0), (0.0, 0.0)]]
#plot_tour(final_tour, static_people, Gstatico, obstacles)
#Dinâmico
WORLD_SIZE = 20
NUM_PEOPLE = 2
NUM_GOALS = 2
CHANGE_INTERVAL = 5
MIN_DISTANCE_BETWEEN_GOALS = 5

people = generate_people(NUM_PEOPLE, WORLD_SIZE, obstacles)
#goals = generate_goals(NUM_GOALS, WORLD_SIZE, obstacles, people, MIN_DISTANCE_BETWEEN_GOALS)
#print(f'goals:{goals}')
goals =[np.array([-5.0, 4.660000000000000]), np.array([17.0, 3.0])]

robot = Robot(path=final_tour)
clusters = []

last_change_time = time.time()
start_time = time.time()
t = 0

# Cria a figura inicial
fig, ax = plt.subplots()
ax.set_xlim(-5, WORLD_SIZE + 5)
ax.set_ylim(-5, WORLD_SIZE + 5)
ax.set_aspect('equal', adjustable='box')

# Desenha as posições iniciais das pessoas que se movem
for person in people:
    person.draw(ax)
G = OverallDensity(person=people, zone='Social', map_resolution=200, window_size=1)
G.make_graph()
G.boundary_estimate()
G.cluster
G.discretize_clusters_to_approach_regions(people)
G.draw_overall(drawDensity=False, drawCluster=True, drawGraph=False, drawSegment=True, people=people, plot=ax)

# Desenha as pessoas fixas (obstáculos)
for spersons in static_people:
    spersons.draw(ax)
Gstatico.draw_overall(drawDensity=False, drawCluster=True, drawGraph=False, drawSegment=True, people=static_people, plot=ax)

# Desenha os obstáculos não humanos
for polygon in obstacles:
    x, y = polygon.exterior.xy
    ax.plot(x, y)
    ax.fill(x, y, alpha=0.2)

# Desenha as posições iniciais dos objetivos
for goal in goals:
    ax.plot(goal[0], goal[1], "g*", markersize=10)

# Desenha a posição inicial do robô
robot.draw(ax)

plt.title('Posições Iniciais')
plt.savefig('initial_figure.png')
plt.show()

def update(frame, people, goals, obstacles, ax, static_people, start_time, WORLD_SIZE, last_change_time, t, CHANGE_INTERVAL, robot):
    current_time = time.time()
    if current_time - last_change_time > CHANGE_INTERVAL:
        change_person_goal(people, goals, obstacles)
        last_change_time = current_time

    
    move_towards_goals(people, goals, obstacles)

    ax.clear()
    #fig, ax = plt.subplots()
    for spersons in static_people:
        spersons.draw(ax)
    Gstatico.draw_overall(drawDensity=False, drawCluster=True, drawGraph=False, drawSegment=True, people=static_people, plot=ax)

    for polygon in obstacles:
        x, y = polygon.exterior.xy
        ax.plot(x, y)
        ax.fill(x, y, alpha=0.2)

    for person in people:
        person.draw(ax)

    G = OverallDensity(person=people, zone='Social', map_resolution=200, window_size=1)
    G.make_graph()
    G.boundary_estimate()
    G.cluster
    clusters = G.discretize_clusters_to_approach_regions(people)
    G.draw_overall(drawDensity=False, drawCluster=True, drawGraph=False, drawSegment=True, people=people, plot=ax)
    move_obstacles = extract_move_obstacles(clusters)

    robot.move_obstacles = move_obstacles  # Atualiza os obstáculos móveis do robô
    move_robot(robot, people, obstacles, move_obstacles)
    robot.draw(ax)

    elapsed_time = time.time() - start_time
    plt.title(f"Elapsed time: {elapsed_time:.2f} seconds")

    all_x = [robot.x] + [person.x for person in people] + [goal[0] for goal in goals]
    all_y = [robot.y] + [person.y for person in people] + [goal[1] for goal in goals]

    buffer = 5
    min_x, max_x = min(all_x) - buffer, max(all_x) + buffer
    min_y, max_y = min(all_y) - buffer, max(all_y) + buffer

    ax.set_xlim(min_x, max_x)
    ax.set_ylim(min_y, max_y)
    t += 1
    #plt.show()
    return ax

fig, ax = plt.subplots()

last_change_time = time.time()
t = 0
start_time = time.time()
ani = FuncAnimation(fig, update, frames=frame_generator(robot, people, goals, obstacles), 
                    fargs=(people, goals, obstacles, ax, static_people, start_time, WORLD_SIZE, last_change_time, t, CHANGE_INTERVAL, robot), 
                    interval=100, blit=False, save_count=700, cache_frame_data=False)

print("Animação Criada com sucesso")
ani.save('animation.gif', writer=PillowWriter(fps=10))
print("Animação salva como 'animation.gif'")
plt.close(fig)