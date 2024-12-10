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
from robot4_2_updated import *
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
group4 = f_formation.retangular(x1=2, y1= 10, th1=np.deg2rad(135), x2= -1, y2 = 10, th2=np.deg2rad(45), x3=-1, y3=12, th3=np.deg2rad(-45), x4=2, y4=12, th4=np.deg2rad(-135))
p4, xc, yc, rc = group4
for sperson in p4:
     static_people.append(sperson)
group1 = f_formation.Face_to_face(x1=10, y1=7, th1=np.deg2rad(0), x2=12.2, y2=7, th2=np.deg2rad(180))
p, xc, yc, rc = group1
for sperson in p:
     static_people.append(sperson)
#group2 = f_formation.Side_by_side(x1=-3.5, y1=5.0, th1=np.deg2rad(0), x2= -3.5, y2=7.75, th2=np.deg2rad(0))
#p_2, xc, yc, rc = group2
#for sperson in p_2:
#    static_people.append(sperson)
#p1 = Person(2, 12, np.deg2rad(0), id_node=0)
#static_people.append(p1)
#p2 = Person (-8, 2.5, np.deg2rad(0), id_node=1)
#static_people.append(p2)
#p3 = Person (6, -5, np.deg2rad(180), id_node=2)
#static_people.append(p3)
#p4 = Person(-6, 10, np.deg2rad(-90), id_node=3)
#static_people.append(p4)
#p5 = Person (10, 7, np.deg2rad(-60), id_node=4)
#static_people.append(p5)
#group= f_formation.semi_circle(0, 12, 1.2)
#p_, xc, yc, rc = group
#for sperson in p_:
#     static_people.append(sperson)
#group = f_formation.Circular(2, 10, 1.5)
#p_, xc, yc, rc = group
#for sperson in p_:
#     static_people.append(sperson)
####################################################
#cenario para variação do budget
# p= Person(-5, 8, np.deg2rad(30), id_node=0)
# static_people.append(p)
# p1 = Person(-3, 10, np.deg2rad(225), id_node=1)
# static_people.append(p1)
# p2 = Person(-3, 12, np.deg2rad(225), id_node=2)
# static_people.append(p2)
# p3 = Person(9, 8, np.deg2rad(180), id_node=3)
# static_people.append(p3)
# p4 = Person(10, 6, np.deg2rad(180), id_node=4)
# static_people.append(p4)
# p5 = Person(9, 4, np.deg2rad(180), id_node=5)
# static_people.append(p5)
# p6 = Person(-2, -11, np.deg2rad(45), id_node=6)
# static_people.append(p6)
# p7 = Person(-2, -9, np.deg2rad(-90), id_node=7)
# static_people.append(p7)
# p8 = Person(1, -9, np.deg2rad(-90), id_node=8)
# static_people.append(p8)
# p9 = Person(1, -11, np.deg2rad(180), id_node=9)
# static_people.append(p9)


# # Inicialização do grafo
Gstatico = OverallDensity(person=static_people, zone='Social', map_resolution=400, window_size=1)
Gstatico.make_graph()
Gstatico.boundary_estimate()
data = Gstatico.discretize_clusters_to_approach_regions(static_people)

# # Pré-processamento do ambiente estático
min_profit = 50
max_profit = 100
processed_segments = process_clusters(data, min_profit, max_profit)
# # # Cria os obstáculos
obstacles = extract_polygons(data)

#Extrair pontos medianos de segments_samples para criar o caminho original
path_storage = routing(static_people, Gstatico, processed_segments, obstacles, data)

# # # # Chamada para obter o resultado da rota
dir = fr"{os.getcwd()}\dataset"
problem = "output23"
# Getting parsed problem
args = receive_data()
if args.path:
    dir = os.path.dirname(args.path)
    problem = os.path.basename(args.path).split('.')[0]
results_path = fr"{dir}\results"
if not os.path.exists(results_path):
    os.makedirs(results_path)
results_file = fr"{results_path}\{problem}.csv"
spreadsheet_header(results_file)
route_result = main(dir, problem, results_file)
print(f'route_result:{route_result}')

final_tour = extract_final_tour(path_storage, route_result)
plot_tour(final_tour, static_people, Gstatico, obstacles)
#print(f'final_tour:{final_tour}')
#output15 budget150
#final_tour=[[(0.0, 0.0), (0.0, 0.0), (0.3461882481895845, -0.9381650690659009), (0.692376496379169, -1.8763301381318018), (1.0385647445687534, -2.8144952071977025), (1.3847529927583389, -2.5309174654670494), (1.7309412409479235, -2.672706336332376), (2.077129489137507, -2.6018119008997127), (2.4233177373270918, -2.6372591186160443), (2.7695059855166764, -2.619535509757879), (3.1156942337062605, -2.6283973141869614), (3.5404947111956506, -2.6345820987460007), (4.180115902935484, -2.8314659200179113), (4.887885520594324, -3.411598229877348), (5.26635945548958, -4.339700742183313), (5.612547703679164, -5.277865811249214), (5.958735951868748, -6.216030880315115), (6.304924200058332, -7.154195949381016), (6.6511124482479165, -8.092361018446917), (3.413533834586465, -9.250626566416042), (2.41353383, -9.25062657)], [(2.41353383, -9.25062657), (3.413533834586465, -9.250626566416042), (3.4024883992603154, 6.157554868664452), (6.2556390977443606, 7.142857142857144), (7.2556391, 7.14285714)], [(7.2556391, 7.14285714), (6.2556390977443606, 7.142857142857144), (5.404431679564068, 7.712460730790307), (4.583137736656235, 7.611325898023687), (3.588265008312922, 7.510191065257067), (2.593392279969609, 7.409056232490447), (1.5985195516262958, 7.307921399723827), (-0.5983419857479187, 8.098584715593043), (-1.5932147140909714, 5.005100234347153), (-3.581453634085216, 6.142857142857142), (-3.58145363, 7.14285714)], [(-3.58145363, 7.14285714), (-3.581453634085216, 6.142857142857142), (-5.535529163856738, 4.27840378094294), (-5.0318553862749775, 3.4145098489255195), (-4.29702947365759, 2.7698491699373893), (-3.539018681702125, 2.5772721458172607), (-3.0220426654905626, 2.573960224506533), (-2.518368887908802, 2.5798674150217753), (-2.014695110327042, 2.5680530339912897), (-1.5110213327452815, 2.591681796052261), (-1.007347555163521, 1.7277878640348407), (-0.5036737775817605, 0.8638939320174204), (0.0, 0.0), (0.0, 0.0)]]
#final_tour = [[(0.0, 0.0), (0.0, 0.0), (-0.06760851165369312, 0.997711926936815), (-0.13521702330738625, 1.99542385387363), (-0.20282553496107936, 2.993135780810445), (-0.2704340466147725, 3.99084770774726), (-0.33804255826846563, 4.988559634684075), (-0.6558201911401754, 9.678065832570246), (-0.013032581453634506, 10.444110275689223)], [(-0.013032581453634506, 10.444110275689223), (-0.6558201911401754, 9.678065832570246), (-0.33804255826846563, 4.988559634684075), (-0.2704340466147725, 3.99084770774726), (-0.20282553496107936, 2.993135780810445), (-0.13521702330738625, 1.99542385387363), (-0.06760851165369312, 0.997711926936815), (0.0, 0.0), (0.0, 0.0)]]
#output6
#final_tour=[[(0.0, 0.0), (0.0, 0.0), (-0.2649090853705842, 0.9642733930209422), (-0.5298181707411684, 1.9285467860418843), (-0.7947272561117527, 2.8928201790628263), (-1.0596363414823369, 3.8570935720837687), (-1.324545426852921, 4.821366965104711), (-1.8438059547337744, 5.559308360341207), (-2.260820970380497, 6.487226619763019), (-2.5257300557510813, 7.451500012783962), (-3.2360609736156176, 7.762488051480443), (-3.802501095061384, 8.395077304818686), (-4.41006036475335, 9.11694978333418), (-4.793835615314822, 10.031247550227972), (-5.058744700685406, 10.995520943248914), (-3.064290057154065, 11.15406580517526), (-2.2982456140350878, 10.511278195488721)], [(-2.2982456140350878, 10.511278195488721), (-3.064290057154065, 11.15406580517526), (-5.058744700685406, 10.995520943248914), (-4.793835615314822, 10.031247550227972), (-4.41006036475335, 9.11694978333418), (-3.802501095061384, 8.395077304818686), (-3.2360609736156176, 7.762488051480443), (-2.5257300557510813, 7.451500012783962), (-2.260820970380497, 6.487226619763019), (-1.8438059547337744, 5.559308360341207), (-1.324545426852921, 4.821366965104711), (-1.0596363414823369, 3.8570935720837687), (-0.7947272561117527, 2.8928201790628263), (-0.5298181707411684, 1.9285467860418843), (-0.2649090853705842, 0.9642733930209422), (0.0, 0.0), (0.0, 0.0)]]
#final_tour=[[(0.0, 0.0), (0.0, 0.0), (-0.06605455157002893, 0.9978160132092903), (-0.13210910314005786, 1.9956320264185805), (-0.1981636547100868, 2.9934480396278706), (-0.2642182062801157, 3.991264052837161), (-0.33027275785014465, 4.989080066046451), (-0.6402813440223815, 9.67205079497626), (0.002506265664159457, 10.438095238095237)], [(0.002506265664159457, 10.438095238095237), (-0.6402813440223815, 9.67205079497626), (-2.63833691355985, 4.939060824543208), (-1.728950711057021, 4.523108021093725), (-0.7448790046668972, 4.2796469066756115), (0.16450719783593137, 4.2726117935477435), (1.07389340033876, 4.28668201980348), (1.9832796028415884, 4.258541567292007), (2.892665805344417, 4.314822472314952), (3.8020520078472453, 4.2022606622690635), (4.412931468962863, 4.715133465716752), (5.038068060955526, 4.6647941942733935), (5.947454263458354, 4.24884139082391), (6.856840465961183, 3.832888587374426), (7.766226668464011, 3.4169357839249423), (8.67561287096684, 3.0009829804754586), (9.584999073469668, 2.585030177025975), (10.40834020986233, 4.6184167097632285), (11.05112781954887, 5.3844611528822055)], [(11.05112781954887, 5.3844611528822055), (10.40834020986233, 4.6184167097632285), (8.995161621673546, 3.8367499323832206), (8.226506059646471, 3.6502873928774315), (7.312449830796863, 3.244699904779939), (6.398393601947255, 2.839112416682447), (5.484337373097647, 2.4335249285849545), (4.570281144248039, 2.027937440487462), (3.6562249153984308, 1.6223499523899698), (2.742168686548823, 1.2167624642924775), (1.8281124576992154, 0.8111749761949849), (0.9140562288496077, 0.40558748809749245), (0.0, 0.0), (0.0, 0.0)]]
###para testes no dinamico
#cenario 1 person: 
#final_tour=[[(0.0, 0.0), (0.0, 0.0), (0.46122657345899426, 0.8872823946947641), (0.9224531469179885, 1.7745647893895282), (1.3836797203769828, 2.6618471840842926), (1.844906293835977, 3.5491295787790564), (2.3061328672949712, 4.43641197347382), (2.7673594407539657, 4.669076407957854), (3.22858601421296, 4.5527441907158375), (3.6898125876719545, 4.610910299336846), (4.1510391611309485, 4.581827245026341), (4.703418163810947, 4.613239607717829), (5.466628992762149, 4.905172285623128), (6.131953319435037, 5.640097214480681), (6.593179892894031, 6.527379609175445), (7.054406466353025, 7.414662003870209), (7.515633039812019, 8.301944398564974), (5.192982456140351, 9.989974937343359), (4.192982456140351, 9.989974937343359)], [(4.192982456140351, 9.989974937343359), (5.192982456140351, 9.989974937343359), (7.515633039812019, 8.301944398564974), (7.054406466353025, 7.414662003870209), (6.593179892894031, 6.527379609175445), (6.131953319435037, 5.640097214480681), (5.466628992762149, 4.905172285623128), (4.703418163810947, 4.613239607717829), (4.1510391611309485, 4.581827245026341), (3.6898125876719545, 4.610910299336846), (3.22858601421296, 4.5527441907158375), (2.7673594407539657, 4.669076407957854), (2.3061328672949712, 4.43641197347382), (1.844906293835977, 3.5491295787790564), (1.3836797203769828, 2.6618471840842926), (0.9224531469179885, 1.7745647893895282), (0.46122657345899426, 0.8872823946947641), (0.0, 0.0), (0.0, 0.0)]]
# cenario 2 people 
#final_tour=[[(0.0, 0.0), (0.0, 0.0), (0.9145266388250867, 0.40452568135940337), (1.8290532776501733, 0.8090513627188067), (2.74357991647526, 1.21357704407821), (3.6581065553003467, 1.6181027254376135), (4.572633194125434, 2.0226284067970166), (5.48715983295052, 2.4271540881564198), (6.401686471775607, 2.831679769515823), (7.316213110600693, 3.236205450875226), (8.23073974942578, 3.640731132234629), (8.984414043574418, 3.8229589841355116), (10.44443043542624, 4.619920469161725), (11.087218045112781, 5.385964912280702)], [(11.087218045112781, 5.385964912280702), (10.44443043542624, 4.619920469161725), (8.984414043574418, 3.8229589841355116), (8.23073974942578, 3.640731132234629), (7.316213110600693, 3.236205450875226), (6.401686471775607, 2.831679769515823), (5.48715983295052, 2.4271540881564198), (4.572633194125434, 2.0226284067970166), (3.6581065553003467, 1.6181027254376135), (2.74357991647526, 1.21357704407821), (1.8290532776501733, 0.8090513627188067), (0.9145266388250867, 0.40452568135940337), (0.0, 0.0), (0.0, 0.0)]]
# cenario 3 output1 
#final_tour=[[(0.0, 0.0), (0.0, 0.0), (0.9138747304856061, 0.4059962770517247), (1.8277494609712122, 0.8119925541034494), (2.7416241914568182, 1.217988831155174), (3.6554989219424243, 1.6239851082068988), (4.56937365242803, 2.0299813852586235), (5.4832483829136365, 2.435977662310348), (6.3971231133992426, 2.841973939362073), (7.310997843884849, 3.2479702164137976), (8.224872574370455, 3.6539664934655223), (8.974603658775866, 3.8213345684099265), (10.427387828909948, 4.632451797482527), (11.07017543859649, 5.398496240601504)], [(11.07017543859649, 5.398496240601504), (10.427387828909948, 4.632451797482527), (9.860635740816674, 2.814053463522712), (9.159427173801552, 3.5270096652241905), (8.223702595947872, 3.522742960242523), (7.455476895278628, 4.13090873477574), (6.727050814690207, 4.820394632879249), (6.025842247675085, 5.533350834580727), (5.324633680659963, 6.246307036282205), (5.6364745926548965, 6.9592632379836825), (5.48055413665743, 7.67221943968516), (5.438602141738484, 8.44790108305405), (5.582533982793824, 8.500111147536815), (5.65519003267713, 8.67076812993434), (5.795081541766637, 8.85765446585401), (5.9437301357839845, 9.143331737917899), (6.105577648968426, 9.586386765447878), (5.172932330827067, 9.974937343358395), (4.172932330827067, 9.974937343358395)], [(4.172932330827067, 9.974937343358395), (5.172932330827067, 9.974937343358395), (7.019825345740742, 9.2158187008755), (6.716156685694232, 8.280034974581167), (6.8621577759568435, 7.389870852293591), (6.5825348442318505, 6.510872070146305), (6.122165596053272, 5.623144547011255), (5.451140324378421, 4.895720674484185), (4.6904446882717625, 4.6118180266956585), (4.143323233607207, 4.582244552939109), (3.6829539854286284, 4.610965940391881), (3.22258473725005, 4.553523165486337), (2.762215489071471, 4.668408715297425), (2.3018462408928926, 4.4386376156752485), (1.8414769927143142, 3.5509100925401986), (1.3811077445357356, 2.6631825694051487), (0.9207384963571571, 1.7754550462700993), (0.46036924817857855, 0.8877275231350497), (0.0, 0.0), (0.0, 0.0)]]
#plot_tour(final_tour, static_people, Gstatico, obstacles)
#Dinâmico
WORLD_SIZE = 20
NUM_PEOPLE = 3
NUM_GOALS = 2
CHANGE_INTERVAL = 2
MIN_DISTANCE_BETWEEN_GOALS = 10

#people = generate_people(NUM_PEOPLE, WORLD_SIZE, obstacles)
people = []
pe=Person(-8, -5, np.deg2rad(180), id_node=0)
people.append(pe)
# pe1=Person(15, -10, np.deg2rad(180), id_node=1)
# people.append(pe1)
pe2=Person(-10, 5, np.deg2rad(80), id_node=1)
people.append(pe2)
pe3= Person(-15, -4, np.deg2rad(0), id_node=2)
people.append(pe3)
#goals = generate_goals(NUM_GOALS, WORLD_SIZE, obstacles, people, MIN_DISTANCE_BETWEEN_GOALS)
#print(f'goals:{goals}')
#goals =[np.array([-5.0, 3.0]), np.array([-8,-15])]
goals = [np.array([17, -9]), np.array([8,15])]

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
# Adicionando a legenda
#ax.legend(loc='upper right', bbox_to_anchor=(1, 1), borderaxespad=0.)
#plt.tight_layout()

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


plt.title('Initial Positions')
plt.savefig('initial_figure.png')
plt.show()

def update(frame, people, goals, obstacles, ax, static_people, start_time, WORLD_SIZE, last_change_time, t, CHANGE_INTERVAL, robot):
    current_time = time.time()
    robot.total_time = current_time - start_time

    # Atualiza o robô e as pessoas
    robot_path.append((robot.x, robot.y))    
    for i, person in enumerate(people):
        people_paths[i].append((person.x, person.y))
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
        
    robot_path_x, robot_path_y = zip(*robot_path)
    ax.plot(robot_path_x, robot_path_y, 'b--', label="Robot's path")
    
    for i, person_path in enumerate(people_paths):
        if person_path:  # Verifica se há pelo menos um ponto no caminho
            person_path_x, person_path_y = zip(*person_path)
            ax.plot(person_path_x, person_path_y, 'y--', label="Individual's path" if i == 0 else "")

    #ax.legend(loc='upper right')  # Adiciona a legenda no canto superior direito

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

    # all_x = [robot.x] + [person.x for person in people] + [goal[0] for goal in goals]
    # all_y = [robot.y] + [person.y for person in people] + [goal[1] for goal in goals]

    # buffer = 5
    # min_x, max_x = min(all_x) - buffer, max(all_x) + buffer
    # min_y, max_y = min(all_y) - buffer, max(all_y) + buffer

    #ax.set_xlim(min_x, max_x)
    #ax.set_ylim(min_y, max_y)
    ax.set_xlim(-WORLD_SIZE-5, WORLD_SIZE + 5)
    ax.set_ylim(-WORLD_SIZE-5, WORLD_SIZE + 5)
    ax.set_aspect('equal', adjustable='box')
    t += 1
    
    # Configura a legenda após os plots
    ax.legend(loc='upper right', bbox_to_anchor=(1, 1), borderaxespad=0.)


    plt.tight_layout()
    plt.show(ax)

    return ax


fig, ax = plt.subplots()
last_change_time = time.time()
t = 0
start_time = time.time()
robot_path = []
people_paths = [[] for _ in people]

ani = FuncAnimation(fig, update, frames=frame_generator(robot, people, goals, obstacles), 
                     fargs=(people, goals, obstacles, ax, static_people, start_time, WORLD_SIZE, last_change_time, t, CHANGE_INTERVAL, robot), 
                     interval=100, blit=False, save_count=700, cache_frame_data=False)

print("Animação Criada com sucesso")
ani.save('animation23.gif', writer=PillowWriter(fps=10))
print("Animação salva como 'animation23.gif'")
plt.close(fig)

# Chama a função para imprimir o resumo ao final da simulação
robot.print_summary()