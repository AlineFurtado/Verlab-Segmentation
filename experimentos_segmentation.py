# -*- coding: utf-8 -*-
"""
Created on Wed Nov 29 14:37:38 2023

@author: User-Aline
"""
#
import sys
import cv2
import numpy as np
import pyopenpose as op

sys.path.append("src/")

from scipy import io
from src.person import Person
from src.overall_density6 import OverallDensity

# Configuração do OpenPose
params = {
    "model_folder": "/caminho/para/a/pasta/models/",  # Diretório com os modelos do OpenPose
    "face": False,  # Não queremos a detecção de rostos neste exemplo
    "hand": False   # Não queremos a detecção de mãos neste exemplo
}

# Inicialização do OpenPose
openpose = op.OpenPose(params)

# Função para obter a pose de uma pessoa na imagem
def get_pose(image):
    # OpenPose processa a imagem para obter as poses
    keypoints, _ = openpose.forward(image, True)
    
    # keypoints contém as coordenadas dos keypoints identificados
    # Você pode processar keypoints para obter informações sobre a pose (por exemplo, calcular a orientação)
    
    # Exemplo simplificado: supondo que a orientação é baseada na diferença entre coordenadas dos ombros
    if len(keypoints) > 0:  # Verifica se há alguma detecção de pose
        # Supondo que os keypoints dos ombros são 2 e 5 (dependendo da ordem retornada por OpenPose)
        shoulder_left = keypoints[0][2]
        shoulder_right = keypoints[0][5]
        
        # Calcula a orientação baseada na diferença entre as coordenadas dos ombros
        theta = abs(shoulder_right[0] - shoulder_left[0])
        
        # Retorna a pose com informações de orientação (theta)
        return {'orientation': theta}
    
    # Se não houver detecção de pose, retorna None ou informações padrão
    return None


# Função que processa as regiões sociais da cena com base nas informações das pessoas
def process_regions(people):
    
    G = OverallDensity(person=people, zone='Social', map_resolution=400, window_size=1)
    G.make_graph()
    G.boundary_estimate()
    G.draw_overall(drawDensity=False, drawCluster=True, drawGraph=False, drawSegment=True, people=people)

# Carregar o modelo YOLOv3 para detecção de pessoas
net = cv2.dnn.readNet("yolov3.weights", "yolov3.cfg")
with open("coco.names", "r") as f:
    classes = [line.strip() for line in f.readlines()]

# Definir caminho para o vídeo
video_path = 'dataset/Salsa/imagens/salsa_ps_cam1.avi'  # Substitua pelo caminho do seu vídeo

# Capturar o vídeo
cap = cv2.VideoCapture(video_path)

while cap.isOpened():
    ret, frame = cap.read()

    if not ret:
        break

    height, width, _ = frame.shape

    # Criar um blob da imagem para a entrada do modelo YOLO
    blob = cv2.dnn.blobFromImage(frame, 1/255.0, (416, 416), swapRB=True, crop=False)

    net.setInput(blob)
    output_layers_names = net.getUnconnectedOutLayersNames()
    layer_outputs = net.forward(output_layers_names)

    boxes = []
    confidences = []
    class_ids = []

    for output in layer_outputs:
        for detection in output:
            scores = detection[5:]
            class_id = np.argmax(scores)
            confidence = scores[class_id]

            if confidence > 0.5 and class_id == 0:  # Class ID 0 é pessoa
                center_x = int(detection[0] * width)
                center_y = int(detection[1] * height)
                w = int(detection[2] * width)
                h = int(detection[3] * height)

                x = int(center_x - w / 2)
                y = int(center_y - h / 2)

                boxes.append([x, y, w, h])
                confidences.append(float(confidence))
                class_ids.append(class_id)

    # Aplicar non-max suppression para eliminar detecções redundantes
    indexes = cv2.dnn.NMSBoxes(boxes, confidences, score_threshold=0.5, nms_threshold=0.4)

#    if len(indexes) > 0:
#        for i in indexes.flatten():
#            x, y, w, h = boxes[i]
#            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)

    # Exibir o frame com as detecções
#    cv2.imshow('Detecção de pessoas', frame)
#    if cv2.waitKey(1) & 0xFF == ord('q'):
#        break

#cap.release()
#cv2.destroyAllWindows()

    if len(indexes) > 0:
        people = []  # Lista para armazenar informações de pessoas (x, y, theta, index)

        for i in indexes.flatten():
            x, y, w, h = boxes[i]
            
            # Obter a orientação (theta) usando a função de estimativa de pose (get_pose)
            frame_with_person = frame[y:y+h, x:x+w]
            poses = get_pose(frame_with_person)  # Obter a pose da pessoa na área da BBox
        
            # Exemplo simplificado: se houver uma pose estimada, usaremos a orientação do corpo (ex: torso)
            if poses:
                theta = poses[0]['orientation']  # Supondo que a orientação está disponível na pose
            else:
                theta = 0  # Valor padrão se a orientação não for estimada

            # Adicionar informações de pessoas (x, y, theta, index) à lista
            people.append({'x': x, 'y': y, 'theta': theta, 'index': i})


            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)

        # Processar as regiões sociais com base nas informações das pessoas
        process_regions(people)

    # Exibir o frame com as detecções
    cv2.imshow('Detecção de pessoas', frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
