import numpy as np
import cv2
import cv2.aruco as aruco
import math

class arucoUtils:
    def __init__(self):
        # Definisco il dizionario ArUco
        self.aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_250)
        # Stabilisco i parametri per la rilevazione 
        self.parameters = aruco.DetectorParameters()
    
    def getAruco_dict(self):
        return self.aruco_dict
    
    def getParameters(self):
        return self.parameters
    
    def get_id_and_position(self, corners, ids, rejected, image):
        '''
        Questa funzione prende in input i corners e gli indici degli aruco
        trovati da opencv, dunque li salva in una lista che ritorna.
        Ogni alemeno della lista è una tripla di elementi del tipo (markerId, cX, cY)
        '''
        list_id_pos = []
        if len(corners) > 0:
            ids = ids.flatten()
            for (markerCorner, markerID) in zip(corners, ids):
                corners = markerCorner.reshape((4, 2))
                (topLeft, topRight, bottomRight, bottomLeft) = corners
                topRight = (int(topRight[0]), int(topRight[1]))
                bottomRight = (int(bottomRight[0]), int(bottomRight[1]))
                bottomLeft = (int(bottomLeft[0]), int(bottomLeft[1]))
                topLeft = (int(topLeft[0]), int(topLeft[1]))
                cX = int((topLeft[0] + bottomRight[0]) / 2.0)
                cY = int((topLeft[1] + bottomRight[1]) / 2.0)
                list_id_pos.append((markerID, cX, cY))
        return list_id_pos

    def calculate_orientation(self, x, y, bottomRight, topRight):
        '''
        Questa funzione calcola l'orientamento di ogni aruco marker tenendo conto di dove si trovano (x,y)
        e di quali siano i punti del corner trovato (capire di quanto sono orientati rispetto al loro lato)
        Restituisce quindi per ogni marker un vettore
        '''
        delta_x = topRight[0] - bottomRight[0]
        delta_y = topRight[1] - bottomRight[1]
        arrow = (x+delta_x, y+delta_y)
        return arrow, math.atan2(delta_y, delta_x)
    
    def aruco_vector_state(self, corners, ids):
        '''
        questa funzione prende in input corners e ids trovati,
        calcola il vettore di rotazione e le coordinate del marker.
        Ritorna la lista di marker trovati
        '''
        vector_markers = []
        if len(corners) > 0:
            ids = ids.flatten()
            # per ogni corner e id rilevato
            for (markerCorner, markerID) in zip(corners, ids):
                corners = markerCorner.reshape((4, 2))
                # estraggo gli angoli
                (topLeft, topRight, bottomRight, bottomLeft) = corners
                # definisco le tuple andando a fare casting esplicito delle coordinate
                topRight = (int(topRight[0]), int(topRight[1]))
                bottomRight = (int(bottomRight[0]), int(bottomRight[1]))
                bottomLeft = (int(bottomLeft[0]), int(bottomLeft[1]))
                topLeft = (int(topLeft[0]), int(topLeft[1]))
                # determino le coordinate del punto
                cX = int((topLeft[0] + bottomRight[0]) / 2.0)
                cY = int((topLeft[1] + bottomRight[1]) / 2.0)
                # determino il vettore di rotazione
                arrow, angle = self.calculate_orientation(cX,cY,bottomRight,topRight)
                # aggiungo alla lista dei marker trovati
                vector_markers.append((markerID, cX, cY, angle))
        return vector_markers
    
    def aruco_display(self, corners, ids, rejected, image):
        '''
        questa funzione prende in input corners e ids trovati,
        calcola il vettore di rotazione e le coordinate del marker,
        quindi disegna sul frame e ritorna una lista di vettori di marker
        '''
        if len(corners) > 0:
            ids = ids.flatten()
            # per ogni corner e id rilevato
            for (markerCorner, markerID) in zip(corners, ids):
                corners = markerCorner.reshape((4, 2))
                # estraggo gli angoli
                (topLeft, topRight, bottomRight, bottomLeft) = corners
                # definisco le tuple andando a fare casting esplicito delle coordinate
                topRight = (int(topRight[0]), int(topRight[1]))
                bottomRight = (int(bottomRight[0]), int(bottomRight[1]))
                bottomLeft = (int(bottomLeft[0]), int(bottomLeft[1]))
                topLeft = (int(topLeft[0]), int(topLeft[1]))
                # traccio le rette nel frame
                cv2.line(image, topLeft, topRight, (0, 255, 0), 2)
                cv2.line(image, topRight, bottomRight, (0, 255, 0), 2)
                cv2.line(image, bottomRight, bottomLeft, (0, 255, 0), 2)
                cv2.line(image, bottomLeft, topLeft, (0, 255, 0), 2)
                # determino le coordinate del punto
                cX = int((topLeft[0] + bottomRight[0]) / 2.0)
                cY = int((topLeft[1] + bottomRight[1]) / 2.0)
                # determino il vettore di rotazione
                arrow, angle = self.calculate_orientation(cX,cY,bottomRight,topRight)
                # traccio l'orientamento dell'aruco dal vettore di rotazione
                cv2.arrowedLine(image, (cX, cY), arrow, (255,0,0), 2)
                cv2.circle(image, (cX, cY), 4, (0, 0, 255), -1)
                # inserisco la descrizione con l'ID del marker
                cv2.putText(image, str(markerID),(topLeft[0], topLeft[1] - 10), cv2.FONT_HERSHEY_SIMPLEX,
                    0.5, (0, 255, 0), 2)
        return image


    # def start_calibration(self):
    #     cap = cv2.VideoCapture(1)
    #     cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
    #     cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)
    #     last_frame = None
    #     while cap.isOpened():
    #         ret, frame = cap.read()
    #         corners, ids, rejected = cv2.aruco.detectMarkers(frame, self.aruco_dict, parameters=self.parameters)
    #         detected_markers = self.aruco_display(corners, ids, rejected, frame)
    #         # resize
    #         cv2.imshow("Immagine originale", cv2.resize(detected_markers, (1920//2, 1080//2)))
    #         last_frame = frame
    #         key = cv2.waitKey(1) & 0xFF
    #         if key == ord("q"):
    #             break
    #     # trovo la matrice di calibrazione
    #     # cv2.imshow('Immagine prima della trasformazione', last_frame)
    #     cv2.imshow('Immagine prima della trasformazione', cv2.resize(last_frame, (1920//2, 1080//2)))
    #     cv2.waitKey(0)
    #     # list_id_pos = self.findArucos(frame)
    #     H = self.matrix_homography(last_frame)
    #     h, w, _ = last_frame.shape
    #     # applico la trasformazione
    #     img_proj = cv2.warpPerspective(last_frame, H, (w, h))
    #     self.H = H
    #     # cv2.imshow('Immagine proiettata', img_proj)
    #     cv2.imshow('Immagine proiettata', cv2.resize(img_proj, (1920//2, 1080//2)))
    #     key = cv2.waitKey(1) & 0xFF
    #     cv2.waitKey(0)
    #     cv2.destroyAllWindows()
    #     cap.release()
    #     np.save('matrice_H.npy',H)
    #     print('Ho salvato la matrice H di calibrazione, adesso procedo con i colori')

