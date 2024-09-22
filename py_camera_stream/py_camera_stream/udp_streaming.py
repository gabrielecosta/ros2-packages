import cv2
import socket
import numpy as np

# definisco lo streamer udp
class udpStreamer():
    def __init__(self, udp_ip, udp_port):
        self.udp_ip = udp_ip
        self.udp_port = udp_port
        self.frame_counter = 0
        # Configura il socket per inviare i dati
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    def send(self, frame):
        '''
        Questa funzione prende in input il frame (catturato da una camera opportuna)
        e lo trasmette tramite socket udp
        '''
        # Comprimi il frame per risparmiare larghezza di banda
        encoded, buffer = cv2.imencode('.jpg', frame, [int(cv2.IMWRITE_JPEG_QUALITY), 50])
        # Converti il buffer in byte e invialo tramite UDP
        self.sock.sendto(buffer.tobytes(), (self.udp_ip, self.udp_port)) 
    
    def closeConnection(self):
        # routine di chiusura della connessione
        self.sock.close()

# definisco il receiver udp
class udpReceiver():
    def __init__(self, udp_ip, udp_port):
        self.udp_ip = udp_ip
        self.udp_port = udp_port
        # creo la socket di connessione
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((self.udp_ip, self.udp_port))
        self.sock.settimeout(5.0)  # Set timeout to 5 seconds
        # Parametri di buffer
        self.buffer_size = 65536  # Dimensione massima del pacchetto UDP

    def receive(self):
        # Riceve i dati dalla rete
        data, addr = self.sock.recvfrom(self.buffer_size)
        # Converte i byte ricevuti in un array numpy
        frame_data = np.frombuffer(data, dtype=np.uint8)
        # Decodifica l'immagine
        frame = cv2.imdecode(frame_data, cv2.IMREAD_COLOR)
        # ritorna il frame da mostrare
        return frame

    def closeConnection(self):
        # routine di chiusura della connessione
        self.sock.close()

