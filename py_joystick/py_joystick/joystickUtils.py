import pygame
import time
import json


class joyStickUtils():
    def __init__(self):
        # Inizializza pygame
        pygame.init()
        # Inizializza il modulo joystick
        pygame.joystick.init()
        self.controller = None
        if self.available_controllers():
            # Inizializza il controller Xbox One S
            self.controller = pygame.joystick.Joystick(0)
            self.controller.init()
            print(f"Controller collegato: {self.controller.get_name()}")


    def available_controllers(self):
        # Controlla se ci sono controller collegati
        if pygame.joystick.get_count() == 0:
            print("Nessun controller Xbox One S collegato.")
            return False
        return True
    
    def controllerReading(self):
        if self.controller:
            # Gestisci gli eventi di pygame
            pygame.event.pump()

            # Leggi gli stick analogici (assi)
            left_stick_x = self.controller.get_axis(0)  # Stick sinistro, asse X
            left_stick_y = self.controller.get_axis(1)  # Stick sinistro, asse Y
            right_stick_x = self.controller.get_axis(3)  # Stick destro, asse X
            right_stick_y = self.controller.get_axis(4)  # Stick destro, asse Y
            # gli stick left e right variano tra -1 e 1 in una circoferenza, quindi X e Y

            # Trigger (grilletti)
            left_trigger = self.controller.get_axis(2)  # Trigger sinistro
            right_trigger = self.controller.get_axis(5)  # Trigger destro
            # i trigger sono quelli posteriori e assumono valori tra -1 e 1
            # -1 stato di riposo, 1 stato di attivazione

            # Leggi i pulsanti
            a_button = self.controller.get_button(0)  # Pulsante A
            b_button = self.controller.get_button(1)  # Pulsante B
            x_button = self.controller.get_button(2)  # Pulsante X
            y_button = self.controller.get_button(3)  # Pulsante Y

            lb_button = self.controller.get_button(4)  # Pulsante LB (dorsale sinistro)
            rb_button = self.controller.get_button(5)  # Pulsante RB (dorsale destro)
            back_button = self.controller.get_button(6)  # Pulsante BACK
            start_button = self.controller.get_button(7)  # Pulsante START
            # i pulsanti assumono solo due valori, 0 e 1
            # back e start sono i pulsanti che si trovano davanti sopra gli stick a sinistra dei ABXY

            # Crea un dizionario con i valori
            controller_data = {
                "stick_sinistro": {
                    "X": round(left_stick_x, 3),
                    "Y": round(left_stick_y, 3)
                },
                "stick_destro": {
                    "X": round(right_stick_x, 3),
                    "Y": round(right_stick_y, 3)
                },
                "trigger": {
                    "sinistro": round(left_trigger, 3),
                    "destro": round(right_trigger, 3)
                },
                "pulsanti": {
                    "A": a_button,
                    "B": b_button,
                    "X": x_button,
                    "Y": y_button,
                    "LB": lb_button,
                    "RB": rb_button,
                    "BACK": back_button,
                    "START": start_button
                }
            }

            return controller_data
        return None
    
    def convertJson(self, data):
        '''
        la seguente funzione trasforma il dizionario in una stringa in json
        '''
        # Trasforma il dizionario in una stringa JSON
        controller_data_json = json.dumps(data)
        return controller_data_json
    

    def convertString(self, msg):
        '''
        la seguente funzione trasforma un messaggio stringa in un dizionario
        '''
        # Converting the string received into a dictionary
        controller_data = json.loads(msg.data) 
        # Accessing values from the dictionary
        # stick_sinistro_x = controller_data['stick_sinistro']['X']
        # stick_sinistro_y = controller_data['stick_sinistro']['Y']
        
        # stick_destro_x = controller_data['stick_destro']['X']
        # stick_destro_y = controller_data['stick_destro']['Y']
        
        # trigger_sinistro = controller_data['trigger']['sinistro']
        # trigger_destro = controller_data['trigger']['destro']
        
        # pulsante_a = controller_data['pulsanti']['A']
        # pulsante_b = controller_data['pulsanti']['B']
        # pulsante_x = controller_data['pulsanti']['X']
        # pulsante_y = controller_data['pulsanti']['Y']
        # pulsante_lb = controller_data['pulsanti']['LB']
        # pulsante_rb = controller_data['pulsanti']['RB']
        # pulsante_back = controller_data['pulsanti']['BACK']
        # pulsante_start = controller_data['pulsanti']['START']
        return controller_data