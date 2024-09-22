# Modulo Camera Streaming ROS2
Autore: Gabriele Nicolò Costa

## Introduzione

Il seguente pacchetto ros permette lo streaming video su una rete, ed è composto tra 4 moduli: 
- streamer
- receiver
- udp_streaming
- example subscriber

### Streamer
Lo streamer verrà eseguito materialmente sul computer definito come "server". Ha il compito di trasmettere tramite udp uno streaming video. Si servirà dunque del modulo udp_streaming che presenta una classe adatta per poter abilitare la socket alla trasmissione udp. Pubblica sul topic **"camera_streaming"**

### Receiver
Il receiver si trova sul computer definito come "client", riceverà lo streaming video tramite udp e pubblicherà su un topic chiamato **"image_feed_topic"**. 

### UDP Streaming
Classe di utilità che contiene gli adattatori udp per streaming e recezione tramite udp.

### Example subscriber
Esempio di nodo che servirà come subscriber per poter stremmare quanto ricevuto dal topic **image_feed_topic**  

## Installazione
Dopo aver clontato la cartella all'interno del percorso 'workspace/src', dobbiamo effettuare una build del package:
```
colcon build --packages-select py_camera_stream
```
Per installare le dipendenze appena compilate:
```
source install/setup.bash
```
Quindi andare ad eseguire i nodi:
- Streamer: `ros2 run py_camera_stream streamer`
- Receiver: `ros2 run py_camera_stream receiver`
- Example sub: `ros2 run py_camera_stream examplesub`
