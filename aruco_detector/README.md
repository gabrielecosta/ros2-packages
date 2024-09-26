# Modulo Aruco Detector ROS2
Autore: Gabriele Nicolò Costa

## Introduzione

Il seguente pacchetto ros permette di rilevare gli aruco marker, stamparli a schermo e
pubblicare su un topic gli id dei marker trovati. Si compone di due moduli: 
- aruco detector
- aruco displayer

### Aruco Detector
Questo nodo legge dal topic **image_feed_topic** e pubblica sul topic **aruco_detection**. 
Effettua la rilevazione degli aruco marker e pubblica la quadrupla (markerID, cX, cY, angle) che
potrà servire in seguito per poter effettuare Omografia

### Aruco Displayer
Attualmente serve solo come nodo di Debug per poter leggere dal topic. In futuro sarà base del nodo
per effettuare omografia o comunque poter trovare la matrice di omografia H da applicare ai frame

## Installazione
Dopo aver clonato la cartella all'interno del percorso 'workspace/src', dobbiamo effettuare una build del package:
```
colcon build --packages-select aruco_detector
```
Per installare le dipendenze appena compilate:
```
source install/setup.bash
```
Quindi andare ad eseguire i nodi:
- aruco detector: `ros2 run aruco_detector arucodetector`
- aruco displayer: `ros2 run aruco_detector arucodisplayer`

## Dipendenze
Il seguente modulo necessita di alcune librerie da installre, come opencv-contrib.
Inoltre dipende dai seguenti pkg ros (disponibili nel repository):
- py_camera_stream per leggere dal feed
- setup per poter impostare la matrice di calibrazione