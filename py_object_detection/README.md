# Modulo di Object Detection

Authors: Gabriele Nicolò Costa, Dario Lombardo

## Introduzione

Il seguente pacchetto ros permette di effettuare object detection, per mezzo
della rete YOLOv10, preaddestrata. Si compone di un unico nodo che serve per leggere
dal topic della live camera e fare live inference. 

## Sviluppi Futuri

In futuro questo nodo servirà per pubblicare su un topic gli oggetti rilevati ed eventualmente fornire
dei feedback. Al momento è un modulo di test e di prova dell'architettura su ros2


## Installazione
Dopo aver clonato la cartella all'interno del percorso 'workspace/src', dobbiamo effettuare una build del package:
```
colcon build --packages-select py_object_detection
```
Per installare le dipendenze appena compilate:
```
source install/setup.bash
```
Quindi andare ad eseguire i nodi:
- detector node: `ros2 run py_object_detection detectornode`

## Dipendenze
Il seguente modulo necessita di alcune librerie da installre, come opencv-contrib e YOLO.
Inoltre dipende dai seguenti pkg ros (disponibili nel repository):
- py_camera_stream per leggere dal feed