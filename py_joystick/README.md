# Modulo di Controller Joystick

Authors: Gabriele Nicolò Costa

## Introduzione

Il seguente pacchetto ros permette di controllare un joystick in Python. Si compone di due nodi:
- reading: lettura del topic
- writing: scrittura sul topic

## Sviluppi Futuri

In futuro questo nodo servirà per controllare poi un robot da remoto.


## Installazione
Dopo aver clonato la cartella all'interno del percorso 'workspace/src', dobbiamo effettuare una build del package:
```
colcon build --packages-select py_joytstick
```
Per installare le dipendenze appena compilate:
```
source install/setup.bash
```
Quindi andare ad eseguire i nodi:
- reading node: `ros2 run py_joystick reading`
- writing node: `ros2 run py_joystick writing` 
