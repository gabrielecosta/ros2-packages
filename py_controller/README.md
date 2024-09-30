# Modulo di Py-Controller

Authors: Gabriele Nicolò Costa, Nickolo Maria Spingola

## Introduzione

Il seguente pacchetto ros permette di controllare un joystick in Python, al fine di inviare comandi ad una nucleo tramite seriale. Si compone di tre nodi:
- writing: lettura comandi del joystick, scrittura sul topic
- nucle_adapter: lettura dei comandi dal topic, mappatura dei comandi ricevuti e pubblica su un topic i comandi di velocità e rotazione tramite gestione di due topic
- nucleo_node: lettura dei comandi di velocità dal topic e scrittura sulla nucleo tramite seriale (usa la libreria nucleo interface)

### Writing
Il seguente nodo permette di leggere i comandi in input dal joystick e pubblica tramite file json una stringa sul topic /controller_commands

### Nucleo Adapter
Legge i comandi dalla stringa del topic /controller_commands, dopodiché permette di mappare i comandi ricevuti in comandi di velocità e rotazione dei servo. Pubblica sui topic: /cmd_vel e /servo_command.

### Nucleo Node
Legge i comandi di velocità dai due topic di /cmd_vel e /servo_command e, tramite la nucleo interface, permette di tradurre in seriale i comandi da inviare alla nucleo pubblicando sul topic /wheel_angles.

## Installazione
Dopo aver clonato la cartella all'interno del percorso 'workspace/src', dobbiamo effettuare una build del package:
```
colcon build --packages-select py_controller
```
Per installare le dipendenze appena compilate:
```
source install/setup.bash
```
Quindi andare ad eseguire i nodi:
- writing node: `ros2 run py_controller writingcommands` 
- nucleo adapter: `ros2 run py_controller nucleoadapter` 
- nucleo node (interface): `ros2 run py_controller nucleointerface`
