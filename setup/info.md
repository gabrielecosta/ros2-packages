# Modulo Setup ROS2
Autore: Gabriele Nicolò Costa

## Introduzione

Il seguente pacchetto ROS permette di effettuare la calibrazione della camera, permettendo così di definire la matrice di calibrazione per mezzo di un servizio. Si compone di 3 moduli:
- **setup_client**
- **setup_server**
- **setup_test**

### Setup Client
Il nodo di setup client permette di leggere dal feed **image_feed_topic** e di catturare almeno 10 immagini per poter effettuare la calibrazione. Non appena salva anche la decima immagine, tramite trigger di richiesta, contatta il server avvisandolo di avviare il processo di calibrazione sulle immagini già salvate. Bisogna usare il seguente pattern di calibrazione:
![Pattern di calibrazione](calibration_pattern.jpg)

### Setup Server
Il nodo di setup server invece si occupa di effettuare la calibrazione, determinare la matrice e i coefficienti di distorsione e infine salvarli.

### Setup Test
Il nodo di setup test carica i parametri di calibrazione ed effettua un test per verificare l'effettiva rimozione della distorsione e applicazione dei parametri

## Installazione
Dopo aver clontato la cartella all'interno del percorso 'workspace/src', dobbiamo effettuare una build del package:
```
colcon build --packages-select setup
```
Per installare le dipendenze appena compilate:
```
source install/setup.bash
```
Quindi andare ad eseguire i nodi:
- Setup client: `ros2 run setup stpclient`
- Setup server: `ros2 run setup stpserver`
- Setup test: `ros2 run setup stptest`

## Raccomandazioni
Per poter utilizzare correttamente il seguente package è necessario installare e verificare correttamente il package **py_camera_stream**, affinché sia presente uno streamer e un receiver per popolare il image feed topic.