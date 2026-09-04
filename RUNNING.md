# Come lanciare la missione

Procedura operativa per far girare la challenge del dado, prima in simulazione e poi sul robot reale. I comandi vanno eseguiti nell'ordine indicato, ciascuno in un terminale separato dentro il container, se non specificato diversamente.

La logica è la stessa nei due casi: cambia solo chi fornisce l'identificazione del dado (il servizio del simulatore oppure `dice_detector`) e se il robot è `fake` o vero. L'albero di comportamento non cambia.

## 0. Una volta sola, sulla macchina

Dall'host, nella cartella del repository:

```bash
bash setup.sh      # regole udev per la telecamera Luxonis e il gruppo drims2
```

Dopo questo script serve un riavvio del sistema.

## 1. Avviare il container

Dall'host:

```bash
bash start.sh
```

Usa `bash`, non `sh`: lo script contiene costrutti bash e con `dash` prende il ramo sbagliato senza ricreare il container. Lo script scarica l'immagine, rimuove il container precedente, ne crea uno nuovo e apre una shell dentro.

Per aprire altri terminali sullo stesso container, sempre dall'host:

```bash
bash connect.sh
```

Il primo controllo da fare, dentro il container:

```bash
echo $ROS_DOMAIN_ID     # deve stampare 5
```

Se è vuoto stai condividendo il dominio DDS di default con tutti gli altri gruppi in aula, e riceverai le loro TF dentro le tue. Il dominio viene impostato da `start.sh` in `~/.bashrc`, quindi manca solo se stai riusando un container creato prima di quella modifica: in quel caso ricrea il container.

## 2. Simulazione

### 2.1 Robot e scena

Terminale 1, dentro il container:

```bash
ros2 launch drims_description ur5e_1_start.launch.py fake:=true
```

Il numero nel nome del launch è la **cella**, e deve corrispondere a quella assegnata al gruppo. Con `fake:=true` parte anche RViz.

Aspetta che lo stack sia pronto prima di procedere. Un controllo affidabile, da un altro terminale:

```bash
ros2 action list | grep gripper_cmd
```

### 2.2 Far comparire il dado

Terminale 2:

```bash
ros2 launch drims_dice_simulator spawn_dice.launch.py \
  face_up:=5 selected_cell:=1 yaw:=0.7 position:="[-0.10, 0.675, 0.0]"
```

`selected_cell` deve essere la stessa cella del robot, altrimenti simuli il braccio di una cella e l'area di gioco di un'altra. I limiti dell'area li decide lo spawner, per cella, in `dice_spawner_parameters.yaml`: per la cella 1 sono `x ∈ [-0.35, 0.15]`, `y ∈ [0.50, 0.85]` rispetto a `base_link`. Fuori da lì lo spawner muore con `Specified position ... is outside the bounds` e la missione fallirà per un motivo che non c'entra nulla con il codice.

`yaw` è la rotazione iniziale del dado attorno alla verticale, in radianti. Vale la pena variarlo fra una prova e l'altra: è la condizione che il lancio reale dell'organizzatore riprodurrà.

**Il primo spawn dopo il lancio dello stack fallisce spesso.** Verifica sempre:

```bash
ros2 service call /dice_identification easy_motion_msgs/srv/DiceIdentification "{}"
```

Se risponde `success=False` o `face_number=0`, chiudi lo spawner e rilancialo:

```bash
pkill -9 -f lib/drims_dice_simulator
```

### 2.3 Lanciare la missione

Modifica i parametri in `drims_ws/src/drims_homework/config/dice_mission_params.yaml`, poi:

```bash
cd ~/drims_ws && colcon build --packages-select drims_homework
ros2 launch drims_homework dice_mission_from_config.launch.py
```

**L'ordine conta.** Il launch riscrive i parametri nella copia *installata* dell'albero, mentre `colcon build` la rigenera dal sorgente. Se costruisci dopo aver lanciato, perdi i valori applicati. Quindi sempre: modifica il YAML, costruisci, lancia.

Se vuoi cambiare un solo valore per una prova senza toccare il file, usa la variante con argomenti:

```bash
ros2 launch drims_homework dice_mission.launch.py target_face:=3
```

Accetta gli stessi nomi dei parametri elencati sotto.

## 3. Robot reale

Le differenze rispetto alla simulazione sono tre: `fake:=false`, l'identificazione arriva dalla telecamera invece che dal simulatore, e il dado lo lancia una persona.

### 3.1 Robot

Terminale 1:

```bash
ros2 launch drims_description ur5e_1_start.launch.py fake:=false
```

L'indirizzo IP del robot è già dentro il launch della cella: `192.168.254.101` per la cella 1, `.102` per la 2, e così via. Con `fake:=false` viene incluso anche il launch di calibrazione della telecamera, che è ciò che collega il frame ottico al robot: senza, la posizione riportata dalla visione non è raggiungibile perché non è agganciata all'albero delle TF.

### 3.2 Visione

Terminale 2:

```bash
ros2 run dice_detector dice_detector
```

**Non lanciare lo spawner del simulatore insieme al detector.** Espongono lo stesso servizio `/dice_identification` e ne può funzionare uno solo.

Il detector ascolta `/oak/rgb/image_raw/compressed` e `/oak/rgb/camera_info`, pubblica la TF `dice_tf` e risponde al servizio con numero della faccia, posizione e orientamento.

Prima di muovere il braccio, con il robot fermo, verifica che risponda e che il dado risulti dove lo vedi davvero:

```bash
ros2 service call /dice_identification easy_motion_msgs/srv/DiceIdentification "{}"
ros2 run tf2_ros tf2_echo base_link dice_tf
```

Il secondo comando è il controllo che conta: le coordinate devono cadere dentro l'area di gioco e a un'altezza plausibile. Se `dice_tf` non esiste, manca la catena di TF fra la telecamera e il robot.

### 3.3 Missione

Identica alla simulazione:

```bash
cd ~/drims_ws && colcon build --packages-select drims_homework
ros2 launch drims_homework dice_mission_from_config.launch.py
```

## 4. I parametri della missione

Tutti in `drims_ws/src/drims_homework/config/dice_mission_params.yaml`, tutti disponibili anche come argomenti di `dice_mission.launch.py`.

| parametro | significato |
|---|---|
| `target_face` | faccia da lasciare rivolta verso l'alto |
| `place_x`, `place_y` | dove appoggiare il dado, in `camera_frame_floor`. Il centro della piastra è `(0.365, 0.265)` |
| `place_z` | quota di rilascio, in `base_link`. `0.012` è dove sta il frame riportato quando il dado è appoggiato |
| `lift_height` | sollevamento relativo prima di spostare il dado |
| `grasp_offset` | posizione di presa relativa a `dice_tf`, `"x;y;z"` in metri |
| `grasp_orientation` | orientamento di presa relativo a `dice_tf`, `"x;y;z;w"`. Quaternione nullo significa "calcolalo" |
| `grip_close` | apertura della pinza in chiusura. Non deve essere 0 |
| `grip_open` | apertura in rilascio |
| `grip_effort` | sforzo della pinza |
| `phase1_tilt` | inclinazione della prima presa. `0` è verticale |
| `tilt_deg` | inclinazione delle prese di riorientamento. `auto` la deriva dalla geometria |

Due avvertenze non deducibili dai nomi.

`grasp_orientation` agisce su **entrambe** le prese. L'orientamento calcolato è ciò che tiene un dito lontano dalla faccia che la telecamera deve vedere e da quella che finirà appoggiata al tavolo, e un quaternione fisso non può farlo, perché quali facce si possono stringere dipende dalla faccia corrente e da quella obiettivo. Usalo per diagnosticare, poi rimettilo a zeri.

`grip_effort` sotto `1.0` non fa quello che sembra: l'adapter Robotiq lo converte a intero, quindi diventa `0` e il comando viene riportato come fallito anche se la pinza si muove. Per stringere meno agisci su `grip_close`.

## 5. Se qualcosa non va

**Un movimento fallisce con `-31 (NO_IK_SOLUTION)`.** Non significa necessariamente che la posa sia irraggiungibile. Leggi il log di `motion_server`: se compare `IK service call timeout`, il servizio non ha risposto in tempo. La causa più comune è che il dado afferrato risulti in collisione con le dita; il nodo `AllowAttachedContact` lo previene, ma se qualcosa lo salta ogni richiesta di IK con controllo collisioni fallisce. Controllo diretto:

```bash
ros2 service call /check_state_validity moveit_msgs/srv/GetStateValidity \
  "{robot_state: {is_diff: true}, group_name: manipulator}"
```

Se risponde `valid=False` con contatti fra `robotiq_hande_*_finger` e `dice`, è quello.

**Un movimento fallisce con `99999` in poche decine di millisecondi.** È il pianificatore cartesiano che non riesce a interpolare in linea retta. Le rotazioni sono in giunti proprio per questo; se il fallimento è su una traslazione, di solito il percorso è troppo lungo o parte da uno stato in collisione.

**Un movimento fallisce ma il braccio si è mosso davvero.** Nel log compare `Goal reached, success!` seguito da `unknown result response, ignoring...`: è una corsa in `rclcpp_action`, il risultato si perde e `motion_server` dichiara fallito un comando riuscito. I movimenti nell'albero sono protetti da `RetryUntilSuccessful` per questo.

**RViz mostra il robot che sfarfalla, o compaiono molti `TF_OLD_DATA`.** Qualcun altro sta pubblicando sul tuo dominio DDS. Verifica `ROS_DOMAIN_ID` e che non ci sia più di un'istanza dello stack:

```bash
ps -eo pid,args | grep -E "[m]ove_group|[e]asy_motion/motion_server"
```

**La missione si comporta in modo diverso fra due lanci identici.** Controlla di non avere due stack, due spawner o due riproduzioni di bag attive insieme: succede facilmente perché `pkill` non sempre chiude i processi figli. Nel dubbio, `docker restart drims2` e riparti dal punto 2.1.

## 6. Cosa è stato verificato

In simulazione, cella 1: missione completa dal centro dell'area e da tutti e quattro gli angoli a 3 cm dal bordo, con il dado ruotato di 0.5, 1.2 e 2.5 radianti. Percorsi a uno e a due passi di rotazione, entrambi conclusi sulla faccia richiesta.

Il `dice_detector` è stato verificato solo sulle registrazioni in `bags/setup_1`, contro le posizioni note in `bags/gt.txt`: sei rilevazioni su sei, errore di posizione fra 3.7 e 11.3 mm.

Due cose restano **non verificate** e vanno guardate per prime sul robot vero. La prima è la fase di identificazione dell'orientamento: in simulazione non è validabile, perché il simulatore non ricalcola quale faccia sia rivolta verso l'alto mentre il dado è in presa. La seconda è l'orientamento riportato dalla visione: la presa è comandata rispetto a `dice_tf`, quindi se quell'orientamento non è affidabile la pinza si avvicina storta. Controllalo con il braccio fermo, come al punto 3.2, prima di lanciare la missione.
