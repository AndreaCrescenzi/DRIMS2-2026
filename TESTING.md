# Testing della challenge in simulazione

Procedura di verifica passo-passo per controllare che l'intera pipeline (robot fake + dado simulato + nodo homework) funzioni, lanciando i comandi uno alla volta in terminali separati. Pensato per essere rieseguito ogni volta che si modifica qualcosa in `drims_homework` o nello script di avvio del container.

## Prerequisiti

Container avviato con `sh start.sh` (o già in esecuzione: usare `sh connect.sh` per aprire altri terminali dentro lo stesso container). Workspace compilato almeno una volta con `colcon build` dentro `~/drims_ws`. Se il pacchetto `drims_homework` non viene trovato da `ros2 launch`, vedi la nota in fondo.

## Terminale 1 — Robot fake + MoveIt + easy_motion

```bash
sh start.sh
ros2 launch drims_description ur5e_1_start.launch.py fake:=true
```

Output atteso: caricamento dei controller (`joint_trajectory_controller`, `gripper_action_controller`), messaggio `You can start planning now!` da MoveIt, e `Motion server is ready to receive requests` da `easy_motion`. Se manca l'ultima riga, il nodo `motion_server` non è partito e i comandi successivi falliranno.

## Terminale 2 — Spawn del dado simulato

```bash
sh connect.sh
ros2 launch drims_dice_simulator spawn_dice.launch.py face_up:=5 position:="[0.6, 0.1, 0.85]"
```

Attenzione: la posizione è espressa rispetto a `base_link` del robot, non al mondo/tavolo. I bound validi in questa versione sono `x∈[0.4, 0.8]`, `y∈[-0.2, 0.4]`. L'esempio delle slide (`[-0.1, 0.0, 0.85]`) è fuori da questi bound e fa morire il nodo subito con `exit code 1` — non è un problema vostro, è un disallineamento tra slide e immagine Docker 2026 (vedi memoria `drims2-known-issues`).

Output atteso: `Spawned dice with: face 5 up ...` seguito da `AddObject response: ... success=True`.

## Terminale 3 — Nodo demo dell'homework

```bash
sh connect.sh
ros2 launch drims_homework demo_python_node_start.launch.py
```

Oppure, per la variante a behavior tree:

```bash
ros2 launch drims_homework demo_behavior_tree_start.launch.py
```

Output atteso: `Moving to home configuration...`, poi `Home reached: 1`, poi la posa del dado letta correttamente, poi `Moving to pick pose...`. Se qui compare `Package 'drims_homework' not found`, il workspace utente non è sorgentato in questa shell — vedi la nota sotto.

## Terminale 3 (bis) — Lanciare la challenge vera con `dice_mission.launch.py`

Questo è il modo giusto per lanciare l'albero consolidato della challenge (`dice_challenge.xml`, in `drims_homework`), con la faccia target scelta da riga di comando invece che editando l'XML a mano:

```bash
sh connect.sh
source ~/drims_ws/install/setup.bash
ros2 launch drims_homework dice_mission.launch.py target_face:=3
```

Cambiare `3` con la faccia desiderata (1-6). Il launch file riscrive automaticamente la riga `<Script code="target_face:=N"/>` nella copia installata di `dice_challenge.xml` prima di avviare `bt_executer_node` (non tocca mai il sorgente nel repo), e usa già la lista giusta di plugin tramite `config/dice_challenge_config.yaml` — non serve passare `-p ros_plugins:=...`/`-p plugins:=...` a mano.

Anche il punto di piazzamento finale (in `base_link`) è configurabile da riga di comando, con `place_x`/`place_y`/`place_z` (default `0.45`/`0.1`/`0.027`, verificati liberi dalla barra porta-telecamera della cella):

```bash
ros2 launch drims_homework dice_mission.launch.py target_face:=3 place_x:=0.5 place_y:=0.0 place_z:=0.03
```

`place_x`/`place_y` aggiornano sia il piazzamento finale sia il punto di ritrovo pre-rotazione (deliberatamente lo stesso punto, vedi i commenti in `dice_challenge.xml`) — cambiarli significa spostare entrambi insieme. `place_z` aggiorna solo l'altezza di rilascio finale. Attenzione: valori diversi da quelli di default non sono stati verificati liberi dalla barra — se la rotazione inizia a fallire dopo aver cambiato `place_x`/`place_y`, è probabile che il nuovo punto sia sotto l'ostacolo.

`dice_challenge.xml` gestisce già sia le rotazioni adiacenti (un solo ciclo presa-ruota-posa) sia le facce opposte (due cicli concatenati via una faccia intermedia, es. 1↔6, 2↔5, 3↔4) — non è più necessario evitarle. Include inoltre diversi livelli di fallback automatici (presa con angolo di avvicinamento diverso, instradamento tramite una faccia di passaggio) per i casi in cui la rotazione diretta risulti cinematicamente bloccata da un ostacolo reale della cella (la barra porta-telecamera sopra il tavolo — vedi il commento in testa a `dice_challenge.xml` per i dettagli).

Output atteso: `MoveToJoint Result: 1` (home), `Face number: N` (identificazione), `GetFaceRotation: N -> M => minimal angle ... deg`, i movimenti di presa/spostamento/rotazione (`MoveToPose Result: 1` ripetuti), `ComputeXYCorrection: ... -> correction [...]` per il piazzamento finale, e infine `DetachObject service responce received` seguito da `Tree completed with no errors`.

Per riprovare senza rilanciare tutto da capo, se il dado finisce in una posizione/stato strano:

```bash
ros2 service call /reset_dice std_srvs/srv/Trigger "{}"
```

Nota: `/reset_dice` ha un bug noto (race condition TF) che a volte, dopo il reset, riporta una faccia diversa da quella impostata al momento dello spawn — se serve una faccia precisa e sicura, meglio uccidere il processo `dice_spawner` (`pkill -f dice_spawner`) e rilanciare `spawn_dice.launch.py` da capo con il `face_up` voluto, piuttosto che fidarsi di `/reset_dice`.

### Variante storica: `_test_bt_move.xml` con `ros2 run` manuale

Prima che esistesse il launch file sopra, l'albero si lanciava così, con il target fissato a mano nel file XML (non più necessario, tenuto solo come riferimento):

```bash
sh connect.sh
source ~/drims_ws/install/setup.bash
ros2 run easy_motion_behavior_tree bt_executer_node --ros-args \
  -p ros_plugins:="['dice_identification','move_to_pose','move_to_joint','gripper_command','attach_object','detach_object','get_face_rotation','compute_xy_correction']" \
  -p bt_package:=drims_homework \
  -p bt_xml_file:=_test_bt_move.xml
```

## Uso di `run_bt.sh`: eseguire un albero con recupero automatico

Dalla root del repo sull'host (non dentro il container):

```bash
sh run_bt.sh <nome_albero.xml> [tentativi_massimi]
# es.
sh run_bt.sh _test_bt_move.xml
sh run_bt.sh _test_bt_flip180_twocycle.xml 3
```

Lancia l'albero dentro il container (stessa lista `ros_plugins`/`plugins`
del test manuale sopra). Se rileva la firma di un blocco reale del server
(`SEND_GOAL_TIMEOUT`) — dovuto a un bug noto in `pymoveit2` (vedi
`drims2-known-issues`), non nostro — riavvia da solo l'intero stack del
robot (`motion_server`, `move_group`, RViz) e riprova, senza bisogno di
intervento manuale su terminale 1. Un fallimento genuino (es. vero
`NO_IK_SOLUTION` dopo i tentativi) **non** causa un riavvio, viene solo
segnalato.

Non gestisce ancora l'esaurimento dei participant DDS (`Failed to find a
free participant index for domain 0`) — quello richiede un riavvio
completo del container (`sh start.sh`), non solo dello stack robot.

Per `dice_challenge.xml`, `run_bt.sh` non accetta ancora `target_face`
da riga di comando come `dice_mission.launch.py` (vedi sopra) — lancia
l'albero così com'è installato al momento. Per scegliere la faccia e
avere comunque il recupero automatico dai blocchi, aggiornare prima il
file con `ros2 launch drims_homework dice_mission.launch.py
target_face:=N` (che riscrive la copia installata e poi esegue), oppure
lanciare `sh run_bt.sh dice_challenge.xml` subito dopo, prima che un
altro `colcon build` sovrascriva la modifica.

## Controlli incrociati (in un quarto terminale)

```bash
sh connect.sh
ros2 node list
ros2 action list
ros2 topic list
```

Nodi attesi: `move_group`, `motion_server_node`, `dice_spawner_node`, `controller_manager`, `joint_state_broadcaster`, `joint_trajectory_controller`, `gripper_action_controller`. Action attese: `/move_to_pose`, `/move_to_joint`, `/plan_to_pose`, `/plan_to_joint`, `/execute_trajectory`, `/gripper_action_controller/gripper_cmd`.

## Se `drims_homework` non viene trovato

Significa che questa shell non ha sorgentato `~/drims_ws/install/setup.bash`. Con la fork corrente (`AndreaCrescenzi/DRIMS2-2026`) questo viene fatto automaticamente ad ogni avvio grazie al fix in `start.sh`/`start_as_root.sh`; se il problema si ripresenta lo si aggira con:

```bash
source ~/drims_ws/install/setup.bash
```

## Pulizia a fine test

Per fermare tutto senza lasciare processi appesi nel container:

```bash
pkill -f 'ros2 launch'
```

Da rilanciare più volte se compaiono ancora processi (`move_group`, `motion_server`, `dice_spawner`) in `ps aux`.
