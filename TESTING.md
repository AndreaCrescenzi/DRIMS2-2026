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

## Terminale 3 (bis) — Test del vero albero BT con GetFaceRotation/ComputeXYCorrection

Alternativa al punto precedente, per verificare la pipeline completa (presa, selezione della rotazione minima, correzione della posizione di rilascio) attraverso i nodi C++ custom in `dice_bt_nodes`:

```bash
sh connect.sh
source ~/drims_ws/install/setup.bash
ros2 run easy_motion_behavior_tree bt_executer_node --ros-args \
  -p ros_plugins:="['dice_identification','move_to_pose','move_to_joint','gripper_command','attach_object','detach_object','get_face_rotation','compute_xy_correction']" \
  -p bt_package:=drims_homework \
  -p bt_xml_file:=_test_bt_move.xml
```

Output atteso: `MoveToJoint Result: 1` (home), poi `MoveToPose Result: 1` (presa), `GetFaceRotation: 5 -> 3 => minimal angle 90.0 deg` (o l'angolo minimo corretto per la coppia di facce scelta), due `MoveToPose Result: 1` (sollevamento + rotazione), `ComputeXYCorrection: ... -> correction [...]` (piccola, pochi cm), un ultimo `MoveToPose Result: 1` (correzione posizione), e infine `DetachObject service responce received`.

Il target è fissato nel file (`target_face="3"`), da editare a mano nell'XML per provare altre facce. Evitare per ora le coppie di facce opposte (es. 5↔2, 1↔6, 3↔4): richiedono un flip di 180° che ha già bloccato `motion_server` una volta, va gestito diversamente (vedi memoria `drims2-challenge-status`).

Per riprovare senza rilanciare tutto da capo, se il dado finisce in una posizione/stato strano:

```bash
ros2 service call /reset_dice std_srvs/srv/Trigger "{}"
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
