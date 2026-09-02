#!/bin/bash
# Runs a Behavior Tree inside the running drims2 container, with automatic
# recovery if motion_server becomes unresponsive (SEND_GOAL_TIMEOUT).
#
# Root cause (see memory drims2-known-issues / TESTING.md): pymoveit2's
# MoveIt2.wait_until_executed() calls rclpy.spin_once() on the same node
# that motion_server.py already spins on its own MultiThreadedExecutor
# thread -- two competing spinners on one node is undefined in rclpy and
# can wedge the executor permanently. Not our code, not fixable without
# patching a third-party library in the ephemeral static workspace, so we
# work around it here instead: detect the wedge and restart just the robot
# launch stack automatically, rather than requiring a manual Ctrl+C.
#
# A real planning failure (e.g. genuine NO_IK_SOLUTION after retries) does
# NOT trigger a restart -- only the SEND_GOAL_TIMEOUT signature does, since
# that's specific to the executor being wedged, not a business failure.
#
# Usage: sh run_bt.sh <bt_xml_file> [max_attempts]
# Example: sh run_bt.sh _test_bt_move.xml

set -u

BT_FILE="${1:?Usage: sh run_bt.sh <bt_xml_file> [max_attempts]}"
MAX_ATTEMPTS="${2:-2}"
CONTAINER_NAME="drims2"

ROS_PLUGINS="['dice_identification','move_to_pose','move_to_joint','gripper_command','attach_object','detach_object','get_face_rotation','compute_xy_correction']"
PLUGINS="['get_grasp_orientation','plan_rotation_path']"

run_tree() {
    docker exec -i "$CONTAINER_NAME" bash -ic "
        source ~/drims_ws/install/setup.bash
        timeout 220 ros2 run easy_motion_behavior_tree bt_executer_node --ros-args \
            -p ros_plugins:=\"$ROS_PLUGINS\" \
            -p plugins:=\"$PLUGINS\" \
            -p bt_package:=drims_homework \
            -p bt_xml_file:=$BT_FILE
    " 2>&1
    echo "RUN_BT_EXIT_CODE:$?"
}

restart_robot_stack() {
    echo "[run_bt] motion_server appears wedged (SEND_GOAL_TIMEOUT). Restarting the robot launch stack..."
    docker exec "$CONTAINER_NAME" bash -c "
        pkill -f 'ros2 launch drims_description' 2>/dev/null
        pkill -f moveit_ros_move_group 2>/dev/null
        pkill -f 'easy_motion/motion_server' 2>/dev/null
        pkill -f rviz2 2>/dev/null
        sleep 2
    "
    docker exec -d "$CONTAINER_NAME" bash -ic '
        source ~/drims_ws/install/setup.bash
        ros2 launch drims_description ur5e_1_start.launch.py fake:=true > ~/robot_launch.log 2>&1
    '
    echo "[run_bt] Waiting for the robot stack to come back up..."
    for _ in $(seq 1 30); do
        sleep 2
        if docker exec "$CONTAINER_NAME" bash -c "pgrep -f moveit_ros_move_group" > /dev/null 2>&1; then
            echo "[run_bt] move_group is up."
            break
        fi
    done
    # move_group being alive doesn't mean controllers/action servers are
    # already serving -- seen in practice: bt_executor_node starts, spends
    # ~100s retrying the gripper action client, then the first MoveToJoint
    # gets ACTION_ABORTED because the stack genuinely wasn't ready yet.
    # Wait for the gripper action server specifically before declaring the
    # stack ready.
    echo "[run_bt] Waiting for the gripper action server..."
    for _ in $(seq 1 30); do
        sleep 2
        if docker exec -i "$CONTAINER_NAME" bash -ic "
            source ~/drims_ws/install/setup.bash
            timeout 5 ros2 action list
        " 2>/dev/null | grep -q "/gripper_action_controller/gripper_cmd"; then
            echo "[run_bt] gripper action server is up."
            break
        fi
    done
    sleep 5  # let motion_server finish initializing (virtual EE transform, etc.)

    echo "[run_bt] Resetting the die state, if the simulator is still running..."
    docker exec -i "$CONTAINER_NAME" bash -ic '
        source ~/drims_ws/install/setup.bash
        timeout 5 ros2 service call /reset_dice std_srvs/srv/Trigger "{}"
    ' > /dev/null 2>&1
}

attempt=1
while [ "$attempt" -le "$MAX_ATTEMPTS" ]; do
    echo "[run_bt] Attempt $attempt/$MAX_ATTEMPTS: running $BT_FILE"
    output="$(run_tree)"
    exit_code="$(echo "$output" | grep -o 'RUN_BT_EXIT_CODE:[0-9]*' | tail -1 | cut -d: -f2)"
    output="$(echo "$output" | grep -v '^RUN_BT_EXIT_CODE:')"
    echo "$output"

    if [ "$exit_code" = "0" ] && ! echo "$output" | grep -q "\[ERROR\]"; then
        echo "[run_bt] Tree completed with no errors."
        exit 0
    fi

    if echo "$output" | grep -q "Failed to find a free participant index"; then
        echo "[run_bt] CycloneDDS has run out of participant slots for this domain (too many short-lived ros2 processes across the session)."
        echo "[run_bt] This needs a FULL container restart (sh start.sh on the host), not just the robot stack -- giving up."
        exit 2
    fi

    if echo "$output" | grep -qE "terminate called|Segmentation fault"; then
        echo "[run_bt] Process crashed (not a robot wedge -- likely a bug in the tree XML or a node). Not restarting the robot stack; fix the tree/code first."
        exit 1
    fi

    if echo "$output" | grep -q "SEND_GOAL_TIMEOUT"; then
        restart_robot_stack
        attempt=$((attempt + 1))
        continue
    fi

    echo "[run_bt] Tree did not complete, but no wedge signature found (likely a real planning failure, e.g. NO_IK_SOLUTION) -- not restarting."
    exit 1
done

echo "[run_bt] Giving up after $MAX_ATTEMPTS attempts."
exit 1
