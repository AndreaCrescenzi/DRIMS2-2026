#!/bin/bash

# Specify the container name
CONTAINER_NAME="drims2"
IMAGE_NAME="smentasti/drims2:2026"
#IMAGE_NAME="my_image"

# Pull the latest image
echo "Pulling the latest image: $IMAGE_NAME..."
docker pull $IMAGE_NAME

# Grant X permissions
#xhost +si:localuser:$(whoami)
xhost +local:root
# Check if the container exists
if docker ps -a | grep -q $CONTAINER_NAME; then
    echo "Container $CONTAINER_NAME exists."

    # Check if the container is running
    if [ "$(docker inspect -f {{.State.Running}} $CONTAINER_NAME)" == "true" ]; then
        echo "Container $CONTAINER_NAME is running. Stopping it now..."
        docker stop $CONTAINER_NAME
        docker rm $CONTAINER_NAME
    else
        echo "Container $CONTAINER_NAME is not running."
        docker rm $CONTAINER_NAME
    fi
else
    echo "Container $CONTAINER_NAME does not exist."
fi

FIX_BASHRC='grep -q "drims_ws/install/setup.bash" /home/drims/.bashrc || echo "source /home/drims/drims_ws/install/setup.bash" >> /home/drims/.bashrc'

# CycloneDDS's default participant-index ceiling is easily exhausted by a
# dev/test session that launches many short-lived `ros2 <verb>` CLI
# processes (each is its own DDS participant; crashed/killed ones can hang
# around until their lease expires instead of releasing immediately) --
# symptom: "Failed to find a free participant index for domain 0". The
# repo already ships cyclone_config.xml with a much higher
# MaxAutoParticipantIndex, but it only gets activated by
# setup_robot_connection.sh (meant for connecting to a real robot). Write
# a minimal local-only version and activate it unconditionally so this
# doesn't bite pure-simulation sessions too.
FIX_CYCLONE='cat > /home/drims/cyclone_config.xml <<EOF
<CycloneDDS>
  <Domain>
    <Discovery>
      <ParticipantIndex>auto</ParticipantIndex>
      <MaxAutoParticipantIndex>1000</MaxAutoParticipantIndex>
    </Discovery>
  </Domain>
</CycloneDDS>
EOF
grep -q "CYCLONEDDS_URI" /home/drims/.bashrc || echo "export CYCLONEDDS_URI=/home/drims/cyclone_config.xml" >> /home/drims/.bashrc'

# Two real bugs in third-party code (pymoveit2, easy_motion's
# motion_server.py -- not ours, not in our fork, live in the ephemeral
# static workspace baked into the image) caused most of the
# NO_IK_SOLUTION/SEND_GOAL_TIMEOUT pain during today's testing (see
# TESTING.md / project memory for the full diagnosis):
#  1. pymoveit2/moveit2.py calls rclpy.spin_once(self._node, ...) in six
#     blocking-wait helpers (wait_until_executed, get_ik, get_fk, ...) --
#     but motion_server.py already spins that same node on its own
#     MultiThreadedExecutor thread. Two competing spinners on one node is
#     undefined in rclpy: can silently degrade (growing response times)
#     or wedge the executor completely, needing a process restart.
#     self._node is already being spun elsewhere, so these loops just
#     need to wait, not spin -- swapped for threading.Event().wait(1.0)
#     (threading is already imported in that file).
#  2. motion_server.py's _compute_ik/_compute_fk ask MoveIt's
#     /compute_ik for up to 20s (the ik_timeout ROS parameter) but give
#     up on their own future after a hardcoded 3s and report
#     NO_IK_SOLUTION -- indistinguishable from a genuine failure. Once
#     the executor is under any contention (including from bug 1), a
#     query move_group would answer in under a millisecond in isolation
#     misses that 3s window and gets misreported. Bumped to 8s: enough
#     margin for contention-induced delay without making a genuinely
#     unreachable query hang for the full 20s every time.
# Patches the *installed* copies (colcon build here doesn't
# symlink-install), so no rebuild is needed for these to take effect --
# just make sure move_group/motion_server are (re)started after the
# container comes up so they load the patched files.
FIX_MOTION_BUGS='sed -i "s/rclpy\.spin_once(self\._node, timeout_sec=1\.0)/threading.Event().wait(1.0)/g" /home/drims/static/drims2_ws/install/pymoveit2/local/lib/python3.10/dist-packages/pymoveit2/moveit2.py
sed -i "s/timeout = 3\.0/timeout = 8.0/g" /home/drims/static/drims2_ws/install/easy_motion/lib/python3.10/site-packages/easy_motion/motion_server.py'

docker run -it  --user drims --privileged -v /dev:/dev -v /dev/bus/usb:/dev/bus/usb --device=/dev/bus/usb --device-cgroup-rule='c 189:* rmw'  -v /etc/udev/rules.d:/etc/udev/rules.d --env="DISPLAY" --env="QT_X11_NO_MITSHM=1" --net=host --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" --volume="$(pwd)/drims_ws:/home/drims/drims_ws" --volume="$(pwd)/bags:/home/drims/bags"  --name drims2 -w /home/drims $IMAGE_NAME bash -c "$FIX_BASHRC; $FIX_CYCLONE; $FIX_MOTION_BUGS; exec bash"
