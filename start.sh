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
# Every group on the lab LAN shares one DDS domain unless told otherwise,
# and the container runs with --net=host, so another group's simulator
# publishes into our /tf: symptom is a flood of "TF_OLD_DATA ignoring data
# from the past for frame wrist_3_link ... authority undetectable" plus a
# second robot_state_publisher on `ros2 topic info /tf --verbose` with only
# one such process in the container. Measured: 1296 such warnings in a
# single run on domain 0, zero after moving to our own domain. We are
# group 5.
ROS_DOMAIN_ID_GROUP=5
FIX_DOMAIN="grep -q ROS_DOMAIN_ID /home/drims/.bashrc || echo 'export ROS_DOMAIN_ID=$ROS_DOMAIN_ID_GROUP' >> /home/drims/.bashrc"

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

# motion_server.py's _compute_ik/_compute_fk ask MoveIt's /compute_ik
# for up to 20s (the ik_timeout ROS parameter) but give up on their own
# future after a hardcoded 3s and report NO_IK_SOLUTION --
# indistinguishable from a genuine failure. Bumped to 8s: enough margin
# for a slow-but-real answer without making a genuinely unreachable
# query hang for the full 20s every time. Patches the *installed* copy
# (colcon build here doesn't symlink-install), so no rebuild is needed
# -- just make sure motion_server is (re)started after the container
# comes up so it loads the patched file.
#
# NOTE: an earlier version of this fix also patched pymoveit2's six
# rclpy.spin_once(self._node, ...) calls (in wait_until_executed,
# get_ik, get_fk, ...) to threading.Event().wait(1.0), reasoning that
# motion_server.py already spins that same node on its own
# MultiThreadedExecutor thread and two competing spinners on one node
# is undefined in rclpy. That diagnosis wasn't wrong in the abstract,
# but the fix broke things in practice: with it applied, even the
# simplest MoveToJoint (previously 100% reliable) hung forever.
# Whatever the outer executor is doing, it evidently does NOT reliably
# pick up the completion callbacks these loops are waiting for on its
# own -- the inline spin_once, race or not, was load-bearing. Reverted;
# do not reapply without first understanding why the outer executor
# doesn't cover this on its own.
FIX_MOTION_BUGS='sed -i "s/timeout = 3\.0/timeout = 8.0/g" /home/drims/static/drims2_ws/install/easy_motion/lib/python3.10/site-packages/easy_motion/motion_server.py'

docker run -it  --group-add dialout --user drims --privileged -v /dev:/dev -v /dev/bus/usb:/dev/bus/usb --device=/dev/bus/usb --device-cgroup-rule='c 189:* rmw'  -v /etc/udev/rules.d:/etc/udev/rules.d --env="DISPLAY" --env="QT_X11_NO_MITSHM=1" --net=host --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" --volume="$(pwd)/drims_ws:/home/drims/drims_ws" --volume="$(pwd)/bags:/home/drims/bags"  --name drims2 -w /home/drims $IMAGE_NAME bash -c "$FIX_BASHRC; $FIX_CYCLONE; $FIX_DOMAIN; $FIX_MOTION_BUGS; exec bash"
