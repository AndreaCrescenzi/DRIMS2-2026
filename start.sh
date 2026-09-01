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

docker run -it  --user drims --privileged -v /dev:/dev -v /dev/bus/usb:/dev/bus/usb --device=/dev/bus/usb --device-cgroup-rule='c 189:* rmw'  -v /etc/udev/rules.d:/etc/udev/rules.d --env="DISPLAY" --env="QT_X11_NO_MITSHM=1" --net=host --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" --volume="$(pwd)/drims_ws:/home/drims/drims_ws" --volume="$(pwd)/bags:/home/drims/bags"  --name drims2 -w /home/drims $IMAGE_NAME bash -c "$FIX_BASHRC; $FIX_CYCLONE; exec bash"
