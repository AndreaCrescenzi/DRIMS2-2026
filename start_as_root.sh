#!/bin/bash

# Specify the container name
CONTAINER_NAME="drims2"
IMAGE_NAME="smentasti/drims2:2026"

# Pull the latest image
echo "Pulling the latest image: $IMAGE_NAME..."
docker pull $IMAGE_NAME
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

# See start.sh for why: CycloneDDS's default participant-index ceiling is
# easily exhausted by a session launching many short-lived ros2 CLI
# processes ("Failed to find a free participant index for domain 0").
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

# See start.sh for why: motion_server.py's IK/FK helper gives up on its
# own future after a hardcoded 3s even though it asked MoveIt for up to
# 20s, misreporting a slow-but-real answer as NO_IK_SOLUTION. Bumped to
# 8s. (A pymoveit2 patch was tried here too and reverted -- see start.sh,
# it broke MoveToJoint entirely. Do not reapply it.)
FIX_MOTION_BUGS='sed -i "s/timeout = 3\.0/timeout = 8.0/g" /home/drims/static/drims2_ws/install/easy_motion/lib/python3.10/site-packages/easy_motion/motion_server.py'

docker run -it --privileged -v /dev:/dev --env="DISPLAY" --env="QT_X11_NO_MITSHM=1" --net=host --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" --volume="$(pwd)/drims_ws:/home/drims/drims_ws" --volume="$(pwd)/bags:/home/drims/bags" --name drims2 -w /home/drims $IMAGE_NAME bash -c "$FIX_BASHRC; $FIX_CYCLONE; $FIX_MOTION_BUGS; exec bash"

