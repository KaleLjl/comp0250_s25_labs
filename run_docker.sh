#!/bin/bash

# Build the Docker image
echo "Building Docker image..."
docker build -t comp0250_ros .

# Enable X11 forwarding
echo "Setting up X11 forwarding..."
xhost +local:docker

# Run the container
echo "Running Docker container..."
docker run -it \
    --privileged \
    --network=host \
    -e DISPLAY=$DISPLAY \
    -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
    -v $(pwd):/root/comp0250_s25_labs \
    comp0250_ros
