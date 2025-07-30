
xhost +local:docker
mkdir -p $HOME/siar_shared
docker run -it \
    --gpus all \
    --env="DISPLAY=$DISPLAY" \
    --name siar \
    --net=host \
    --mount type=bind,source=$HOME/siar_shared,target=/home/siar/siar_shared \
    --env NVIDIA_DRIVER_CAPABILITIES=all \
    --env QT_X11_NO_MITSHM=1 \
    --privileged \
    --volume /tmp/.X11-unix:/tmp/.X11-unix \
    siar \
    bash
    
docker rm siar
