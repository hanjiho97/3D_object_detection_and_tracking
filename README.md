# object_detection_and_depth_estimation_with_carla

Install nvidia docker

```bash
sudo apt install -y nvidia-docker2
sudo systemctl daemon-reload
sudo systemctl restart docker
```

Pull the CARLA image

```bash
docker pull carlasim/carla:0.9.13
```

Run the CARLA container

```bash
docker run --rm --privileged --gpus all --runtime=nvidia  --net=host -e DISPLAY=$DISPLAY carlasim/carla:0.9.13 /bin/bash ./CarlaUE4.sh -RenderOffScreen
```

Build base image

```bash
git clone https://github.com/hanjiho97/object_detection_and_depth_estimation_with_carla.git
cd object_detection_and_depth_estimation_with_carla
docker build --tag team2_final:base . 
```
