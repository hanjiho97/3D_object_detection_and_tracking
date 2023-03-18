# object_detection_and_depth_estimation_with_carla

### Setting up NVIDIA Container Toolkit

Setup the package repository and the GPG key

```bash
distribution=$(. /etc/os-release;echo $ID$VERSION_ID) \
      && curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey | sudo gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg \
      && curl -s -L https://nvidia.github.io/libnvidia-container/$distribution/libnvidia-container.list | \
            sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' | \
            sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list
```

Install the nvidia-container-toolkit package

```bash
sudo apt-get update
sudo apt-get install -y nvidia-container-toolkit
```

```bash
sudo nvidia-ctk runtime configure --runtime=docker
```

```bash
sudo systemctl restart docker
```

---

### Pull the CARLA image

```bash
docker pull carlasim/carla:0.9.13
```

### Run the CARLA container

```bash
sudo docker run --privileged --gpus all --net=host -v /tmp/.X11-unix:/tmp/.X11-unix:rw carlasim/carla:0.9.13 /bin/bash ./CarlaUE4.sh -RenderOffScreen
```

---

### Foward X11

```bash
xhost +local:docker
```

### Build base image

```bash
git clone https://github.com/hanjiho97/object_detection_and_depth_estimation_with_carla.git
cd object_detection_and_depth_estimation_with_carla
sudo docker build --tag team2_final:base . 
```

### RUN the base container

```bash
sudo docker run -it --privileged --gpus all --runtime=nvidia --net=host -e DISPLAY=$DISPLAY team2_final:base
```

---

### excute program

```bash
roslaunch carla_ros_bridge carla_ros_bridge_with_example_ego_vehicle.launch
```
