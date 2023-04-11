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

### Download kitti raw data

### Build and Run docker contatiner

Build the docker image

```bash
cd docker
docker build -t viconv:realtime .
```

Run docker image

```bash
docker run -it --privileged --gpus all --runtime=nvidia --net=host -e DISPLAY=$DISPLAY --ipc=host -v ~/Downloads/data/:/root/VirConv/data/ viconv:realtime
```

### Set up for VirConv

In docker container

```bash
cd VirConv
python setup.py develop
```
