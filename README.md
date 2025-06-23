# pcdet_ros2
pcdet_ros2 implementation for sdv

📌 **Source**: https://bitbucket.org/pradhanshrijal/pcdet_ros2/src/main/

📢 There are two branch, the main the one for CUDA 11.7 and the other for CUDA 12.


## → 📥 Setup
First you must have ros2 humble installed, CUDA 11.7 installed and python 3.10 enviroment created and activated. 

```bash
git clone https://github.com/Abraham1501/pcdet_ros2.git
cd pcdet_ros2
# Install all dependecies, libraries and repositories
./setup.shso
```
### Build

```bash
# Configure CUDA path
export PATH=/usr/local/cuda-11.7/bin:$PATH
export LD_LIBRARY_PATH=/usr/local/cuda-11.7/lib64:$LD_LIBRARY_PATH

colcon build --symlink-install
```
### Run 
```bash
source install/setup.sh
# Configure the enviroment of python with ROS2
export PYTHONPATH=<.../path-to-your-env>/lib/python3.10/site-packages:$PYTHONPATH
ros2 launch pcdet_ros2 sdv_pcdet.launch.py
```
## → 🐋 Dockerfiles

First set your GPU architecture in the dockerfile.

```bash
###########################################
#  OpenPCDet Installation image
###########################################

# Set GPU architecture
ENV TORCH_CUDA_ARCH_LIST="8.6" <-----or 8.0=Ampere, 7.5=Turing, 8.9=Ada Lovelace
```

Build the docker image from the */docker/x86_64* then run *create_container.sh*

```bash
docker build -t sdv_pcdet-ros2 .

./create_container.sh
```
Inside of container
```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install
# Run
ros2 launch pcdet_ros2 sdv_pcdet.launch.py
```

## → 📗 Configuration

Inside the pcdet_ros2 package, you can find the launch files  where you configure parameters and topic, also the models are saved in the checkpoint folder. Currently configured for *pv_rcnn_8369.pth*. More information in the original repository.

Check [OpenPCDet's Model Zoo](https://github.com/open-mmlab/OpenPCDet#model-zoo) for avaible models and weights

##  ✒️ Citation
```bash
@misc{openpcdet2020,
    title={OpenPCDet: An Open-source Toolbox for 3D Object Detection from Point Clouds},
    author={OpenPCDet Development Team},
    howpublished = {\url{https://github.com/open-mmlab/OpenPCDet}},
    year={2020}
}
```