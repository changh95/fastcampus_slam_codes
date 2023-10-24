# SHINE-Mapping

> Original repo link: https://github.com/PRBonn/SHINE_mapping

----

## How to build

### 1. Install docker
[https://docs.docker.com/engine/install/ubuntu/](https://docs.docker.com/engine/install/ubuntu/)
### 2. Install nvidia container runtime
[https://developer.nvidia.com/nvidia-container-runtime](https://developer.nvidia.com/nvidia-container-runtime)
### 3. Clone SHINE Mapping repository
```
git clone git@github.com:PRBonn/SHINE_mapping.git
cd SHINE_mapping
```
### 4. Build container
```
docker build --tag shine .
```
### 5. Run container with example
```
mkdir /tmp/shine_test_data
docker run --rm -v `pwd`/:/repository -v /tmp/shine_test_data:/data -it --gpus all shine
```
Results will be produced in `/tmp/shine_test_data/results`.
### 6. Run container on your own data
```
docker run --rm -v .:/repository -v ${MY_DATA_DIR}:/data -it --gpus all shine bash
```
where `${MY_DATA_DIR}` is the directory on the host with data in the format described in `config/kitti/docker_kitti_batch.yaml`.
Once inside the container RUN as described below. Results will be found on host in `${MY_DATA_DIR}/results` .

----
## Prepare data

Generally speaking, you only need to provide:
1. `pc_path` : the folder containing the point cloud (`.bin`, `.ply` or `.pcd` format) for each frame.
2. `pose_path` : the pose file (`.txt`) containing the transformation matrix of each frame. 
3. `calib_path` : the calib file (`.txt`) containing the static transformation between sensor and body frames (optional, would be identity matrix if set as `''`).

They all follow the [KITTI odometry data format](https://www.cvlibs.net/datasets/kitti/eval_odometry.php).

After preparing the data, you need to correctly set the data path (`pc_path`, `pose_path` and `calib_path`) in the config files under `config` folder. You may also set a path `output_root` to store the experiment results and logs.

Here, we provide the link to several publicly available datasets for testing SHINE Mapping:

### MaiCity synthetic LiDAR dataset

Download the dataset from [here](https://www.ipb.uni-bonn.de/data/mai-city-dataset/) or use the following script to download (3.4GB):

```
sh ./scripts/download_maicity.sh
```

### KITTI real-world LiDAR dataset

Download the full dataset from [here](https://www.cvlibs.net/datasets/kitti/eval_odometry.php).

If you want to use an example part of the dataset (seq 00) for the test, you can use the following script to download (117 MB):
```
sh ./scripts/download_kitti_example.sh
```

