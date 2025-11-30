# CUDA 11.7, TensorRT 8.4.1
FROM nvcr.io/nvidia/tensorrt:22.07-py3 

ARG DEBIAN_FRONTEND=noninteractive

# Install miniconda
ENV CONDA_DIR /opt/conda
RUN wget --quiet https://repo.anaconda.com/miniconda/Miniconda3-latest-Linux-x86_64.sh -O ~/miniconda.sh && \
    /bin/bash ~/miniconda.sh -b -p /opt/conda

# Put conda in path so we can use conda activate
ENV PATH=$CONDA_DIR/bin:$PATH

RUN conda update -n base conda &&\
    conda install -n base conda-libmamba-solver &&\
    conda config --set solver libmamba

RUN git clone https://github.com/stschubert/VPR_Tutorial.git

RUN conda create -y -n vprtutorial python numpy pytorch torchvision natsort tqdm opencv pillow scikit-learn faiss matplotlib-base tensorflow tensorflow-hub tqdm scikit-image patchnetvlad -c conda-forge -c tobiasrobotics

RUN cd VPR_Tutorial &&\
    pip3 install --no-deps -e .
    
RUN apt-get -y update 
    
RUN apt-get install -y mesa-utils && \
    apt-get install -y libgl1-mesa-glx && \
    apt-get install -y libglu1-mesa-dev && \
    apt-get install -y libglew-dev &&\
    apt-get install -y libglvnd-dev &&\
    apt-get install -y libgl1-mesa-dev &&\
    apt-get install -y libegl1-mesa-dev &&\
    apt-get install -y mesa-common-dev

