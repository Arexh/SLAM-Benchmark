FROM ubuntu:20.04

ARG MAKE_FLAG=-j

RUN ln -sf /usr/share/zoneinfo/Asia/Shanghai /etc/localtime

RUN apt-get update && apt-get install -y \
  wget gcc g++ git openssh-server \
  build-essential freeglut3-dev \
  libeigen3-dev libgoogle-glog-dev \
  libssl-dev libboost-all-dev \
  unzip libglew-dev

WORKDIR /root/

RUN wget https://github.com/Kitware/CMake/releases/download/v3.22.1/cmake-3.22.1.zip\
  && unzip cmake-3.22.1.zip && cd cmake-3.22.1 \
  && ./bootstrap && make ${MAKE_FLAG} \
  && make install && make clean && rm /root/cmake-3.22.1.zip

RUN git clone https://github.com/zzx2GH/Pangolin.git && cd Pangolin \
  && mkdir build && cd build && cmake -DBUILD_PANGOLIN_VIDEO=false .. .. \
  && make ${MAKE_FLAG} && make install && make clean

RUN wget https://github.com/ceres-solver/ceres-solver/archive/refs/tags/1.14.0.zip \
  && unzip 1.14.0.zip && cd ceres-solver-1.14.0/ && mkdir build && cd build && cmake ..\
  && make ${MAKE_FLAG} && make install && make clean && rm /root/1.14.0.zip

RUN wget https://github.com/opencv/opencv/archive/3.4.0.zip && unzip 3.4.0.zip && cd opencv-3.4.0 \
  && mkdir build && cd build && cmake -D CMAKE_BUILD_TYPE=Release -D CMAKE_INSTALL_PREFIX=/usr/local ..\
  && make ${MAKE_FLAG} && make install && make clean && rm 3.4.0.zip

RUN git clone https://github.com/Arexh/SLAM-Benchmark.git \
  && cd SLAM-Benchmark && mkdir build && cd build \
  && cmake .. && make ${MAKE_FLAG} && cd .. \
  && rm -rf build/ third_party/*/build third_party/*/Thirdparty/*/build

ARG SSH_CONFIG_FILE=/etc/ssh/sshd_config

RUN sed -i -E 's/.*PermitRootLogin .*/PermitRootLogin yes/g' ${SSH_CONFIG_FILE} \
    && sed -i -E 's/.*PubkeyAuthentication .*/PubkeyAuthentication yes/g' ${SSH_CONFIG_FILE} \
    && sed -i -E 's/.*AuthorizedKeysFile .*/AuthorizedKeysFile  %h\/.ssh\/authorized_keys/g' ${SSH_CONFIG_FILE} \
    && echo "root:vslam" | chpasswd \
    && /etc/init.d/ssh restart

CMD /usr/sbin/sshd -D
