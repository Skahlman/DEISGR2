FROM osrf/ros:foxy-desktop

ARG ROS_DISTRO=foxy

RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /root/.bashrc

#
# base packages
#

RUN apt-get update && \
    apt-get install -y --no-install-recommends \
        software-properties-common \
        libepoxy-dev \
        rsync

RUN apt-get update && \
    apt-get install -y --no-install-recommends \
        python3-pip && \
    rm -rf /var/lib/apt/lists/* && \
    apt-get clean


RUN apt-get update && \
    apt-get install -y --no-install-recommends \
        ros-${ROS_DISTRO}-vision-opencv \
        ros-${ROS_DISTRO}-ament-cmake \
        ros-${ROS_DISTRO}-message-filters

WORKDIR /root

RUN mkdir -p Install

#
# pangolin
#

RUN python3 -m pip install wheel setuptools

RUN git clone https://github.com/stevenlovegrove/Pangolin.git Install/Pangolin && \
    cd Install/Pangolin && \
    mkdir build && cd build && \
    cmake .. && \
    make -j && \
    make install

#
# orbslam3 & sophus
#

RUN git clone https://github.com/zang09/ORB-SLAM3-STEREO-FIXED.git Install/ORB_SLAM3 && \
    cd Install/ORB_SLAM3 && \
    chmod +x build.sh && \
    ./build.sh && \
    cd Thirdparty/Sophus/build && \
    make install


COPY ./excludefile.txt .
COPY ./config config
#
# remove unnecessary files
#
RUN rm -rf Install/Pangolin
RUN rsync -v -r --remove-source-files --exclude-from='excludefile.txt' Install/ORB_SLAM3 Install/ORB_SLAM3_DELETE
RUN rm -rf Install/ORB_SLAM3_DELETE

#
# orbslam ros node
#

RUN mkdir -p colcon_ws/src 
RUN git clone https://github.com/zang09/ORB_SLAM3_ROS2.git colcon_ws/src/orbslam3_ros2 && \
    cd colcon_ws/src/orbslam3_ros2 && \
    sed -i \
    's|set(ORB_SLAM3_ROOT_DIR "~/Install/ORB_SLAM/ORB_SLAM3")|set(ORB_SLAM3_ROOT_DIR "~/Install/ORB_SLAM3")|' \
    CMakeModules/FindORB_SLAM3.cmake

COPY adapt_send_data.patch colcon_ws/src/orbslam3_ros2

RUN cd colcon_ws/src/orbslam3_ros2 && git apply adapt_send_data.patch

RUN echo "source /root/colcon_ws/install/local_setup.bash" >> /root/.bashrc

RUN cd colcon_ws && \
    . /opt/ros/${ROS_DISTRO}/setup.sh && \
    colcon build --symlink-install --cmake-args -DCMAKE_CXX_FLAGS="-w" --packages-select orbslam3

RUN cd colcon_ws/src/orbslam3_ros2/vocabulary && \
    tar xzvf ORBvoc.txt.tar.gz && \
    rm ORBvoc.txt.tar.gz

RUN ldconfig

ENV DISPLAY=:0

CMD . /opt/ros/${ROS_DISTRO}/setup.sh ; \
    . /root/colcon_ws/install/local_setup.sh; \
    ros2 run orbslam3 mono colcon_ws/src/orbslam3_ros2/vocabulary/ORBvoc.txt \
                            config/tello.yaml