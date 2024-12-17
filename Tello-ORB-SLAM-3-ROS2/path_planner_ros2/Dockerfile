FROM osrf/ros:foxy-desktop

ARG ROS_DISTRO=foxy

RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /root/.bashrc

WORKDIR /root

RUN apt-get update && \
    apt-get install -y --no-install-recommends \
        software-properties-common

RUN apt-get update && \
    apt-get install -y --no-install-recommends \
        python3-pip && \
    rm -rf /var/lib/apt/lists/* && \
    apt-get clean

RUN pip3 install image opencv-python av

RUN echo "source /root/colcon_ws/install/local_setup.bash" >> /root/.bashrc

RUN mkdir -p colcon_ws/src

COPY src colcon_ws/src

RUN cd colcon_ws && \
    . /opt/ros/${ROS_DISTRO}/setup.sh && \
    colcon build --symlink-install --cmake-args -DCMAKE_CXX_FLAGS="-w"

RUN ldconfig

ENV DISPLAY=:0

CMD . /opt/ros/${ROS_DISTRO}/setup.sh ; \
    . /root/colcon_ws/install/local_setup.sh; \
    ros2 run path_planner_ros2 path_planner_node