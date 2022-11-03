FROM ros:humble

RUN apt-get update && apt-get install -y \
    && apt install python3-vcstool \
    && apt-get install -y git 

ENV ros2_ws=/root/ur_ws
RUN mkdir -p $ros2_ws/src
WORKDIR $ros2_ws/src

RUN git clone -b ur5e-working-with-msa https://github.com/ipa-rar/ur5e_cell.git 

WORKDIR $ros2_ws
RUN vcs import src --skip-existing --input src/ur5e_cell/dependencies_rolling.repos
    
RUN rosdep update \
    && rosdep install --from-paths src --ignore-src -r -y \
    && echo "Dependecies are installed in the workspace"

RUN . /opt/ros/humble/setup.sh \
    && colcon build \
    && rm -rf /var/lib/apt/lists/* \
    && echo "Building packages completed"


COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"] 
CMD ["bash"]