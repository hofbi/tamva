FROM telecarla

COPY . /home/catkin_ws/src/view-adaptation

WORKDIR /home/catkin_ws
RUN bash -c "source /opt/ros/noetic/setup.bash && catkin build"

ENTRYPOINT [ "src/view-adaptation/docker/entrypoint.sh" ]
