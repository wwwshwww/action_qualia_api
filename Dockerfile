FROM tiryoh/ros-desktop-vnc:kinetic

RUN apt-get update && apt-get install -y \
    ros-kinetic-husky-navigation \
    ros-kinetic-husky-gazebo \
    ros-kinetic-husky-viz \
    ros-kinetic-rosbridge-server \
    ros-kinetic-tf2-web-republisher

CMD echo 'husky environment for rosbridge'