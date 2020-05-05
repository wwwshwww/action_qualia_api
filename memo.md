# memo
## getting started
docker host
```
$ <docker build>
$ docker run -p 6080:80 -p 9090:9090 --shm-size=512m -e TZ=Asia/Tokyo husky_rosbridge:kinetic
```

ros commands in container
```
$ roscore
$ roslaunch rosbridge_server rosbridge_websocket.launch
$ rosrun tf2_web_republisher tf2_web_republisher
$ roslaunch husky_gazebo husky_playpen.launch
$ roslaunch husky_viz view_robot.launch
$ roslaunch husky_navigation gmapping_demo.launch
```

action_server debug
```
$ rosrun actionlib axclient.py <action server topic>
```
default action server topic: `/move_base`