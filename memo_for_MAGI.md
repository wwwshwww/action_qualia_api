# memo for when work in magi
## environment organization
### enable to swicthing workspaces(desktop)
reference: https://www.wagavulin.jp/entry/2016/07/03/073247

```
$ apt-get install compizconfig-settings-manager unity-tweak-tool
```

### preparring for ROS wrapper
install `rosbridge`,`tf_republisher`
```
$ apt-get install -y ros-kinetic-rosbridge-server ros-kinetic-tf2-web-republisher
```


### install and launch `husky`
`husky` is cool simulater.
```
$ apt-get install -y ros-kinetic-husky-navigation ros-kinetic-husky-gazebo ros-kinetic-husky-viz
```

lanuch:
```
$ roslaunch husky_gazebo husky_playpen.launch
$ roslaunch husky_viz view_robot.launch
$ roslaunch husky_navigation gmapping_demo.launch
```


