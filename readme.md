### turtlebot3_drive 코드 공부

* 실행 순서
<pre>
  1. 소스 코드 다운 및 빌드
     $ cd ~/catkin_ws/src/
     $ git clone -b kinetic-devel https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git
     $ cd ~/catkin_ws && catkin_make
     
  2. 환경 모델 world 불러오기
     $ export TURTLEBOT3_MODEL=burger
     $ source ./devel/setup.bash 
     $ roslaunch turtlebot3_gazebo turtlebot3_empty_world.launch
     
  3. 자동 시뮬레이션 - $ roslaunch turtlebot3_gazebo turtlebot3_simulation.launch
  4. rviz 시각화 - roslaunch turtlebot3_gazebo turtlebot3_simulation.launch 
  
</pre>

### turtlehouse3_house world, model 실행 및 simulation

- turtlehouse3_house.launch 먼저 실행

  $(find gazebo_ros)/launch/empty_world.launch
  위 경로의 empty_world 파일은 /opt/ros/melodic/share/gazebo_ros/launch에 있다.
  
  ```html
  <arg name="world_name" value="$(find turtlebot3_gazebo)/worlds/turtlebot3_house.world"/>
  위 경로에서 turtlebot3_house.world를 환경모델로 열어라

  <param name="robot_description" command="$(find xacro)/xacro --inorder $(find turtlebot3_description)/urdf/turtlebot3_$(arg model).urdf.xacro" />
  xacro 파일은 /opt/ros/melodic/share/turtlebot3_description/urdf 경로에서 터틀봇 모델 불러오기

  <node name="spawn_urdf" pkg="gazebo_ros" type="spawn_model"...
  /opt/ros/melodic/lib/gazebo_ros 경로의 gazebo_ros 패키지에서 spawn_model 노드 찾아 실행해라 
  ```

- roslaunch turtlebot3_gazebo turtlebot3_simulation.launch 실행
  ```html
  <node name="$(arg name)_drive" pkg="turtlebot3_gazebo" type="turtlebot3_drive" required="true" output="screen"/>
  turtlebot3_drive.cpp 파일 찾아서 실행
  ```

출처 : <https://github.com/ROBOTIS-GIT/turtlebot3_simulations/tree/master/turtlebot3_gazebo/src>
