# robot_location
Este proyecto consiste en un conjunto de nodos de ROS que permiten obtener la posición de un robot a partir de una cámara convencional y una serie de landmarks autoidentificativos (QR)

## Calibración y distorsión
<i>TO DO</i>

## Simulación
<i>TO DO</i>

## Entorno real
<i>TO DO</i>

## Comandos útiles

Para modificar los parámetros de la cámara añadida en la simulación:

<code>~/qr_robot_ws/src/qr_robot_description/urdf$ vim turtlebot_gazebo.urdf.xacro</code>

Para modificar los brazos de la cámara añadida en la simulación:

<code>~/qr_robot_ws/src/qr_robot_description/urdf/kobuki$ vim kobuki.urdf.xacro</code>

Para guardar una imagen del topic en el que publica la cámara:

<code>~/qr_robot_ws$ rosrun image_view image_view image:=/qr_robot/wide_camera/image_raw</code>

Para modificar el modelo del QR en la simulación: 

<code>~/.gazebo/models/qr0_0$ vim model.sdf</code>

Para abrir el mundo de Gazebo sin el turtlebot y modificarlo:

<code>~/qr_robot_ws$ roslaunch gazebo_ros empty_world.launch world_name:="/home/carmenballester/qr_robot_ws/src/qr_robot_gazebo/qr0_0.world"</code>

Para lanzar el entorno de simulación con Gazebo + Rviz (después de compilar el espacio de trabajo y de hacer source devel/setup.bash): 

<code>~/qr_robot_ws$ roslaunch qr_robot_gazebo turtlebot_world.launch</code>








