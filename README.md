# Demo ur_icam

## But
Ces exemples servent à montrer l'utilisation de différentes classes permettant de simuler sous Gazebo un robot Universal Robot, une webcam et des ventouses.

## Installation 

Depuis votre workspace ROS (on suppose que c'est ~/catkin_ws):

```
cd src
git clone https://github.com/raiv-toulouse/universal_robot.git
git clone https://github.com/raiv-toulouse/ur_icam.git
cd ..
catkin_make
```
## Utilisation
Ce programme lance Gazebo, y charge un robot UR5 muni d'une caméra et de 9 ventouses ainsi qu'une pièce rouge.

Le robot se déplace (en coord articulaires puis cartésiennes) afin de se positionner au dessus de la pièce.

A l'aide d'un topic, on va pouvoir activer les ventouses pour se saisir de la pièce.

```
roslaunch ur_icam ur_avec_camera.launch
```
dans un autre shell
```
rostopic pub /ur5/vacuum_gripper/grasp std_msgs/Bool "data: true" 
```

## Les classes
Les classes suivantes sont disponibles :
  * **RobotUR** ([robotUR.py](https://github.com/raiv-toulouse/ur_icam/blob/master/scripts/robotUR.py)) : 
  un robot UR5 que l'on peut déplacer à des coordonnées cartésiennes ou articulaires. La trajectoire est générée par MoveIt!
  * **MyCamera** ([camera.py](https://github.com/raiv-toulouse/ur_icam/blob/master/scripts/camera.py)) :
  une camera qui affiche l'image venant du topic /ur5/usbcam/image_raw et fournit un service permettant de sauver l'image sur disque 
  *  **Gripper** ([gripper.py](https://github.com/raiv-toulouse/ur_icam/blob/master/scripts/gripper.py)) :
  une ventouse que l'on peut activer ou désactiver à l'aide d'un service
  * **GripperSet** ([gripperSet.py](https://github.com/raiv-toulouse/ur_icam/blob/master/scripts/gripperSet.py)) :
  regroupe un ensemble de ventouses et permet de les activer toutes à l'aide d'un seul topic /ur5/vacuum_gripper/grasp
  
## Les programmes
Les programmes suivants mettent en oeuvre les classes précédentes et permettent de les tester.

### create_objects.py : 
Création à des positions aléatoires d'un ensemble d'objets.

Exemple d'intégration dans un fichier launch.
```
  <node name="create_objects" pkg="ur_icam" type="create_objects.py"
        args="--nb_objects 5 -x 0.3 -y 0.3 --dx 2 --dy 2 $(find ur_icam)/urdf/red_box.urdf"/>
```
### get_image.py :
Appelle le service de prise d'une photo.

Le node **camera.py** doit être lancé avant.

### move_cartesian.py :
Déplace le robot à une position cartésienne. Vous devez éditer le programme pour changer les coordonnées.

### move_robot_with_topic.py : 
Déplacement à une coordonnée articulaire à l'aide du topic /arm_controller/command

