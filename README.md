# Demo ur_icam

## But
Ces exemples servent à montrer l'utilisation de différentes classes permettant de simuler sous Gazebo un robot Universal Robot, une webcam, des ventouses et une pince Robotiq85.

Un document décrit plus précisément le contenu de ce package (voir [ici](https://docs.google.com/document/d/1sBbloDQ2kFa2piVtJMc2v_1zyN-QUbDZUSSiXVrd0uw/edit#))

## Installation 

Depuis votre workspace ROS (on suppose que c'est ~/catkin_ws):

```
cd src
git clone https://github.com/raiv-toulouse/universal_robot.git
git clone https://github.com/raiv-toulouse/ur_icam.git
git clone https://github.com/raiv-toulouse/pkg_gripper.git 
cd ..
catkin_make
```
## Les démonstrations
### Robot + caméra + ensemble de ventouses
Ce programme lance Gazebo, y charge un robot UR5 muni d'une caméra et de 9 ventouses ainsi qu'une pièce rouge.

Le robot se déplace (en coord articulaires puis cartésiennes) afin de se positionner au dessus de la pièce.

A l'aide d'un topic, on va pouvoir activer les ventouses pour se saisir de la pièce.

```
roslaunch ur_icam_description ur_camera_vacuum_gripper.launch
```
dans un autre shell
```
rosrun ur_icam_description node_vacuum_gripper_set.py
ou bien
rosservice call /record_image "data: '/home/philippe'"
```
### Robot + pince Robotiq85
Ce programme empile 2 cubes sur un troisième.
```commandline
roslaunch ur_icam_gazebo ur5.launch
roslaunch ur_icam_moveit_config ur5_moveit_planning_execution.launch sim:=true
rosrun ur_icam_description node_stack_cubes.py
```

## Les classes
Les classes suivantes sont disponibles :
  * **RobotUR** ([robotUR.py](https://github.com/raiv-toulouse/ur_icam/blob/master/scripts/robotUR.py)) : 
  un robot UR que l'on peut déplacer à des coordonnées cartésiennes ou articulaires. La trajectoire est générée par MoveIt!
  * **Camera** ([camera.py](https://github.com/raiv-toulouse/ur_icam/blob/master/scripts/camera.py)) :
  une camera qui affiche l'image venant du topic /ur5/usbcam/image_raw et fournit un service permettant de sauver l'image sur disque 
  *  **VacuumGripper** ([vacuum_gripper.py](https://github.com/raiv-toulouse/ur_icam/blob/master/ur_icam_description/src/ur_icam_description/vacuum_gripper.py)) :
  une ventouse que l'on peut activer ou désactiver à l'aide d'un service
  * **VacuumGripperSet** ([vacuum_gripper_set.py](https://github.com/raiv-toulouse/ur_icam/blob/master/ur_icam_description/src/ur_icam_description/vacuum_gripper_set.py)) :
  regroupe un ensemble de ventouses et permet de les activer toutes à l'aide d'un seul topic /ur5/vacuum_gripper/grasp
  * **Gripper** et **Robotiq85Gripper** ([gripper.py](https://github.com/raiv-toulouse/ur_icam/blob/master/ur_icam_description/src/ur_icam_description/gripper.py)) : permet de piloter l'ouverture et la fermeture d'une pince Robotiq85

## Les nodes
Les programmes suivants mettent en oeuvre les classes précédentes et permettent de les tester.

### node_camera.py
Utilisation de la classe Camera pour ajouter une fenêtre visualisant l'image prise par une caméra. Il faut avoir ajouté une caméra dans le fichier URDF.

Ne pas oublier de paramétrer le répertoire de sauvegarde des images dans le constructeur de Camera.

### get_image.py
Appelle le service de prise d'une photo.

Le node **node_camera.py** doit être lancé avant.

### node_create_objects.py 
Création à des positions aléatoires d'un ensemble d'objets dans une simulation Gazebo.

Exemple d'intégration dans un fichier launch.
```
<node name="node_create_objects" pkg="ur_icam_description" type="node_create_objects.py"
      args="--nb_objects 5 -x 0.3 -y 0.3 --dx 2 --dy 2 $(find ur_icam_description)/urdf/red_box.urdf"/>
```
### node_move_cartesian.py
Déplace le robot à une position cartésienne. Vous devez éditer le programme pour changer les coordonnées.

### node_move_robot_with_topic.py 
Déplacement à une coordonnée articulaire à l'aide du topic /arm_controller/command

### node_pyqt_interact_robot.py
Affiche une interface permettant de déplacer le robot, à l'aide de MoveIt!, à partir de coordonnées articulaires ou cartésiennes.

### node_robotiq85_gripper.py
Ouvre/ferme la pince (0.0=ouvert , 0.8=fermé).
```
rosrun ur_icam_description node_robotiq85_gripper.py --value 0.4
```
### node_stack_cubes.py
Empile 2 cubes sur un troisième. Utilisation des classes RobotUR et Robotiq85Gripper.

### node_vacuum_gripper_set
Teste l'activation/désactivation d'un ensemble de ventouses à l'aide de la classe VacuumGripperSet