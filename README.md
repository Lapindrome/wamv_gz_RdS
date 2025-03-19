# wamv_gz_rds: WAM-V Simulation for ROS 2 Jazzy + Gazebo Harmonic

**Auteurs**: [TURPAULT Maxence] & [VILLEGER Arthur]  
**Projet/Référent**: [Robotique de Service] / [MOLLARD Yoan] 

Ce projet permet la simulation du robot WAM-V (Wave Adaptive Modular Vessel) dans l'environnement Gazebo, utilisant ROS 2 Jazzy et Gazebo Harmonic. Il s'agit d'une plateforme idéale pour tester des algorithmes de contrôle et de perception avant leur implémentation sur un robot physique.

---

## Table des matières

- [wamv\_gz\_rds: WAM-V Simulation for ROS 2 Jazzy + Gazebo Harmonic](#wamv_gz_rds-wam-v-simulation-for-ros-2-jazzy--gazebo-harmonic)
  - [Table des matières](#table-des-matières)
  - [Introduction](#introduction)
  - [Prérequis](#prérequis)
  - [Capture vidéo](#capture-vidéo)

---

## Introduction

Le package `wamv_gz_rds` fournit une simulation réaliste du WAM-V dans un environnement Gazebo sur ROS 2 Jazzy. Ce projet est destiné à la recherche et au développement d'applications robotiques maritimes, en permettant une simulation avancée du robot avec des capteurs et des actions physiques réalistes. 

---

## Prérequis

Avant d'installer ce projet, vous aurez besoin des éléments suivants :

- **Système d'exploitation**: Ubuntu 24.04
- **ROS 2 Jazzy**: Suivez les instructions [d'installation de ROS 2 Jazzy](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html)
- **Gazebo Harmonic**: Suivez les étapes d'installation de Gazebo Harmonic [ici](https://gazebosim.org/docs/harmonic/install_ubuntu/)
- **ros_gz**: Installez le package ROS-Gazebo avec cette commande :
  ```bash
  sudo apt-get install ros-jazzy-ros-gz

## Installation

### Étapes d'installation :

1. **Installer ROS 2 Jazzy**  
   
    Suivez les instructions officielles pour installer ROS 2 Jazzy sur Ubuntu 24.04.

2. **Installer Gazebo Harmonic**  
   
    Installez Gazebo Harmonic selon les instructions fournies [ici](https://gazebosim.org/docs/harmonic/install_ubuntu/).

3. **Installer ros_gz**
   
    Installez le package ros_gz avec la commande suivante :
    ```bash
    sudo apt-get install ros-jazzy-ros-gz
    ```

4. **Cloner ce dépôt dans votre espace de travail ROS 2**  
    
    Créez un espace de travail ROS 2 (si ce n'est pas déjà fait) et clonez ce dépôt dans le répertoire `src` :
    ```bash
    cd ~/ros2_ws/src
    git clone https://github.com/<votre-username>/wamv_gz_rds.git
    ```

5. ## Construire le package
   
    Dans le répertoire racine de votre espace de travail ROS 2, compilez le projet avec la commande suivante :
    ```bash
    cd ~/ros2_ws
    colcon build
    ```

6. ## Sourcez l'environnement
    
    Une fois la construction terminée, sourcez votre espace de travail :
    ```bash
    source install/setup.bash
    ```

7. ## Lancer la simulation WAM-V
    
    Lancez le fichier de lancement principal pour démarrer la simulation :
    ```bash
    ros2 launch wamv_gz_rds wamv_launch.py
    ```


## Utilisation

### Contrôler le WAM-V

Après avoir lancé la simulation, vous pouvez contrôler les moteurs du WAM-V à l'aide de commandes ROS.

#### Activer les propulseurs

Utilisez les commandes suivantes pour contrôler les propulseurs gauche et droit :

# Activation du propulseur gauche, plage recommandée de -10.0 à 10.0
ros2 topic pub /wamv/thrusters/left/thrust std_msgs/msg/Float64 "data: 10.0"

# Activation du propulseur droit, plage recommandée de -10.0 à 10.0
ros2 topic pub /wamv/thrusters/right/thrust std_msgs/msg/Float64 "data: -4.0"


### Autres commandes

Vous pouvez également interagir avec d'autres parties du robot via les topics ROS disponibles. Consultez la liste complète dans la section suivante.

---

## Topics ROS disponibles

Voici les topics principaux disponibles pour l'interaction avec le robot WAM-V dans la simulation :

- `/wamv/ground_truth/odometry`: Odometry du robot (position et orientation).
- `/wamv/sensors/camera/front/camera_info`: Informations sur la caméra frontale.
- `/wamv/sensors/camera/front/image`: Flux d'images de la caméra frontale.
- `/wamv/sensors/front_lidar/points`: Données du lidar frontal sous forme de points.
- `/wamv/sensors/front_lidar/scan`: Scan du lidar frontal.
- `/wamv/sensors/gps/gps/fix`: Données GPS du robot.
- `/wamv/sensors/imu/imu/data`: Données de l'IMU (unité de mesure inertielle).

Pour explorer ces topics, vous pouvez utiliser la commande suivante :
```bash
ros2 topic echo <topic_name>
```

## Capture vidéo

Une démonstration vidéo de la simulation du WAM-V dans Gazebo est disponible ci-dessous :

[Screencast from 2024-12-29 18-36-39.webm](https://github.com/user-attachments/assets/71d9622f-a003-4d59-bfaf-0f32e410608c)
