# SP3-Panda-Control

Dieses ROS-Paket dient der Steuerung des Franka Emika Panda Cobots mit ROS 1 Noetic unter Ubuntu 20.04.

Es nutzt das `MoveIT Framework` für die Bewegungsplanung und enthält einen ausführbaren Node

- `panda_cli_control`: Interaktives Kommandozeilen-Tool zur Steuerung des Roboterarms  


---

## Voraussetzungen

### System

- **Betriebssystem**: Ubuntu 20.04 (Focal)  
- **ROS-Version**: ROS Noetic (ROS 1)  
- **Roboter**: Franka Emika Panda  
- **Kamera**: Intel RealSense D435

### Softwareabhängigkeiten und ROS-Pakete

| Paket                | Version | Repository / Anleitung |
|----------------------|---------|-------------------------|
| **libfranka**        | 0.9.2   | https://github.com/frankarobotics/libfranka/tree/0.9.2 |
| **franka_ros**       | 0.10.1  | https://github.com/frankaemika/franka_ros/tree/0.10.1 |
| **MoveIt** (panda_moveit_config) | 0.8.1 | https://github.com/moveit/panda_moveit_config/tree/0.8.1 |
| **gazebo_ros_pkgs**  | 2.9.3   | https://classic.gazebosim.org/tutorials?tut=ros_installing |

> Verwende exakt die oben genannten Versionen, um Kompatibilitätsprobleme zu vermeiden.

## Installation

### 1. Catkin Workspace erstellen

1. Workspace-Ordner anlegen:  
   ```bash
   mkdir -p ~/catkin_ws/src 
   ```
2. Workspace initialisieren:  
   ```bash
   cd ~/catkin_ws/src  
   catkin_init_workspace
   ```
3. Workspace kompilieren:  
   ```bash
   cd ~/catkin_ws  
   catkin_make
   ```

4. `.bashrc` anpassen:  
   Öffne die Datei:  
  ```bash
   gedit ~/.bashrc 
   ```
   Und füge folgende Zeile am Ende hinzu:  
   `source ~/catkin_ws/devel/setup.bash`

---

### 2. Installation von **libfranka**

#### 2.1 Abhängigkeiten installieren

```bash
sudo apt-get update  
sudo apt-get install -y build-essential cmake git libpoco-dev libeigen3-dev libfmt-dev
```

#### 2.2 Vorherige Installationen entfernen

```bash
sudo apt-get remove "*libfranka*"
```

#### 2.3 Repository klonen

```bash
git clone --recurse-submodules https://github.com/frankaemika/libfranka.git  
cd libfranka
```

#### 2.4 Version auswählen

```bash
git tag -l                 
git checkout 0.9.2       
git submodule update    
```

#### 2.5 Kompilieren

```bash
mkdir build  
cd build  
cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_PREFIX_PATH=/opt/openrobots/lib/cmake -DBUILD_TESTS=OFF ..  
make
```

---

### 3. Installation von **franka_ros**

```bash
cd ~/catkin_ws/src  
git clone --recursive https://github.com/frankaemika/franka_ros  
cd franka_ros  
git checkout 0.10.1  
git submodule update
```

#### 3.1 ROS-Abhängigkeiten installieren

```bash
cd ~/catkin_ws
rosdep init  
rosdep update --include-eol-distros  
rosdep install --from-paths src --ignore-src --rosdistro noetic -y --skip-keys libfranka  
rosdep install --from-paths . --ignore-src --rosdistro noetic -y
```

#### 3.2 Kompilieren

```bash
cd ~/catkin_ws  
catkin_make -DCMAKE_BUILD_TYPE=Release -DFranka_DIR:PATH=/home/student/libfranka/build  
source devel/setup.sh
```

---

### 4. Installation von **gazebo_ros_pkgs**

```bash
cd ~/catkin_ws/src  
sudo apt-get install -y ros-noetic-gazebo-ros-pkgs ros-noetic-gazebo-ros-control
```

---

### 5. Installation von **moveit**

```bash
cd ~/catkin_ws/src  
git clone --recursive https://github.com/moveit/panda_moveit_config.git
```

### 6. Klonenen des SP3-Panda-Ctrl Packet

```bash
cd ~/catkin_ws/src  
git clone --recursive https://github.com/DylanTHGA/sp3_panda_ctrl.git
```

## Änderungen zur Integration des Panda-Arms in die Simulation

### 1.1 Einbinden des SP3_PAnda_Control URDF-Verzeichnis 
- Das URDF-Verzeichnis vom SP3_Panda_CTRL Packet muss im `franka_ros/franka_description/ -Verzeichnis` enthalten sein. 
- Der gesamte Ordner kann in das `franka_ros/franka_description/ -Verzeichnis` verschoben werden.
### 1.2 Austausch der `panda.urdf.xacro` Datei 
- Die Datei `franka_ros/franka_description/robots/panda.urdf.xacro` muss ersetzt werden durch die `sp3_control/urdf/panda_urdf.xacro` Datei
### 1.2 Änderung des `z_offset` in der `panda.launch` 
- In der Datei franka_ros/franka_gazebo/launch/panda.launch muss der Parameter `z_offset` von `0.0` auf `0.79` geändert werden.

## Verwendung des `panda_cli_control` Nodes zur Steuerung des Panda-Greifarms

### Starten der Simulationsumgebung und RViz

```bash
roslaunch panda_moveit_config demo_gazebo.launch
```

Dieser Befehl startet die Simulationsumgebung in Gazebo sowie die RViz-Oberfläche mit MoveIt-Unterstützung.

---

### Starten der CLI-Steuerung des Roboters

```bash
rosrun sp3_panda_ctrl panda_cli_control
```
Mit diesem Node kann der Panda-Greifarm über eine Kommandozeile gesteuert werden.  
Befehle wie `move_pose`, `pick_xy`, `open_gripper`. sind darüber ausführbar.  
> Durch Eingabe von `help` im Terminal werden alle verfügbaren Kommandos angezeigt.

---

### Kamera-Topic für Bilddaten

Das aktuelle Kamerabild ist unter folgendem ROS-Topic verfügbar:

```
/image_raw
```

> Dieses Topic kann z. B. mit `rqt_image_view` oder in RViz abonniert werden.
