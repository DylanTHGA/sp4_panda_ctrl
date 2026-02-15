# SP4-Panda-Control
## Beschreibung 
Dieses Repo ermöglicht die Steuerung des **Panda-Greifarms** von Franka Emika unter **ROS-1** über die terminalbasierte Steuerung `panda_cli_control`. Zusätzlich enthält es alle relevanten Dateien und Startskripte, um den Panda Greifarm in der nachgebildeten Laborumgebung der **Technischen Hochschule Georg Agricola** innerhalb der Simulationsumgebung **Gazebo** zu steuern. Der Node `panda_cli_control` dient neben der manuellen Bedienung auch als ROS-basierte Schnittstelle zum SweetPicker System. Dadurch kann der Panda Greifarm als ausführende Roboterkomponente im **SweetPicker-System der THGA** eingesetzt werden und anhand übermittelter Objektinformationen automatisierte **Pick-and-Place-Routinen** ausführen.

---

![alt text](/sp4_panda_ctrl/images/sp4xpanda.png)
---

## Voraussetzungen

### System

- Betriebssystem: Ubuntu 20.04 (Focal)
- ROS-Version: ROS Noetic (ROS 1)
- Roboter: Franka Emika Panda
- Kamera: Intel RealSense D435

### Softwareabhängigkeiten und ROS Pakete

| Paket | Version | Repository |
|------|--------|------------|
| libfranka | 0.9.2 | https://github.com/frankaemika/libfranka/tree/0.9.2 |
| franka_ros | 0.10.1 | https://github.com/frankaemika/franka_ros/tree/0.10.1 |
| MoveIt (panda_moveit_config) | 0.8.1 | https://github.com/moveit/panda_moveit_config/tree/0.8.1 |
| gazebo_ros_pkgs | 2.9.3 | https://classic.gazebosim.org |

---

## Installation

### 1. Catkin Workspace erstellen

```bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
catkin_init_workspace
cd ~/catkin_ws
catkin_make
```

ROS Umgebung dauerhaft einbinden:

```bash
gedit ~/.bashrc
```

Am Ende ergänzen:

```bash
source ~/catkin_ws/devel/setup.bash
```

---

### 2. Installation von libfranka

```bash
sudo apt update
sudo apt install -y build-essential cmake git \
  libpoco-dev libeigen3-dev libfmt-dev
```

Alte Installationen entfernen:

```bash
sudo apt remove "*libfranka*"
```

Repository klonen und Version auswählen:

```bash
git clone --recurse-submodules https://github.com/frankaemika/libfranka.git
cd libfranka
git checkout 0.9.2
git submodule update
```

Kompilieren:

```bash
mkdir build && cd build
cmake -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTS=OFF ..
make
```

---

### 3. Installation von franka_ros

```bash
cd ~/catkin_ws/src
git clone --recursive https://github.com/frankaemika/franka_ros.git
cd franka_ros
git checkout 0.10.1
git submodule update
```

ROS Abhängigkeiten installieren:

```bash
cd ~/catkin_ws
rosdep init
rosdep update --include-eol-distros
rosdep install --from-paths src --ignore-src \
  --rosdistro noetic -y --skip-keys libfranka
```

Kompilieren:

```bash
catkin_make -DCMAKE_BUILD_TYPE=Release \
  -DFranka_DIR:PATH=~/libfranka/build
source devel/setup.bash
```

---

### 4. Installation von Gazebo

```bash
sudo apt install -y \
  ros-noetic-gazebo-ros-pkgs \
  ros-noetic-gazebo-ros-control
```

---

### 5. Installation von MoveIt

```bash
cd ~/catkin_ws/src
git clone --recursive https://github.com/moveit/panda_moveit_config.git
```

---

### 6. Klonen des SP4 Panda Control Pakets

```bash
cd ~/catkin_ws/src
git clone --recursive https://github.com/DylanTHGA/sp4_panda_ctrl.git
```

---

## Startskripte zur Systeminitialisierung

Es gibt zwei Startskripte: 
### Simulation: `start_panda_gazebo_sp.sh`

Start:

```bash
chmod +x start_panda_gazebo_sp.sh
./start_panda_gazebo_sp.sh
```

Damit werden gestartet:

- `roslaunch sp4_panda_ctrl panda_demo_gazebo.launch`  
  Gazebo, RViz und MoveIt in der Simulationskonfiguration
- `rosrun sp4_panda_ctrl panda_cli_control`  
  CLI Steuerung und SweetPicker Anbindung
- `coordinate_translator`  
  Umrechnung SweetPicker Objektkoordinaten in eine Roboterzielpose

## Kamera-Topic für Bilddaten

Das simulierte Kamerabild ist unter folgendem ROS-Topic verfügbar:

```text
/image_raw
```
---

### Realbetrieb: `start_panda_real_sp.sh`

#### Voraussetzungen


- Der Roboter muss im **Automatikbetrieb (Automatic Mode)** laufen
-  **FCI muss in Franka Desk aktiviert sein**
- Die **Robot-IP** muss erreichbar sein (z. B. `172.16.0.2`)

#### Start

Zuerst die Roboteranbindung starten:

```bash
roslaunch panda_moveit_config franka_control.launch robot_ip:=172.16.0.2
```
Anschließend das Startskript ausführen:

```bash
chmod +x start_panda_real_sp.sh
./start_panda_real_sp.sh
```

Damit werden gestartet:

- `rosrun sp4_panda_ctrl panda_cli_control`  
  CLI Steuerung und SweetPicker Anbindung
- `coordinate_translator`  
  Umrechnung SweetPicker Objektkoordinaten in eine Roboterzielpose

---

## Verwendung des `panda_cli_control` Nodes zur Steuerung des Panda Greifarms

### Start über das Startskript

```bash
chmod +x start_panda_gazebo_sp.sh
./start_panda_gazebo_sp.sh
```

### Manuelles Starten einzelner Komponenten

Simulationsumgebung und RViz:

```bash
roslaunch panda_moveit_config demo_gazebo.launch
```

CLI Steuerung:

```bash
rosrun sp4_panda_ctrl panda_cli_control
```
Coordinate Translator:

```bash
rosrun sp4_panda_ctrl coordinate_translator
```


## CLI Befehle (`panda_cli_control`)

Nach dem Start von `panda_cli_control` können die folgenden Kommandos direkt im Terminal eingegeben werden.  
Mit `help` wird die Übersicht ebenfalls im Terminal ausgegeben.

### Allgemein

| Befehl | Parameter | Beschreibung |
|---|---|---|
| `help` | – | Zeigt alle Befehle an |
| `quit` | – | Beendet den Node und fährt ROS herunter |
| `stop` | – | Stoppt eine laufende Bewegung |

### Bewegungen

| Befehl | Parameter | Beschreibung |
|---|---|---|
| `mode` | `ptp\|lin` | Setzt den Bewegungsmodus für `move_to` |
| `move_to` | `x y z` | Bewegt den TCP zu `(x,y,z)` in Metern. Orientierung bleibt wie aktuell |
| `set_joints` | `j1 j2 j3 j4 j5 j6 j7` | Setzt alle 7 Gelenkwinkel in Grad |
| `pick` | `x y [angle_deg]` | Startet Pick Routine bei `(x,y)`. Optional TCP Yaw in Grad |

> Hinweis: Während eine Pick Routine läuft werden alle Befehle außer `stop` blockiert.

### TCP und Greifer

| Befehl | Parameter | Beschreibung |
|---|---|---|
| `open` | – | Öffnet den Greifer |
| `close` | – | Schließt den Greifer vollständig |
| `close_w` | `width_mm` | Schließt den Greifer auf eine definierte Öffnungsweite in mm |
| `turn_hand` | `angle_deg` | Dreht das Handgelenk um `angle_deg` Grad |
| `set_orientation` | `qw qx qy qz` | Setzt die Default Orientierung als Quaternion |

### Statusausgaben

| Befehl | Parameter | Beschreibung |
|---|---|---|
| `print_pose` | – | Gibt die aktuelle TCP Pose aus |
| `print_joints` | – | Gibt die aktuellen Gelenkwinkel aus |

### Vordefinierte Posen

| Befehl | Parameter | Beschreibung |
|---|---|---|
| `startPos` | – | Fährt in die vordefinierte Startpose |
| `observerPos` | – | Fährt in die vordefinierte Beobachterpose |
| `placePos` | – | Fährt in die vordefinierte Ablagepose |



