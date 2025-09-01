# Raport z projektu: Platforma jezdna z lidarem 🚗💨

Projekt z przedmiotu **Sensory w Aplikacjach Wbudowanych**  
Kierunek: **Elektronika i Telekomunikacja – Systemy Wbudowane**  
Rok: I studiów magisterskich  

Autorzy:  
- Maciej Żurek  
- Michał Glos  
- Remigiusz Leśny  

---

## Opis projektu

Celem projektu jest stworzenie autonomicznej platformy jezdnej wyposażonej w lidar, zintegrowanej z **ROS2**, umożliwiającej:

- Wizualizację danych lidarowych w **RViz2**  
- Autonomiczną nawigację przy użyciu **Navigation2 (Nav2)**  

Platforma wykorzystuje sensory **LDRobot LDLidar** do pomiaru odległości i tworzenia mapy otoczenia w czasie rzeczywistym.  

---

## Architektura systemu


graph TD
A[Lidar LDLidar] -->|Dane skanowania| B[ldlidar_node]
B -->|Scan w ROS2| C[lifecycle_manager]
C --> D[RViz2 Visualization]
B --> E[Nav2 Navigation Stack]
E --> F[Autonomous Robot Motion]

Wymagania systemowe

    Ubuntu 22.04 (Jammy Jellyfish)

    ROS2 Humble

    Python 3.10+

    Rviz2, Nav2, colcon

Instalacja
1. Aktualizacja systemu

sudo apt update && sudo apt upgrade -y
sudo apt autoremove -y

2. Instalacja ROS2 Humble

sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update
sudo apt install ros-humble-desktop -y
sudo apt install ros-dev-tools -y
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc

3. Instalacja Navigation2

sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup -y
sudo apt install ros-humble-nav2-util -y

4. Instalacja LDLidar

mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/rem421/SWAW-car-lidar
sudo apt install libudev-dev -y
rosdep install --from-paths . --ignore-src -r -y
cd ~/ros2_ws
colcon build --symlink-install --cmake-args=-DCMAKE_BUILD_TYPE=Release
echo "source ~/ros2_ws/install/local_setup.bash" >> ~/.bashrc
source ~/.bashrc

Test i uruchomienie
1. Test komunikacji ROS2

ros2 run demo_nodes_cpp talker &
ros2 run demo_nodes_py listener

2. Uruchomienie lidara

ros2 launch ldlidar_node ldlidar_bringup.launch.py

3. Wyświetlanie skanu w RViz2

ros2 launch ldlidar_node ldlidar_rviz2.launch.py

Przykładowy GIF działania lidara w RViz2:

Parametry węzła LDLidar
Parametr	Opis	Przykładowe wartości
general.debug_mode	Aktywacja komunikatów debug	true / false
comm.serial_port	Ścieżka portu szeregowego	/dev/ttyUSB0
comm.baudrate	Prędkość portu szeregowego	115200
lidar.model	Model lidara	LDLiDAR_LD19
lidar.rot_verse	Kierunek obrotu	CW / CCW
lidar.units	Jednostki pomiaru	M / CM / MM
lidar.frame_id	Nazwa ramki TF	lidar_frame
lidar.bins	Liczba próbek skanu	0 lub 455
lidar.range_min	Minimalny dystans	np. 0.12 m
lidar.range_max	Maksymalny dystans	np. 8.0 m
lidar.enable_angle_crop	Przycinanie kąta	true / false
lidar.angle_crop_min	Minimalny kąt przycięcia	np. 0°
lidar.angle_crop_max	Maksymalny kąt przycięcia	np. 360°
