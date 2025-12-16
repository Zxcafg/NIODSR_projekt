# ROS 2 – Sterowanie symulacyjnym robotem TurtleBot3 za pomocą obrazu z kamery

## 1. Opis projektu
Projekt został wykonany w środowisku **ROS 2**.  
Celem projektu jest implementacja prostego interfejsu umożliwiającego sterowanie
**symulacyjnym robotem mobilnym TurtleBot3** w środowisku **Gazebo** na podstawie
interakcji użytkownika z obrazem z kamery.

Sterowanie realizowane jest poprzez kliknięcie myszą w okno obrazu z kamery:
- kliknięcie **powyżej środka obrazu** powoduje ruch robota **do przodu**,
- kliknięcie **poniżej środka obrazu** powoduje ruch robota **do tyłu**.

Projekt spełnia wymagania:
- wykorzystania ROS 2,
- stworzenia własnego node’a,
- użycia zewnętrznych pakietów (usb_cam),
- publikacji projektu w repozytorium GitHub.

---

## 2. Wykorzystane technologie i pakiety
- ROS 2 Humble
- Python 3
- OpenCV
- Gazebo
- TurtleBot3
- `usb_cam`
- `cv_bridge`
- `sensor_msgs`
- `geometry_msgs`

---

## 3. Struktura projektu
Struktura pakietów w workspace ROS 2:

```text
ros2_ws/
└── src/
    ├── camera_subscriber/
    │   ├── camera_subscriber/
    │   │   └── camera_node.py
    │   ├── package.xml
    │   └── setup.py
    └── usb_cam/   (pakiet zewnętrzny)
```

## Wymagania
- Ubuntu 22.04
- ROS 2 Humble
- Gazebo
- TurtleBot3
- Podłączona kamera USB (lub kamera laptopa)

---

## Instalacja

### 1. Instalacja pakietu `usb_cam`
```bash
sudo apt update
sudo apt install ros-$ROS_DISTRO-usb-cam
```
Opcjonalnie wersja z repozytorium git:
```bash
sudo apt install ros-$ROS_DISTRO-usb-cam-git
```
### 2.Instalacja pakietu `turtlebot`
```bash
sudo apt install ros-humble-turtlebot3*
```
## Tworzienie pakietu
### 1. Utworzenie pakietu camera_subscriber

Przejdź do katalogu roboczego ROS 2:
```bash
cd ~/ros2_ws/src
```
Utwórz pakiet:
```bash
ros2 pkg create camera_subscriber --build-type ament_python --dependencies cv_bridge python3-opencv sensor_msgs geometry_msgs
```
### 2. Dodanie node’a

Przejdź do katalogu:
```bash
cd ~/ros2_ws/src/camera_subscriber/camera_subscriber
```
Utwórz plik:
```bash
touch camera_node.py
chmod +x camera_node.py
```
Wklej do pliku kod node’a (obsługa kamery, kliknięć i sterowania robotem).

### 3. Kod źródłowy node’a camera_node.py
```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point, Twist
from cv_bridge import CvBridge
import cv2
import numpy as np


class CameraNode(Node):
    def __init__(self):
        super().__init__('camera_node')

        self.bridge = CvBridge()
        self.window_name = "camera_interface"

        self.image_width = 700
        self.image_height = 512

        # Subskrypcja obrazu z kamery
        self.create_subscription(
            Image,
            "/image_raw",
            self.image_callback,
            10
        )

        # Publikacja punktu kliknięcia
        self.point_pub = self.create_publisher(Point, "/point", 10)

        # Subskrypcja punktu kliknięcia
        self.create_subscription(
            Point,
            "/point",
            self.point_callback,
            10
        )

        # Publikacja komend prędkości robota
        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)

    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

        self.image_height, self.image_width = frame.shape[:2]

        cv2.imshow(self.window_name, frame)
        cv2.setMouseCallback(self.window_name, self.mouse_callback)
        cv2.waitKey(1)

    def mouse_callback(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            # Zatrzymanie robota przed nową komendą
            stop_msg = Twist()
            self.cmd_pub.publish(stop_msg)

            # Publikacja punktu kliknięcia
            point_msg = Point()
            point_msg.x = float(x)
            point_msg.y = float(y)
            point_msg.z = 0.0

            self.point_pub.publish(point_msg)

    def point_callback(self, msg):
        center_y = self.image_height / 2
        twist = Twist()

        if msg.y < center_y:
            twist.linear.x = 0.5   # ruch do przodu
        else:
            twist.linear.x = -0.5  # ruch do tyłu

        self.cmd_pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = CameraNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
```

### 5. Opis działania node’a camera_node

Node camera_node pełni rolę interfejsu pomiędzy użytkownikiem a robotem.
Odpowiada on za:

- odbiór obrazu z kamery,

- obsługę kliknięć myszy,

- interpretację położenia kliknięcia,

- wysyłanie komend prędkości do robota.

Subskrybowane tematy

- /image_raw (sensor_msgs/Image) – obraz z kamery

- /point (geometry_msgs/Point) – punkt kliknięcia

Publikowane tematy

- /point (geometry_msgs/Point) – współrzędne kliknięcia

- /cmd_vel (geometry_msgs/Twist) – komendy ruchu robota
  
### 6. Modyfikacja setup.py

W pliku camera_subscriber/setup.py dodaj wpis:
```python
entry_points={
    'console_scripts': [
        'camera_node = camera_subscriber.camera_node:main',
    ],
},
```
## Budowanie projektu

W katalogu ros2_ws wykonaj:
```bash
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
```
## Uruchamianie projektu

Do poprawnego działania potrzebne są trzy terminale.

Terminal 1 – node sterujący
```bash
cd ~/ros2_ws
source install/setup.bash
ros2 run camera_subscriber camera_node
```
Terminal 2 – symulacja TurtleBot3 (Gazebo)

Konfiguracja:
```bash
export TURTLEBOT3_MODEL=burger
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:`ros2 pkg prefix turtlebot3_gazebo`/share/turtlebot3_gazebo/models/
```
Uruchomienie symulacji:
```bash
ros2 launch turtlebot3_gazebo empty_world.launch.py
```
Terminal 3 – kamera USB
```bash
source /opt/ros/humble/setup.bash
ros2 run usb_cam usb_cam_node_exe
```

## 9. Działanie systemu

Uruchamiane jest okno z obrazem z kamery.

Kliknięcie myszą w górnej części obrazu powoduje ruch robota do przodu.

Kliknięcie w dolnej części obrazu powoduje ruch robota do tyłu.

Każde kliknięcie resetuje poprzedni ruch robota.

## 10. Autorzy

Pavel Tshonek 155068
Pavel Skabeltsyn 155113
Projekt zespołowy – repozytorium zawiera commity wszystkich autorów zgodnie z wymaganiami.

## 11. Uwagi końcowe

Projekt stanowi wersję podstawową (ocena 3.0).

Sterowanie odbywa się wyłącznie w osi przód–tył.

Do poprawnego działania wymagane jest uruchomienie kamery.
