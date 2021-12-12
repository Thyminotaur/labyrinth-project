# labyrinth-project
Project for Mobile Robotics.

Main: [Main.py](Main.py)

Requirements
------------

Install the requirements.

```bash
pip install -r requirements.txt
```

Make sure `opencv-contrib` is installed
instead of `opencv`. This is needed to use
the ArUco utilities.

Usage
-----

Run the project.

* Deactivate any laptop's webcam in the device manager
* Connect the external webcam + Thymio
* Start the project using:

```bash
python Main.py
```

Troubleshooting
---------------

Generate AruCo

No module named "cv2.aruco" : https://stackoverflow.com/questions/45972357/python-opencv-aruco-no-module-named-cv2-aruco

## ArUco

![aruco](data/aruco_id_2.png)

## Labyrinth layout

![labyrinth](data/labyrinth.jpg)

## Localize ArUco

![localize](data/localize.png)

## Detect Labyrinth

![detect labyrinth](data/detect_labyrinth.png)


![big labyrinth](data/big_labyrinth.png)

In ~ 12 ms

## Detect corners

![original corners](data/aruco_corners.png)

![corners cropped](data/aruco_corners_crop.png)
