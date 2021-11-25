# labyrinth-project
Project for Mobile Robotics

Main: [main_script.py](main_script.py)

vision_utils
------------

```python
detect_aruco(img)
get_pos_aruco(img, detected, search_id)
localize_thymio(img, detected)
erase_aruco(img, detected)
detect_labyrinth(img, wall_size)
get_labyrinth_perspective(img)
crop_labyrinth(img, M)
calibrate_corners(cam)
```

generate_aruco.py
-----------------

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
