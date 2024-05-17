# rcb4

## Read worm gear magenc

```
from rcb4.armh7interface import ARMH7Interface
interface = ARMH7Interface()
interface.auto_open()

while True:
    sensors = interface.all_jointbase_sensors()
    for sensor in sensors:
        print(sensor.magenc)
```

## Calibrate worm gear servo module
https://gist.github.com/iory/53399a69cec6127558ae123059cc2961


### Move Kondo motors

```
from rcb4.armh7interface import ARMH7Interface
interface = ARMH7Interface()
interface.auto_open()

# Return all connected motor ids
interface.search_servo_ids()
# -> array([10])

# Read motor angles
interface.angle_vector()
# If arg is empty, hold all motors
interface.hold([10])
# Send angle (-20deg) command
interface.angle_vector([-20], servo_ids=[10])
# If arg is empty, free all motors
interface.free([10])
```

# ROS

## Start ROS controller

On radxa,

```
roslaunch kxr_controller kxr_controller.launch urdf_path:=PATH_TO_URDF servo_config_path:=PATH_TO_SERVO_CONFIG
```

## Roseus interface for KXR system

Start roseus interface for rcb4 on radxa

```
roslaunch kxreus eusmodel_server.launch
```

Start roseus client

```
roseus $(rospack find kxreus)/euslisp/kxr-interface.l
(kxr-init)
(send *ri* :servo-on)
(send *ri* :angle-vector (send *robot* :angle-vector) 3000)
(send *ri* :servo-off)
```
