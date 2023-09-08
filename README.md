# luxonis_support

- oak_d_node.py starts a ros node that emulates a realsense output at 640x480, and offers a snapshot service which publishes a 4K (3840*2160) image on the ~inspection/image_raw topic.

## requirements
- ros1 noetic
- install depthai API using instructions here: https://github.com/luxonis/depthai-ros#install-from-ros-binaries



## notes
- Video unit is limited to 4k, so full 12MP isnt available when using video unit.
https://docs.luxonis.com/projects/api/en/latest/components/nodes/color_camera/
```
                      ColorCamera node
               ┌──────────────────────────────┐
               │   ┌─────────────┐            │
               │   │    Image    │ raw        │     raw
               │   │    Sensor   │---┬--------├────────►
               │   └────▲────────┘   |        │
               │        │   ┌--------┘        │
               │      ┌─┴───▼─┐               │     isp
inputControl   │      │       │-------┬-------├────────►
──────────────►│------│  ISP  │ ┌─────▼────┐  │   video
               │      │       │ |          |--├────────►
               │      └───────┘ │   Image  │  │   still
inputConfig    │                │   Post-  │--├────────►
──────────────►│----------------|Processing│  │ preview
               │                │          │--├────────►
               │                └──────────┘  │
               └──────────────────────────────┘
```