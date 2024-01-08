# foxglove-bridge-docker

Docker image for https://github.com/foxglove/ros-foxglove-bridge ROS 2 package

## Demo

1. Run demo

    ```bash
    git clone  https://github.com/foxglove/ros-foxglove-bridge
    cd ros-foxglove-bridge/demo
    docker compose up
    ```

2. Open Foxglove application in browser

To access Foxglove, input the following in your browser's search bar:

- `http://localhost:8080/ui` - if you work locally on your ROSbot,
- `http://<ROSBOT_IP>:8080/ui` - if you want to connect to a device connected to the same LAN,
- `http://<HUSARNET_NAME>:8080/ui` - if you want to connect to the device using Husarnet VPN.
