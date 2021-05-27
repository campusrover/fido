import base64
import io
import typing
from abc import abstractmethod
from typing import Any, Callable, Mapping

import numpy as np
from PIL import Image
from roslibpy import Topic

from ...errors import RobotError

if typing.TYPE_CHECKING:
    from roslibpy import Ros

    from ...robot import Robot


class Sensor(object):
    """Represents a sensor on the robot.

    Internally, a sensor listens to a specific ROS topic and updates the robot's
    internal state. For instance, a Odomer listens to the /odom topic and update the
    robot's `x`, `y`, `z` coordinates. A sensor only starts listening to updates once
    the simulation has started. It will stop updating once the simulation is destroyed.
    """

    robot: "Robot"

    def __init__(self, robot: "Robot", **kwargs):
        self.robot = robot
        for k, v in kwargs.items():
            setattr(self, k, v)

    @abstractmethod
    def handle_updates(self, ros: "Ros") -> Callable[[], None]:
        """Handle incoming update from ROS topic and update robot's state accordingly.

        Args:
            ros (Ros): ROS client.

        Returns:
            A callback for unsubscribing the topic.
        """

        raise RobotError(
            "failed to call method on abstract sensor"
        ) from NotImplementedError("handle_updates() not implemented")


class Odomer(Sensor):
    """Represents an Odometry updater."""

    def handle_updates(self, ros: "Ros") -> Callable[[], None]:
        """Handle incoming update for `/odom` topic and update robot's position
        accordingly.

        Args:
            ros (Ros): ROS client.

        Returns:
            A callback for unsubscribing the topic.
        """
        topic = Topic(ros, "/odom", "nav_msgs/Odometry")
        topic.subscribe(self.__odom_handler)

        return lambda: topic.unsubscribe()

    def __odom_handler(self, msg: Mapping[Any, Any]) -> None:
        x = msg["pose"]["pose"]["position"]["x"]
        y = msg["pose"]["pose"]["position"]["y"]
        z = msg["pose"]["pose"]["position"]["z"]

        setattr(self.robot, "x", x)
        setattr(self.robot, "y", y)
        setattr(self.robot, "z", z)


class Lidar(Sensor):
    """Represents a Lidar sensor."""

    def handle_updates(self, ros: "Ros") -> Callable[[], None]:
        """Handle incoming update for `/scan` topic and update robot's ranges
        accordingly.

        Args:
            ros (Ros): ROS client.

        Returns:
            A callback for unsubscribing the topic.
        """
        topic = Topic(ros, "/scan", "sensor_msgs/LaserScan")
        topic.subscribe(self.__scan_handler)

        return lambda: topic.unsubscribe()

    def __scan_handler(self, msg: Mapping[Any, Any]) -> None:
        ranges = msg["ranges"]

        setattr(self.robot, "ranges", ranges)


class Camera(Sensor):
    """Represents a Camera sensor."""

    def handle_updates(self, ros: "Ros") -> Callable[[], None]:
        """Handle incoming update for `/camera/rgb/image_raw/compressed` topic and
        update robot's image accordingly.

        Args:
            ros (Ros): ROS client.

        Returns:
            A callback for unsubscribing the topic.
        """
        topic = Topic(
            ros, "/camera/rgb/image_raw/compressed", "sensor_msgs/CompressedImage"
        )
        topic.subscribe(self.__image_handler)

        return lambda: topic.unsubscribe()

    def __image_handler(self, msg: Mapping[Any, Any]) -> None:
        base64_bytes = msg["data"].encode("ascii")
        image_bytes = base64.b64decode(base64_bytes)
        image = Image.open(io.BytesIO(image_bytes))

        setattr(self.robot, "image", image)
