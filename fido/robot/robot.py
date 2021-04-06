from abc import ABC, abstractmethod

from ..errors import RobotError, NotImplementedError
from ..ros import InstallFile


class Robot(ABC):
    """Represents a physical robot.

    A robot is a navigable collection of sensors modeled by a
    `fido.robot.Model`  which describes its physical appearance. Fido
    provides a collection of premade robots that can be used directly in
    a simulation.
    """

    def __init__(self, name, model_name):
        self.name = name
        self.model_name = model_name

    @abstractmethod
    def move(self, distance, duration, speed):
        """Move the robot at a certain distance at a certain speed or for a
        certain duration.

        To move backwards, set speed to negative. If the given speed is
        larger than the maximum speed, it will be set to the maximum
        speed.
        """
        raise RobotError(
            "failed to call method on abstract robot"
        ) from NotImplementedError("move() not implemented")

    @abstractmethod
    def rotate(self, angle, duration, speed):
        """Rotate the robot at a certain angle at a certain speed or for a
        certain duration.

        To rotate clockwise, set the speed to positive. To rotate in
        anti-clockwise, set the speed to negative. If the given speed is
        larger  than the maximum speed, it will be set to the maximum
        speed.
        """
        raise RobotError(
            "failed to call method on abstract robot"
        ) from NotImplementedError("rotate() not implemented")

    @abstractmethod
    def stop(self, forced=False):
        """Stop the robot.

        This is a blocking call. It will block execution until the robot
        is gracefully stopped unless `forced` is set to `True`.
        """
        raise RobotError(
            "failed to call method on abstract robot"
        ) from NotImplementedError("stop() not implemented")

    @abstractmethod
    def ros_urdf(self, path):
        """Exports a ROS compatible URDF representation of the robot.

        Internally, this is a wrapper around the internal
        `fido.robot.Model.ros_urdf()` method.
        """
        raise RobotError(
            "failed to call method on abstract robot"
        ) from NotImplementedError("ros_urdf() not implemented")

    @abstractmethod
    def robot_description(self):
        """"""
        raise RobotError(
            "failed to call method on abstract robot"
        ) from NotImplementedError("robot_description() not implemented")

    @abstractmethod
    def fill_dependency(self, installfile: InstallFile):
        """Fills the needed dependency to the given installfile."""
        raise RobotError(
            "failed to call method on abstract robot"
        ) from NotImplementedError("dependency() not implemented")
