import typing
import commands2
from wpimath.filter import SlewRateLimiter
from constants import ControllerConstants, PhysicalConstants
from subsystems.swervesubsystem import SwerveSubsystem


class DriveCommand(commands2.Command):
    def __init__(
        self,
        swerve_subsystem: SwerveSubsystem,
        y_speed: typing.Callable[[], float],
        x_speed: typing.Callable[[], float],
        rot_speed: typing.Callable[[], float],
        throttle: typing.Callable[[], float],
        field_oriented: typing.Callable[[], bool],
    ):
        super().__init__()

        self.swerve_subsystem = swerve_subsystem
        self.y_speed = y_speed
        self.x_speed = x_speed
        self.rot_speed = rot_speed
        self.throttle = throttle
        self.field_oriented = field_oriented

        self.y_limiter = SlewRateLimiter(PhysicalConstants.MAX_ACCELERATION)
        self.x_limiter = SlewRateLimiter(PhysicalConstants.MAX_ANGULAR_ACCELERATION)

    def execute(self):
        adj_throttle = (
            -(ControllerConstants.MAX_THROTTLE - ControllerConstants.MIN_THROTTLE) / 2
        ) + (ControllerConstants.MIN_THROTTLE + ControllerConstants.MAX_THROTTLE) / 2

        adj_y = (
            self._apply_deadband(
                self.y_speed(), ControllerConstants.FLIGHT_STICK_Y_DEADBAND
            )
            * adj_throttle
        )
        adj_x = (
            self._apply_deadband(
                -self.x_speed(), ControllerConstants.FLIGHT_STICK_X_DEADBAND
            )
            * adj_throttle
        )
        adj_rot = (
            self._apply_deadband(
                -self.rot_speed(), ControllerConstants.FLIGHT_STICK_Z_DEADBAND
            )
            * adj_throttle
        )

        self.swerve_subsystem.drive(adj_y, adj_x, adj_rot, self.field_oriented())

    def _apply_deadband(self, value: float, deadband: float) -> float:
        if abs(value) < deadband:
            return 0
        return value
