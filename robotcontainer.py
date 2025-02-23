import typing

import commands2
import wpilib
import romi
from subsystems.swervesubsystem import SwerveSubsystem
from commands.drivecommand import DriveCommand


class RobotContainer:
    def __init__(self) -> None:
        self.swervesubsystem = SwerveSubsystem()

        self.joystick = wpilib.XboxController(0)
        self.onboardIO = romi.OnBoardIO(
            romi.OnBoardIO.ChannelMode.INPUT, romi.OnBoardIO.ChannelMode.INPUT
        )

        self.chooser = wpilib.SendableChooser()

        self._configureButtonBindings()

    def _configureButtonBindings(self):
        self.swervesubsystem.setDefaultCommand(
            DriveCommand(
                self.swervesubsystem,
                lambda: self.joystick.getLeftY(),
                lambda: self.joystick.getLeftX(),
                lambda: self.joystick.getRightX(),
                lambda: 1.0,
                lambda: self.joystick.getLeftBumperButton(),
            )
        )
        wpilib.SmartDashboard.putData(self.chooser)

    def getAutonomousCommand(self) -> typing.Optional[commands2.Command]:
        return self.chooser.getSelected()
