from __future__ import annotations
import math
from contextlib import nullcontext

import commands2
import wpimath
import wpilib
import typing

from commands2 import cmd, InstantCommand, RunCommand
from commands2.button import JoystickButton
from wpilib import XboxController, PS4Controller
from wpimath.controller import PIDController, ProfiledPIDControllerRadians, HolonomicDriveController
from wpimath.geometry import Pose2d, Rotation2d, Translation2d
from wpimath.trajectory import TrajectoryConfig, TrajectoryGenerator

from constants import AutoConstants, DriveConstants, OIConstants
from subsystems.drivesubsystem import DriveSubsystem

from commands.followobject import FollowObject
from subsystems.limelight_camera import LimelightCamera
from subsystems import limelight_camera
from commands.findobject import FindObject
from commands.alignwithtag import AlignWithTag

from commands.reset_xy import ResetXY, ResetSwerveFront
from subsystems.arm import Arm, ArmConstants
from constants import CANIDs

class RobotContainer:
    """
    This class is where the bulk of the robot should be declared. Since Command-based is a
    "declarative" paradigm, very little robot logic should actually be handled in the :class:`.Robot`
    periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
    subsystems, commands, and button mappings) should be declared here.
    """

    def __init__(self) -> None:
        # The robot's subsystems
        self.robotDrive = DriveSubsystem()

        # The driver's controller
        self.driverController = wpilib.XboxController(OIConstants.kDriverControllerPort)

        # Configure the button bindings and autos
        self.configureButtonBindings()
        self.configureAutos()

        # Configure default commands
        self.robotDrive.setDefaultCommand(
            # The left stick controls translation of the robot.
            # Turning is controlled by the X axis of the right stick.
            commands2.RunCommand(
                lambda: self.robotDrive.drive(
                    -wpimath.applyDeadband(
                        self.driverController.getLeftY(), OIConstants.kDriveDeadband
                    ),
                    -wpimath.applyDeadband(
                        self.driverController.getLeftX(), OIConstants.kDriveDeadband
                    ),
                    -wpimath.applyDeadband(
                        self.driverController.getRightX(), OIConstants.kDriveDeadband
                    ),
                    True,
                    True,
                ),
                self.robotDrive,
            )
        )
        from subsystems.arm import Arm, ArmConstants
#        self.arm = Arm(CANIds.kArmMotorRight, CANIds.kArmMotorLeft, True)

    def configureButtonBindings(self) -> None:
        """
        Use this method to define your button->command mappings. Buttons can be created by
        instantiating a :GenericHID or one of its subclasses (Joystick or XboxController),
        and then passing it to a JoystickButton.
        """
        ## start of arm joystick control code
        from commands2.button import JoystickButton
        from commands2 import RunCommand

        yButton = JoystickButton(self.driverController, wpilib.XboxController.Button.kY)
        yButton.onTrue(commands2.InstantCommand(lambda: self.arm.setAngleGoal(70)))
        ## end of arm joystick control code

        xButton = JoystickButton(self.driverController, XboxController.Button.kX)
        xButton.onTrue(ResetXY(x=0.0, y=0.0, headingDegrees=0.0, drivetrain=self.robotDrive))
        xButton.whileTrue(RunCommand(self.robotDrive.setX, self.robotDrive))  # use the swerve X brake when "X" is pressed

        yButton = JoystickButton(self.driverController, XboxController.Button.kY)
        yButton.onTrue(ResetSwerveFront(self.robotDrive))

    def disablePIDSubsystems(self) -> None:
        """Disables all ProfiledPIDSubsystem and PIDSubsystem instances.
        This should be called on robot disable to prevent integral windup."""

    def getAutonomousCommand(self) -> commands2.Command:
        """
        :returns: the command to run in autonomous
        """
        command = self.chosenAuto.getSelected()
        return command()

    def configureAutos(self):
        self.chosenAuto = wpilib.SendableChooser()
        # you can also set the default option, if needed
#        self.chosenAuto.setDefaultOption("trajectory example", self.getAutonomousTrajectoryExample)
#        self.chosenAuto.addOption("left blue", self.getAutonomousLeftBlue)
#        self.chosenAuto.addOption("left red", self.getAutonomousLeftRed)
        self.chosenAuto.setDefaultOption("auto test", self.getAutoTest)
        self.chosenAuto.addOption("follow object", self.getFollowObject)
        wpilib.SmartDashboard.putData("Chosen Auto", self.chosenAuto)

    def getFollowObject(self):
        findTag = FindObject(
            camera= LimelightCamera("limelight"),
            drivetrain= self.robotDrive,
            turnDegrees= -45,
            turnSpeed= 0.2,
            waitSeconds= 1.0
        )
        alignTag = AlignWithTag(
            camera= LimelightCamera("limelight"),
            drivetrain= self.robotDrive,
            speed= 0.2,
            reverse= False,
            pushForwardSeconds= 0.0,
            pushForwardSpeed= 0.2,
        )
        followTag = FollowObject(
            camera= LimelightCamera("limelight"),
            drivetrain= self.robotDrive,
            stepSeconds= 0.33,
            stopWhen= 1,
            smoothness=1.0,
            speed= 0.2
        )

        command = findTag.andThen(alignTag).andThen(followTag)
        return command

    def getAutoTest(self):
        setStartPose = ResetXY(x=8.795, y=3.013, headingDegrees=+50.000, drivetrain=self.robotDrive)

        from commands.jerky_trajectory import JerkyTrajectory
        goToFinish = JerkyTrajectory(
            drivetrain=self.robotDrive,
            endpoint=(14.270, 5.773, 0.682),
            waypoints=[
                (9.962, 4.638, 52.524),
                (11.966, 5.905, -11.459)
            ],
            swerve=True,
            speed=0.2)

        from commands.aimtodirection import AimToDirection
        aimDown = AimToDirection(degrees=90, drivetrain=self.robotDrive)

        command = setStartPose.andThen(goToFinish).andThen(aimDown)
        return command

    def getTestCommand(self) -> typing.Optional[commands2.Command]:
        """
        :returns: the command to run in test mode (to exercise all systems)
        """
        return None
