from __future__ import annotations
import typing
import wpilib._wpilib
import wpilib.interfaces._interfaces
import wpimath.geometry._geometry
import wpiutil._wpiutil
__all__ = ['DifferentialDrive', 'MecanumDrive', 'RobotDriveBase']
class DifferentialDrive(RobotDriveBase, wpiutil._wpiutil.Sendable):
    """
    A class for driving differential drive/skid-steer drive platforms such as
    the Kit of Parts drive base, "tank drive", or West Coast Drive.
    
    These drive bases typically have drop-center / skid-steer with two or more
    wheels per side (e.g., 6WD or 8WD). This class takes a MotorController per
    side. For four and six motor drivetrains, construct and pass in
    :class:`MotorControllerGroup` instances as follows.
    
    Four motor drivetrain::
    
      import wpilib.drive
    
      class Robot(wpilib.TimedRobot):
          def robotInit(self):
              self.front_left = wpilib.PWMVictorSPX(1)
              self.rear_left = wpilib.PWMVictorSPX(2)
              self.left = wpilib.MotorControllerGroup(self.front_left, self.rear_left)
    
              self.front_right = wpilib.PWMVictorSPX(3)
              self.rear_right = wpilib.PWMVictorSPX(4)
              self.right = wpilib.MotorControllerGroup(self.front_right, self.rear_right)
    
              self.drive = wpilib.drive.DifferentialDrive(self.left, self.right)
    
    Six motor drivetrain::
    
      import wpilib.drive
    
      class Robot(wpilib.TimedRobot):
          def robotInit(self):
              self.front_left = wpilib.PWMVictorSPX(1)
              self.mid_left = wpilib.PWMVictorSPX(2)
              self.rear_left = wpilib.PWMVictorSPX(3)
              self.left = wpilib.MotorControllerGroup(self.front_left, self.mid_left, self.rear_left)
    
              self.front_right = wpilib.PWMVictorSPX(4)
              self.mid_right = wpilib.PWMVictorSPX(5)
              self.rear_right = wpilib.PWMVictorSPX(6)
              self.right = wpilib.MotorControllerGroup(self.front_right, self.mid_right, self.rear_right)
    
              self.drive = wpilib.drive.DifferentialDrive(self.left, self.right)
    
    A differential drive robot has left and right wheels separated by an
    arbitrary width.
    
    Drive base diagram::
    
      |_______|
      | |   | |
        |   |
      |_|___|_|
      |       |
    
    Each Drive() function provides different inverse kinematic relations for a
    differential drive robot. Motor outputs for the right side are negated, so
    motor direction inversion by the user is usually unnecessary.
    
    This library uses the NED axes convention (North-East-Down as external
    reference in the world frame):
    http://www.nuclearprojects.com/ins/images/axis_big.png.
    
    The positive X axis points ahead, the positive Y axis points to the right,
    and the positive Z axis points down. Rotations follow the right-hand rule,
    so clockwise rotation around the Z axis is positive.
    
    Inputs smaller then 0.02 will be set to 0, and larger values will be scaled
    so that the full range is still used. This deadband value can be changed
    with SetDeadband().
    
    RobotDrive porting guide:
    
    * :meth:`tankDrive` is equivalent to ``RobotDrive.tankDrive``
      if a deadband of 0 is used.
    * :meth:`arcadeDrive` is equivalent to ``RobotDrive.arcadeDrive``
      if a deadband of 0 is used and the the rotation input is inverted,
      e.g. ``arcadeDrive(y, -rotation, squareInputs=False)``
    * :meth:`curvatureDrive` is similar in concept to
      ``RobotDrive.drive`` with the addition of a quick turn
      mode. However, it is not designed to give exactly the same response.
    """
    class WheelSpeeds:
        """
        Wheel speeds for a differential drive.
        
        Uses normalized voltage [-1.0..1.0].
        """
        def __init__(self) -> None:
            ...
        @property
        def left(self) -> float:
            """
            Left wheel speed.
            """
        @left.setter
        def left(self, arg0: float) -> None:
            ...
        @property
        def right(self) -> float:
            """
            Right wheel speed.
            """
        @right.setter
        def right(self, arg0: float) -> None:
            ...
    @staticmethod
    def arcadeDriveIK(xSpeed: float, zRotation: float, squareInputs: bool = True) -> DifferentialDrive.WheelSpeeds:
        """
        Arcade drive inverse kinematics for differential drive platform.
        
        Note: Some drivers may prefer inverted rotation controls. This can be done
        by negating the value passed for rotation.
        
        :param xSpeed:       The speed at which the robot should drive along the X
                             axis [-1.0..1.0]. Forward is positive.
        :param zRotation:    The rotation rate of the robot around the Z axis
                             [-1.0..1.0]. Clockwise is positive.
        :param squareInputs: If set, decreases the input sensitivity at low speeds.
        
        :returns: Wheel speeds [-1.0..1.0].
        """
    @staticmethod
    def curvatureDriveIK(xSpeed: float, zRotation: float, allowTurnInPlace: bool) -> DifferentialDrive.WheelSpeeds:
        """
        Curvature drive inverse kinematics for differential drive platform.
        
        The rotation argument controls the curvature of the robot's path rather
        than its rate of heading change. This makes the robot more controllable at
        high speeds.
        
        :param xSpeed:           The robot's speed along the X axis [-1.0..1.0].
                                 Forward is positive.
        :param zRotation:        The normalized curvature [-1.0..1.0]. Clockwise is
                                 positive.
        :param allowTurnInPlace: If set, overrides constant-curvature turning for
                                 turn-in-place maneuvers. zRotation will control
                                 turning rate instead of curvature.
        
        :returns: Wheel speeds [-1.0..1.0].
        """
    @staticmethod
    def tankDriveIK(leftSpeed: float, rightSpeed: float, squareInputs: bool = True) -> DifferentialDrive.WheelSpeeds:
        """
        Tank drive inverse kinematics for differential drive platform.
        
        :param leftSpeed:    The robot left side's speed along the X axis
                             [-1.0..1.0]. Forward is positive.
        :param rightSpeed:   The robot right side's speed along the X axis
                             [-1.0..1.0]. Forward is positive.
        :param squareInputs: If set, decreases the input sensitivity at low speeds.
        
        :returns: Wheel speeds [-1.0..1.0].
        """
    @typing.overload
    def __init__(self, leftMotor: wpilib.interfaces._interfaces.MotorController, rightMotor: wpilib.interfaces._interfaces.MotorController) -> None:
        """
        Construct a DifferentialDrive.
        
        To pass multiple motors per side, use CAN motor controller followers or
        PWMSpeedController::AddFollower(). If a motor needs to be inverted, do so
        before passing it in.
        
        :param leftMotor:  Left motor.
        :param rightMotor: Right motor.
        """
    @typing.overload
    def __init__(self, leftMotor: typing.Callable[[float], None], rightMotor: typing.Callable[[float], None]) -> None:
        """
        Construct a DifferentialDrive.
        
        To pass multiple motors per side, use CAN motor controller followers or
        PWMSpeedController::AddFollower(). If a motor needs to be inverted, do so
        before passing it in.
        
        :param leftMotor:  Left motor setter.
        :param rightMotor: Right motor setter.
        """
    def arcadeDrive(self, xSpeed: float, zRotation: float, squareInputs: bool = True) -> None:
        """
        Arcade drive method for differential drive platform.
        
        Note: Some drivers may prefer inverted rotation controls. This can be done
        by negating the value passed for rotation.
        
        :param xSpeed:       The speed at which the robot should drive along the X
                             axis [-1.0..1.0]. Forward is positive.
        :param zRotation:    The rotation rate of the robot around the Z axis
                             [-1.0..1.0]. Counterclockwise is positive.
        :param squareInputs: If set, decreases the input sensitivity at low speeds.
        """
    def curvatureDrive(self, xSpeed: float, zRotation: float, allowTurnInPlace: bool) -> None:
        """
        Curvature drive method for differential drive platform.
        
        The rotation argument controls the curvature of the robot's path rather
        than its rate of heading change. This makes the robot more controllable at
        high speeds.
        
        :param xSpeed:           The robot's speed along the X axis [-1.0..1.0].
                                 Forward is positive.
        :param zRotation:        The normalized curvature [-1.0..1.0].
                                 Counterclockwise is positive.
        :param allowTurnInPlace: If set, overrides constant-curvature turning for
                                 turn-in-place maneuvers. zRotation will control
                                 turning rate instead of curvature.
        """
    def getDescription(self) -> str:
        ...
    def initSendable(self, builder: wpiutil._wpiutil.SendableBuilder) -> None:
        ...
    def stopMotor(self) -> None:
        ...
    def tankDrive(self, leftSpeed: float, rightSpeed: float, squareInputs: bool = True) -> None:
        """
        Tank drive method for differential drive platform.
        
        :param leftSpeed:    The robot left side's speed along the X axis
                             [-1.0..1.0]. Forward is positive.
        :param rightSpeed:   The robot right side's speed along the X axis
                             [-1.0..1.0]. Forward is positive.
        :param squareInputs: If set, decreases the input sensitivity at low speeds.
        """
class MecanumDrive(RobotDriveBase, wpiutil._wpiutil.Sendable):
    """
    A class for driving Mecanum drive platforms.
    
    Mecanum drives are rectangular with one wheel on each corner. Each wheel has
    rollers toed in 45 degrees toward the front or back. When looking at the
    wheels from the top, the roller axles should form an X across the robot.
    
    Drive base diagram:
    ::
    
      \\_______/
      \\ |   | /
        |   |
      /_|___|_\\
      /       \\
    
    Each Drive() function provides different inverse kinematic relations for a
    Mecanum drive robot.
    
    This library uses the NWU axes convention (North-West-Up as external
    reference in the world frame). The positive X axis points ahead, the positive
    Y axis points to the left, and the positive Z axis points up. Rotations
    follow the right-hand rule, so counterclockwise rotation around the Z axis is
    positive.
    
    Note: the axis conventions used in this class differ from DifferentialDrive.
    This may change in a future year's WPILib release.
    
    Inputs smaller then 0.02 will be set to 0, and larger values will be scaled
    so that the full range is still used. This deadband value can be changed
    with SetDeadband().
    
    MotorSafety is enabled by default. The DriveCartesian or DrivePolar
    methods should be called periodically to avoid Motor Safety timeouts.
    """
    class WheelSpeeds:
        """
        Wheel speeds for a mecanum drive.
        
        Uses normalized voltage [-1.0..1.0].
        """
        def __init__(self) -> None:
            ...
        @property
        def frontLeft(self) -> float:
            """
            Front-left wheel speed.
            """
        @frontLeft.setter
        def frontLeft(self, arg0: float) -> None:
            ...
        @property
        def frontRight(self) -> float:
            """
            Front-right wheel speed.
            """
        @frontRight.setter
        def frontRight(self, arg0: float) -> None:
            ...
        @property
        def rearLeft(self) -> float:
            """
            Rear-left wheel speed.
            """
        @rearLeft.setter
        def rearLeft(self, arg0: float) -> None:
            ...
        @property
        def rearRight(self) -> float:
            """
            Rear-right wheel speed.
            """
        @rearRight.setter
        def rearRight(self, arg0: float) -> None:
            ...
    @staticmethod
    def driveCartesianIK(xSpeed: float, ySpeed: float, zRotation: float, gyroAngle: wpimath.geometry._geometry.Rotation2d = ...) -> MecanumDrive.WheelSpeeds:
        """
        Cartesian inverse kinematics for Mecanum platform.
        
        Angles are measured counterclockwise from the positive X axis. The robot's
        speed is independent from its angle or rotation rate.
        
        :param xSpeed:    The robot's speed along the X axis [-1.0..1.0]. Forward is
                          positive.
        :param ySpeed:    The robot's speed along the Y axis [-1.0..1.0]. Left is
                          positive.
        :param zRotation: The robot's rotation rate around the Z axis [-1.0..1.0].
                          Counterclockwise is positive.
        :param gyroAngle: The gyro heading around the Z axis. Use this to implement
                          field-oriented controls.
        
        :returns: Wheel speeds [-1.0..1.0].
        """
    @typing.overload
    def __init__(self, frontLeftMotor: wpilib.interfaces._interfaces.MotorController, rearLeftMotor: wpilib.interfaces._interfaces.MotorController, frontRightMotor: wpilib.interfaces._interfaces.MotorController, rearRightMotor: wpilib.interfaces._interfaces.MotorController) -> None:
        """
        Construct a MecanumDrive.
        
        If a motor needs to be inverted, do so before passing it in.
        
        :param frontLeftMotor:  Front-left motor.
        :param rearLeftMotor:   Rear-left motor.
        :param frontRightMotor: Front-right motor.
        :param rearRightMotor:  Rear-right motor.
        """
    @typing.overload
    def __init__(self, frontLeftMotor: typing.Callable[[float], None], rearLeftMotor: typing.Callable[[float], None], frontRightMotor: typing.Callable[[float], None], rearRightMotor: typing.Callable[[float], None]) -> None:
        """
        Construct a MecanumDrive.
        
        If a motor needs to be inverted, do so before passing it in.
        
        :param frontLeftMotor:  Front-left motor setter.
        :param rearLeftMotor:   Rear-left motor setter.
        :param frontRightMotor: Front-right motor setter.
        :param rearRightMotor:  Rear-right motor setter.
        """
    def driveCartesian(self, xSpeed: float, ySpeed: float, zRotation: float, gyroAngle: wpimath.geometry._geometry.Rotation2d = ...) -> None:
        """
        Drive method for Mecanum platform.
        
        Angles are measured counterclockwise from the positive X axis. The robot's
        speed is independent from its angle or rotation rate.
        
        :param xSpeed:    The robot's speed along the X axis [-1.0..1.0]. Forward is
                          positive.
        :param ySpeed:    The robot's speed along the Y axis [-1.0..1.0]. Left is
                          positive.
        :param zRotation: The robot's rotation rate around the Z axis [-1.0..1.0].
                          Counterclockwise is positive.
        :param gyroAngle: The gyro heading around the Z axis. Use this to implement
                          field-oriented controls.
        """
    def drivePolar(self, magnitude: float, angle: wpimath.geometry._geometry.Rotation2d, zRotation: float) -> None:
        """
        Drive method for Mecanum platform.
        
        Angles are measured counterclockwise from the positive X axis. The robot's
        speed is independent from its angle or rotation rate.
        
        :param magnitude: The robot's speed at a given angle [-1.0..1.0]. Forward is
                          positive.
        :param angle:     The angle around the Z axis at which the robot drives.
        :param zRotation: The robot's rotation rate around the Z axis [-1.0..1.0].
                          Counterclockwise is positive.
        """
    def getDescription(self) -> str:
        ...
    def initSendable(self, builder: wpiutil._wpiutil.SendableBuilder) -> None:
        ...
    def stopMotor(self) -> None:
        ...
class RobotDriveBase(wpilib._wpilib.MotorSafety):
    """
    Common base class for drive platforms.
    
    MotorSafety is enabled by default.
    """
    class MotorType:
        """
        The location of a motor on the robot for the purpose of driving.
        
        Members:
        
          kFrontLeft : Front-left motor.
        
          kFrontRight : Front-right motor.
        
          kRearLeft : Rear-left motor.
        
          kRearRight : Rear-right motor.
        
          kLeft : Left motor.
        
          kRight : Right motor.
        
          kBack : Back motor.
        """
        __members__: typing.ClassVar[dict[str, RobotDriveBase.MotorType]]  # value = {'kFrontLeft': <MotorType.kFrontLeft: 0>, 'kFrontRight': <MotorType.kFrontRight: 1>, 'kRearLeft': <MotorType.kRearLeft: 2>, 'kRearRight': <MotorType.kRearRight: 3>, 'kLeft': <MotorType.kFrontLeft: 0>, 'kRight': <MotorType.kFrontRight: 1>, 'kBack': <MotorType.kRearLeft: 2>}
        kBack: typing.ClassVar[RobotDriveBase.MotorType]  # value = <MotorType.kRearLeft: 2>
        kFrontLeft: typing.ClassVar[RobotDriveBase.MotorType]  # value = <MotorType.kFrontLeft: 0>
        kFrontRight: typing.ClassVar[RobotDriveBase.MotorType]  # value = <MotorType.kFrontRight: 1>
        kLeft: typing.ClassVar[RobotDriveBase.MotorType]  # value = <MotorType.kFrontLeft: 0>
        kRearLeft: typing.ClassVar[RobotDriveBase.MotorType]  # value = <MotorType.kRearLeft: 2>
        kRearRight: typing.ClassVar[RobotDriveBase.MotorType]  # value = <MotorType.kRearRight: 3>
        kRight: typing.ClassVar[RobotDriveBase.MotorType]  # value = <MotorType.kFrontRight: 1>
        def __eq__(self, other: typing.Any) -> bool:
            ...
        def __getstate__(self) -> int:
            ...
        def __hash__(self) -> int:
            ...
        def __index__(self) -> int:
            ...
        def __init__(self, value: int) -> None:
            ...
        def __int__(self) -> int:
            ...
        def __ne__(self, other: typing.Any) -> bool:
            ...
        def __repr__(self) -> str:
            ...
        def __setstate__(self, state: int) -> None:
            ...
        def __str__(self) -> str:
            ...
        @property
        def name(self) -> str:
            ...
        @property
        def value(self) -> int:
            ...
    _kDefaultDeadband: typing.ClassVar[float] = 0.02
    _kDefaultMaxOutput: typing.ClassVar[float] = 1.0
    @staticmethod
    def _desaturate(wheelSpeeds: list[float]) -> None:
        """
        Renormalize all wheel speeds if the magnitude of any wheel is greater than
        1.0.
        """
    def __init__(self) -> None:
        ...
    def feedWatchdog(self) -> None:
        """
        Feed the motor safety object. Resets the timer that will stop the motors if
        it completes.
        
        @see MotorSafetyHelper::Feed()
        """
    def getDescription(self) -> str:
        ...
    def setDeadband(self, deadband: float) -> None:
        """
        Sets the deadband applied to the drive inputs (e.g., joystick values).
        
        The default value is 0.02. Inputs smaller than the deadband are set to 0.0
        while inputs larger than the deadband are scaled from 0.0 to 1.0. See
        frc::ApplyDeadband().
        
        :param deadband: The deadband to set.
        """
    def setMaxOutput(self, maxOutput: float) -> None:
        """
        Configure the scaling factor for using RobotDrive with motor controllers in
        a mode other than PercentVbus or to limit the maximum output.
        
        :param maxOutput: Multiplied with the output percentage computed by the
                          drive functions.
        """
    def stopMotor(self) -> None:
        ...
    @property
    def _m_deadband(self) -> float:
        """
        Input deadband.
        """
    @_m_deadband.setter
    def _m_deadband(self, arg0: float) -> None:
        ...
    @property
    def _m_maxOutput(self) -> float:
        """
        Maximum output.
        """
    @_m_maxOutput.setter
    def _m_maxOutput(self, arg0: float) -> None:
        ...
