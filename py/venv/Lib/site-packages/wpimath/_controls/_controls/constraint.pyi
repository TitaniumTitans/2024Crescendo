from __future__ import annotations
import wpimath._controls._controls.controller
import wpimath.geometry._geometry
import wpimath.kinematics._kinematics
import wpimath.units
__all__ = ['CentripetalAccelerationConstraint', 'DifferentialDriveKinematicsConstraint', 'DifferentialDriveVoltageConstraint', 'EllipticalRegionConstraint', 'MaxVelocityConstraint', 'MecanumDriveKinematicsConstraint', 'RectangularRegionConstraint', 'SwerveDrive2KinematicsConstraint', 'SwerveDrive3KinematicsConstraint', 'SwerveDrive4KinematicsConstraint', 'SwerveDrive6KinematicsConstraint', 'TrajectoryConstraint']
class CentripetalAccelerationConstraint(TrajectoryConstraint):
    """
    A constraint on the maximum absolute centripetal acceleration allowed when
    traversing a trajectory. The centripetal acceleration of a robot is defined
    as the velocity squared divided by the radius of curvature.
    
    Effectively, limiting the maximum centripetal acceleration will cause the
    robot to slow down around tight turns, making it easier to track trajectories
    with sharp turns.
    """
    @staticmethod
    def fromFps(maxCentripetalAcceleration: wpimath.units.feet_per_second_squared) -> CentripetalAccelerationConstraint:
        ...
    def __init__(self, maxCentripetalAcceleration: wpimath.units.meters_per_second_squared) -> None:
        ...
    def maxVelocity(self, pose: wpimath.geometry._geometry.Pose2d, curvature: wpimath.units.radians_per_meter, velocity: wpimath.units.meters_per_second) -> wpimath.units.meters_per_second:
        ...
    def minMaxAcceleration(self, pose: wpimath.geometry._geometry.Pose2d, curvature: wpimath.units.radians_per_meter, speed: wpimath.units.meters_per_second) -> TrajectoryConstraint.MinMax:
        ...
class DifferentialDriveKinematicsConstraint(TrajectoryConstraint):
    """
    A class that enforces constraints on the differential drive kinematics.
    This can be used to ensure that the trajectory is constructed so that the
    commanded velocities for both sides of the drivetrain stay below a certain
    limit.
    """
    @staticmethod
    def fromFps(kinematics: wpimath.kinematics._kinematics.DifferentialDriveKinematics, maxSpeed: wpimath.units.feet_per_second) -> DifferentialDriveKinematicsConstraint:
        ...
    def __init__(self, kinematics: wpimath.kinematics._kinematics.DifferentialDriveKinematics, maxSpeed: wpimath.units.meters_per_second) -> None:
        ...
    def maxVelocity(self, pose: wpimath.geometry._geometry.Pose2d, curvature: wpimath.units.radians_per_meter, velocity: wpimath.units.meters_per_second) -> wpimath.units.meters_per_second:
        ...
    def minMaxAcceleration(self, pose: wpimath.geometry._geometry.Pose2d, curvature: wpimath.units.radians_per_meter, speed: wpimath.units.meters_per_second) -> TrajectoryConstraint.MinMax:
        ...
class DifferentialDriveVoltageConstraint(TrajectoryConstraint):
    """
    A class that enforces constraints on differential drive voltage expenditure
    based on the motor dynamics and the drive kinematics.  Ensures that the
    acceleration of any wheel of the robot while following the trajectory is
    never higher than what can be achieved with the given maximum voltage.
    """
    def __init__(self, feedforward: wpimath._controls._controls.controller.SimpleMotorFeedforwardMeters, kinematics: wpimath.kinematics._kinematics.DifferentialDriveKinematics, maxVoltage: wpimath.units.volts) -> None:
        """
        Creates a new DifferentialDriveVoltageConstraint.
        
        :param feedforward: A feedforward component describing the behavior of the
                            drive.
        :param kinematics:  A kinematics component describing the drive geometry.
        :param maxVoltage:  The maximum voltage available to the motors while
                            following the path. Should be somewhat less than the nominal battery
                            voltage (12V) to account for "voltage sag" due to current draw.
        """
    def maxVelocity(self, pose: wpimath.geometry._geometry.Pose2d, curvature: wpimath.units.radians_per_meter, velocity: wpimath.units.meters_per_second) -> wpimath.units.meters_per_second:
        ...
    def minMaxAcceleration(self, pose: wpimath.geometry._geometry.Pose2d, curvature: wpimath.units.radians_per_meter, speed: wpimath.units.meters_per_second) -> TrajectoryConstraint.MinMax:
        ...
class EllipticalRegionConstraint(TrajectoryConstraint):
    """
    Enforces a particular constraint only within an elliptical region.
    """
    @staticmethod
    def fromFeet(center: wpimath.geometry._geometry.Translation2d, xWidth: wpimath.units.feet, yWidth: wpimath.units.feet, rotation: wpimath.geometry._geometry.Rotation2d, constraint: TrajectoryConstraint) -> EllipticalRegionConstraint:
        ...
    def __init__(self, center: wpimath.geometry._geometry.Translation2d, xWidth: wpimath.units.meters, yWidth: wpimath.units.meters, rotation: wpimath.geometry._geometry.Rotation2d, constraint: TrajectoryConstraint) -> None:
        """
        Constructs a new EllipticalRegionConstraint.
        
        :param center:     The center of the ellipse in which to enforce the constraint.
        :param xWidth:     The width of the ellipse in which to enforce the constraint.
        :param yWidth:     The height of the ellipse in which to enforce the constraint.
        :param rotation:   The rotation to apply to all radii around the origin.
        :param constraint: The constraint to enforce when the robot is within the
                           region.
        """
    def isPoseInRegion(self, pose: wpimath.geometry._geometry.Pose2d) -> bool:
        """
        Returns whether the specified robot pose is within the region that the
        constraint is enforced in.
        
        :param pose: The robot pose.
        
        :returns: Whether the robot pose is within the constraint region.
        """
    def maxVelocity(self, pose: wpimath.geometry._geometry.Pose2d, curvature: wpimath.units.radians_per_meter, velocity: wpimath.units.meters_per_second) -> wpimath.units.meters_per_second:
        ...
    def minMaxAcceleration(self, pose: wpimath.geometry._geometry.Pose2d, curvature: wpimath.units.radians_per_meter, speed: wpimath.units.meters_per_second) -> TrajectoryConstraint.MinMax:
        ...
class MaxVelocityConstraint(TrajectoryConstraint):
    """
    Represents a constraint that enforces a max velocity. This can be composed
    with the EllipticalRegionConstraint or RectangularRegionConstraint to enforce
    a max velocity within a region.
    """
    @staticmethod
    def fromFps(maxVelocity: wpimath.units.feet_per_second) -> MaxVelocityConstraint:
        ...
    def __init__(self, maxVelocity: wpimath.units.meters_per_second) -> None:
        """
        Constructs a new MaxVelocityConstraint.
        
        :param maxVelocity: The max velocity.
        """
    def maxVelocity(self, pose: wpimath.geometry._geometry.Pose2d, curvature: wpimath.units.radians_per_meter, velocity: wpimath.units.meters_per_second) -> wpimath.units.meters_per_second:
        ...
    def minMaxAcceleration(self, pose: wpimath.geometry._geometry.Pose2d, curvature: wpimath.units.radians_per_meter, speed: wpimath.units.meters_per_second) -> TrajectoryConstraint.MinMax:
        ...
class MecanumDriveKinematicsConstraint(TrajectoryConstraint):
    """
    A class that enforces constraints on the mecanum drive kinematics.
    This can be used to ensure that the trajectory is constructed so that the
    commanded velocities for wheels of the drivetrain stay below a certain
    limit.
    """
    @staticmethod
    def fromFps(kinematics: wpimath.kinematics._kinematics.MecanumDriveKinematics, maxSpeed: wpimath.units.feet_per_second) -> MecanumDriveKinematicsConstraint:
        ...
    def __init__(self, kinematics: wpimath.kinematics._kinematics.MecanumDriveKinematics, maxSpeed: wpimath.units.meters_per_second) -> None:
        ...
    def maxVelocity(self, pose: wpimath.geometry._geometry.Pose2d, curvature: wpimath.units.radians_per_meter, velocity: wpimath.units.meters_per_second) -> wpimath.units.meters_per_second:
        ...
    def minMaxAcceleration(self, pose: wpimath.geometry._geometry.Pose2d, curvature: wpimath.units.radians_per_meter, speed: wpimath.units.meters_per_second) -> TrajectoryConstraint.MinMax:
        ...
class RectangularRegionConstraint(TrajectoryConstraint):
    """
    Enforces a particular constraint only within a rectangular region.
    """
    def __init__(self, bottomLeftPoint: wpimath.geometry._geometry.Translation2d, topRightPoint: wpimath.geometry._geometry.Translation2d, constraint: TrajectoryConstraint) -> None:
        """
        Constructs a new RectangularRegionConstraint.
        
        :param bottomLeftPoint: The bottom left point of the rectangular region in
                                which to enforce the constraint.
        :param topRightPoint:   The top right point of the rectangular region in which
                                to enforce the constraint.
        :param constraint:      The constraint to enforce when the robot is within the
                                region.
        """
    def isPoseInRegion(self, pose: wpimath.geometry._geometry.Pose2d) -> bool:
        """
        Returns whether the specified robot pose is within the region that the
        constraint is enforced in.
        
        :param pose: The robot pose.
        
        :returns: Whether the robot pose is within the constraint region.
        """
    def maxVelocity(self, pose: wpimath.geometry._geometry.Pose2d, curvature: wpimath.units.radians_per_meter, velocity: wpimath.units.meters_per_second) -> wpimath.units.meters_per_second:
        ...
    def minMaxAcceleration(self, pose: wpimath.geometry._geometry.Pose2d, curvature: wpimath.units.radians_per_meter, speed: wpimath.units.meters_per_second) -> TrajectoryConstraint.MinMax:
        ...
class SwerveDrive2KinematicsConstraint(TrajectoryConstraint):
    """
    A class that enforces constraints on the swerve drive kinematics.
    This can be used to ensure that the trajectory is constructed so that the
    commanded velocities of the wheels stay below a certain limit.
    """
    @staticmethod
    def fromFps(kinematics: wpimath.kinematics._kinematics.SwerveDrive2Kinematics, maxSpeed: wpimath.units.feet_per_second) -> SwerveDrive2KinematicsConstraint:
        ...
    def __init__(self, kinematics: wpimath.kinematics._kinematics.SwerveDrive2Kinematics, maxSpeed: wpimath.units.meters_per_second) -> None:
        ...
    def maxVelocity(self, pose: wpimath.geometry._geometry.Pose2d, curvature: wpimath.units.radians_per_meter, velocity: wpimath.units.meters_per_second) -> wpimath.units.meters_per_second:
        ...
    def minMaxAcceleration(self, pose: wpimath.geometry._geometry.Pose2d, curvature: wpimath.units.radians_per_meter, speed: wpimath.units.meters_per_second) -> TrajectoryConstraint.MinMax:
        ...
class SwerveDrive3KinematicsConstraint(TrajectoryConstraint):
    """
    A class that enforces constraints on the swerve drive kinematics.
    This can be used to ensure that the trajectory is constructed so that the
    commanded velocities of the wheels stay below a certain limit.
    """
    @staticmethod
    def fromFps(kinematics: wpimath.kinematics._kinematics.SwerveDrive3Kinematics, maxSpeed: wpimath.units.feet_per_second) -> SwerveDrive3KinematicsConstraint:
        ...
    def __init__(self, kinematics: wpimath.kinematics._kinematics.SwerveDrive3Kinematics, maxSpeed: wpimath.units.meters_per_second) -> None:
        ...
    def maxVelocity(self, pose: wpimath.geometry._geometry.Pose2d, curvature: wpimath.units.radians_per_meter, velocity: wpimath.units.meters_per_second) -> wpimath.units.meters_per_second:
        ...
    def minMaxAcceleration(self, pose: wpimath.geometry._geometry.Pose2d, curvature: wpimath.units.radians_per_meter, speed: wpimath.units.meters_per_second) -> TrajectoryConstraint.MinMax:
        ...
class SwerveDrive4KinematicsConstraint(TrajectoryConstraint):
    """
    A class that enforces constraints on the swerve drive kinematics.
    This can be used to ensure that the trajectory is constructed so that the
    commanded velocities of the wheels stay below a certain limit.
    """
    @staticmethod
    def fromFps(kinematics: wpimath.kinematics._kinematics.SwerveDrive4Kinematics, maxSpeed: wpimath.units.feet_per_second) -> SwerveDrive4KinematicsConstraint:
        ...
    def __init__(self, kinematics: wpimath.kinematics._kinematics.SwerveDrive4Kinematics, maxSpeed: wpimath.units.meters_per_second) -> None:
        ...
    def maxVelocity(self, pose: wpimath.geometry._geometry.Pose2d, curvature: wpimath.units.radians_per_meter, velocity: wpimath.units.meters_per_second) -> wpimath.units.meters_per_second:
        ...
    def minMaxAcceleration(self, pose: wpimath.geometry._geometry.Pose2d, curvature: wpimath.units.radians_per_meter, speed: wpimath.units.meters_per_second) -> TrajectoryConstraint.MinMax:
        ...
class SwerveDrive6KinematicsConstraint(TrajectoryConstraint):
    """
    A class that enforces constraints on the swerve drive kinematics.
    This can be used to ensure that the trajectory is constructed so that the
    commanded velocities of the wheels stay below a certain limit.
    """
    @staticmethod
    def fromFps(kinematics: wpimath.kinematics._kinematics.SwerveDrive6Kinematics, maxSpeed: wpimath.units.feet_per_second) -> SwerveDrive6KinematicsConstraint:
        ...
    def __init__(self, kinematics: wpimath.kinematics._kinematics.SwerveDrive6Kinematics, maxSpeed: wpimath.units.meters_per_second) -> None:
        ...
    def maxVelocity(self, pose: wpimath.geometry._geometry.Pose2d, curvature: wpimath.units.radians_per_meter, velocity: wpimath.units.meters_per_second) -> wpimath.units.meters_per_second:
        ...
    def minMaxAcceleration(self, pose: wpimath.geometry._geometry.Pose2d, curvature: wpimath.units.radians_per_meter, speed: wpimath.units.meters_per_second) -> TrajectoryConstraint.MinMax:
        ...
class TrajectoryConstraint:
    """
    An interface for defining user-defined velocity and acceleration constraints
    while generating trajectories.
    """
    class MinMax:
        """
        Represents a minimum and maximum acceleration.
        """
        def __init__(self) -> None:
            ...
        @property
        def maxAcceleration(self) -> wpimath.units.meters_per_second_squared:
            """
            The maximum acceleration.
            """
        @maxAcceleration.setter
        def maxAcceleration(self, arg0: wpimath.units.meters_per_second_squared) -> None:
            ...
        @property
        def minAcceleration(self) -> wpimath.units.meters_per_second_squared:
            """
            The minimum acceleration.
            """
        @minAcceleration.setter
        def minAcceleration(self, arg0: wpimath.units.meters_per_second_squared) -> None:
            ...
    def __init__(self) -> None:
        ...
    def maxVelocity(self, pose: wpimath.geometry._geometry.Pose2d, curvature: wpimath.units.radians_per_meter, velocity: wpimath.units.meters_per_second) -> wpimath.units.meters_per_second:
        """
        Returns the max velocity given the current pose and curvature.
        
        :param pose:      The pose at the current point in the trajectory.
        :param curvature: The curvature at the current point in the trajectory.
        :param velocity:  The velocity at the current point in the trajectory before
                          constraints are applied.
        
        :returns: The absolute maximum velocity.
        """
    def minMaxAcceleration(self, pose: wpimath.geometry._geometry.Pose2d, curvature: wpimath.units.radians_per_meter, speed: wpimath.units.meters_per_second) -> TrajectoryConstraint.MinMax:
        """
        Returns the minimum and maximum allowable acceleration for the trajectory
        given pose, curvature, and speed.
        
        :param pose:      The pose at the current point in the trajectory.
        :param curvature: The curvature at the current point in the trajectory.
        :param speed:     The speed at the current point in the trajectory.
        
        :returns: The min and max acceleration bounds.
        """
