from __future__ import annotations
import typing
import wpimath._controls._controls.constraint
import wpimath.geometry._geometry
import wpimath.kinematics._kinematics
import wpimath.spline._spline
import wpimath.units
__all__ = ['ExponentialProfileMeterVolts', 'Trajectory', 'TrajectoryConfig', 'TrajectoryGenerator', 'TrajectoryParameterizer', 'TrajectoryUtil', 'TrapezoidProfile', 'TrapezoidProfileRadians']
class ExponentialProfileMeterVolts:
    """
    A Exponential-shaped velocity profile.
    
    While this class can be used for a profiled movement from start to finish,
    the intended usage is to filter a reference's dynamics based on
    ExponentialProfile velocity constraints. To compute the reference obeying
    this constraint, do the following.
    
    Initialization:
    @code{.cpp}
    ExponentialProfile::Constraints constraints{kMaxV, kV, kA};
    State previousProfiledReference = {initialReference, 0_mps};
    
    Run on update:
    @code{.cpp}
    previousProfiledReference = profile.Calculate(timeSincePreviousUpdate,
    previousProfiledReference, unprofiledReference);
    
    where `unprofiledReference` is free to change between calls. Note that when
    the unprofiled reference is within the constraints, `Calculate()` returns the
    unprofiled reference unchanged.
    
    Otherwise, a timer can be started to provide monotonic values for
    `Calculate()` and to determine when the profile has completed via
    `IsFinished()`.
    """
    class Constraints:
        """
        Profile constraints.
        """
        @staticmethod
        def fromCharacteristics(maxInput: wpimath.units.volts, kV: wpimath.units.volt_seconds_per_meter, kA: wpimath.units.volt_seconds_squared_per_meter) -> ExponentialProfileMeterVolts.Constraints:
            ...
        @staticmethod
        def fromStateSpace(maxInput: wpimath.units.volts, a: wpimath.units.units_per_second, b: wpimath.units.meters_per_second_squared_per_volt) -> ExponentialProfileMeterVolts.Constraints:
            ...
        def maxVelocity(self) -> wpimath.units.meters_per_second:
            """
            Computes the max achievable velocity for an Exponential Profile.
            
            :returns: The seady-state velocity achieved by this profile.
            """
        @property
        def A(self) -> wpimath.units.units_per_second:
            """
            The State-Space 1x1 system matrix.
            """
        @property
        def B(self) -> wpimath.units.meters_per_second_squared_per_volt:
            """
            The State-Space 1x1 input matrix.
            """
        @property
        def maxInput(self) -> wpimath.units.volts:
            """
            Maximum unsigned input voltage.
            """
    class ProfileTiming:
        """
        Profile timing.
        """
        def __init__(self) -> None:
            ...
        def isFinished(self, t: wpimath.units.seconds) -> bool:
            """
            Decides if the profile is finished by time t.
            
            :param t: The time since the beginning of the profile.
            
            :returns: if the profile is finished at time t.
            """
        @property
        def inflectionTime(self) -> wpimath.units.seconds:
            """
            Profile inflection time.
            """
        @property
        def totalTime(self) -> wpimath.units.seconds:
            """
            Total profile time.
            """
    class State:
        """
        Profile state.
        """
        __hash__: typing.ClassVar[None] = None
        def __eq__(self, arg0: ExponentialProfileMeterVolts.State) -> bool:
            ...
        def __init__(self, arg0: wpimath.units.meters, arg1: wpimath.units.meters_per_second) -> None:
            ...
        @property
        def position(self) -> wpimath.units.meters:
            """
            The position at this state.
            """
        @property
        def velocity(self) -> wpimath.units.meters_per_second:
            """
            The velocity at this state.
            """
    def __init__(self, constraints: ExponentialProfileMeterVolts.Constraints) -> None:
        """
        Construct a ExponentialProfile.
        
        :param constraints: The constraints on the profile, like maximum input.
        """
    def calculate(self, t: wpimath.units.seconds, current: ExponentialProfileMeterVolts.State, goal: ExponentialProfileMeterVolts.State) -> ExponentialProfileMeterVolts.State:
        """
        Calculate the correct position and velocity for the profile at a time t
        where the current state is at time t = 0.
        """
    def calculateInflectionPoint(self, current: ExponentialProfileMeterVolts.State, goal: ExponentialProfileMeterVolts.State) -> ExponentialProfileMeterVolts.State:
        """
        Calculate the point after which the fastest way to reach the goal state is
        to apply input in the opposite direction.
        """
    def calculateProfileTiming(self, current: ExponentialProfileMeterVolts.State, goal: ExponentialProfileMeterVolts.State) -> ExponentialProfileMeterVolts.ProfileTiming:
        """
        Calculate the time it will take for this profile to reach the inflection
        point, and the time it will take for this profile to reach the goal state.
        """
    def timeLeftUntil(self, current: ExponentialProfileMeterVolts.State, goal: ExponentialProfileMeterVolts.State) -> wpimath.units.seconds:
        """
        Calculate the time it will take for this profile to reach the goal state.
        """
class Trajectory:
    """
    Represents a time-parameterized trajectory. The trajectory contains of
    various States that represent the pose, curvature, time elapsed, velocity,
    and acceleration at that point.
    """
    class State:
        """
        Represents one point on the trajectory.
        """
        __hash__: typing.ClassVar[None] = None
        curvature: wpimath.units.radians_per_meter
        def __eq__(self, arg0: Trajectory.State) -> bool:
            """
            Checks equality between this State and another object.
            """
        def __init__(self, t: wpimath.units.seconds = 0.0, velocity: wpimath.units.meters_per_second = 0.0, acceleration: wpimath.units.meters_per_second_squared = 0.0, pose: wpimath.geometry._geometry.Pose2d = ..., curvature: wpimath.units.radians_per_meter = 0.0) -> None:
            ...
        def __repr__(self) -> str:
            ...
        def interpolate(self, endValue: Trajectory.State, i: float) -> Trajectory.State:
            """
            Interpolates between two States.
            
            :param endValue: The end value for the interpolation.
            :param i:        The interpolant (fraction).
            
            :returns: The interpolated state.
            """
        @property
        def acceleration(self) -> wpimath.units.meters_per_second_squared:
            """
            The acceleration at that point of the trajectory.
            """
        @acceleration.setter
        def acceleration(self, arg0: wpimath.units.meters_per_second_squared) -> None:
            ...
        @property
        def acceleration_fps(self) -> wpimath.units.feet_per_second_squared:
            ...
        @property
        def pose(self) -> wpimath.geometry._geometry.Pose2d:
            """
            The pose at that point of the trajectory.
            """
        @pose.setter
        def pose(self, arg0: wpimath.geometry._geometry.Pose2d) -> None:
            ...
        @property
        def t(self) -> wpimath.units.seconds:
            """
            The time elapsed since the beginning of the trajectory.
            """
        @t.setter
        def t(self, arg0: wpimath.units.seconds) -> None:
            ...
        @property
        def velocity(self) -> wpimath.units.meters_per_second:
            """
            The speed at that point of the trajectory.
            """
        @velocity.setter
        def velocity(self, arg0: wpimath.units.meters_per_second) -> None:
            ...
        @property
        def velocity_fps(self) -> wpimath.units.feet_per_second:
            ...
    __hash__: typing.ClassVar[None] = None
    def __add__(self, arg0: Trajectory) -> Trajectory:
        """
        Concatenates another trajectory to the current trajectory. The user is
        responsible for making sure that the end pose of this trajectory and the
        start pose of the other trajectory match (if that is the desired behavior).
        
        :param other: The trajectory to concatenate.
        
        :returns: The concatenated trajectory.
        """
    def __eq__(self, arg0: Trajectory) -> bool:
        """
        Checks equality between this Trajectory and another object.
        """
    @typing.overload
    def __init__(self) -> None:
        ...
    @typing.overload
    def __init__(self, states: list[Trajectory.State]) -> None:
        """
        Constructs a trajectory from a vector of states.
        
        @throws std::invalid_argument if the vector of states is empty.
        """
    def initialPose(self) -> wpimath.geometry._geometry.Pose2d:
        """
        Returns the initial pose of the trajectory.
        
        :returns: The initial pose of the trajectory.
        """
    def relativeTo(self, pose: wpimath.geometry._geometry.Pose2d) -> Trajectory:
        """
        Transforms all poses in the trajectory so that they are relative to the
        given pose. This is useful for converting a field-relative trajectory
        into a robot-relative trajectory.
        
        :param pose: The pose that is the origin of the coordinate frame that
                     the current trajectory will be transformed into.
        
        :returns: The transformed trajectory.
        """
    def sample(self, t: wpimath.units.seconds) -> Trajectory.State:
        """
        Sample the trajectory at a point in time.
        
        :param t: The point in time since the beginning of the trajectory to sample.
        
        :returns: The state at that point in time.
                  @throws std::runtime_error if the trajectory has no states.
        """
    def states(self) -> list[Trajectory.State]:
        """
        Return the states of the trajectory.
        
        :returns: The states of the trajectory.
        """
    def totalTime(self) -> wpimath.units.seconds:
        """
        Returns the overall duration of the trajectory.
        
        :returns: The duration of the trajectory.
        """
    def transformBy(self, transform: wpimath.geometry._geometry.Transform2d) -> Trajectory:
        """
        Transforms all poses in the trajectory by the given transform. This is
        useful for converting a robot-relative trajectory into a field-relative
        trajectory. This works with respect to the first pose in the trajectory.
        
        :param transform: The transform to transform the trajectory by.
        
        :returns: The transformed trajectory.
        """
class TrajectoryConfig:
    """
    Represents the configuration for generating a trajectory. This class stores
    the start velocity, end velocity, max velocity, max acceleration, custom
    constraints, and the reversed flag.
    
    The class must be constructed with a max velocity and max acceleration.
    The other parameters (start velocity, end velocity, constraints, reversed)
    have been defaulted to reasonable values (0, 0, {}, false). These values can
    be changed via the SetXXX methods.
    """
    @staticmethod
    def fromFps(maxVelocity: wpimath.units.feet_per_second, maxAcceleration: wpimath.units.feet_per_second_squared) -> TrajectoryConfig:
        ...
    def __init__(self, maxVelocity: wpimath.units.meters_per_second, maxAcceleration: wpimath.units.meters_per_second_squared) -> None:
        """
        Constructs a config object.
        
        :param maxVelocity:     The max velocity of the trajectory.
        :param maxAcceleration: The max acceleration of the trajectory.
        """
    def addConstraint(self, constraint: wpimath._controls._controls.constraint.TrajectoryConstraint) -> None:
        """
        Adds a user-defined constraint to the trajectory.
        
        :param constraint: The user-defined constraint.
        """
    def endVelocity(self) -> wpimath.units.meters_per_second:
        """
        Returns the ending velocity of the trajectory.
        
        :returns: The ending velocity of the trajectory.
        """
    def isReversed(self) -> bool:
        """
        Returns whether the trajectory is reversed or not.
        
        :returns: whether the trajectory is reversed or not.
        """
    def maxAcceleration(self) -> wpimath.units.meters_per_second_squared:
        """
        Returns the maximum acceleration of the trajectory.
        
        :returns: The maximum acceleration of the trajectory.
        """
    def maxVelocity(self) -> wpimath.units.meters_per_second:
        """
        Returns the maximum velocity of the trajectory.
        
        :returns: The maximum velocity of the trajectory.
        """
    def setEndVelocity(self, endVelocity: wpimath.units.meters_per_second) -> None:
        """
        Sets the end velocity of the trajectory.
        
        :param endVelocity: The end velocity of the trajectory.
        """
    @typing.overload
    def setKinematics(self, kinematics: wpimath.kinematics._kinematics.DifferentialDriveKinematics) -> None:
        """
        Adds a differential drive kinematics constraint to ensure that
        no wheel velocity of a differential drive goes above the max velocity.
        
        :param kinematics: The differential drive kinematics.
        """
    @typing.overload
    def setKinematics(self, kinematics: wpimath.kinematics._kinematics.MecanumDriveKinematics) -> None:
        """
        Adds a mecanum drive kinematics constraint to ensure that
        no wheel velocity of a mecanum drive goes above the max velocity.
        
        :param kinematics: The mecanum drive kinematics.
        """
    @typing.overload
    def setKinematics(self, kinematics: wpimath.kinematics._kinematics.SwerveDrive2Kinematics) -> None:
        """
        Adds a swerve drive kinematics constraint to ensure that
        no wheel velocity of a swerve drive goes above the max velocity.
        
        :param kinematics: The swerve drive kinematics.
        """
    @typing.overload
    def setKinematics(self, kinematics: wpimath.kinematics._kinematics.SwerveDrive3Kinematics) -> None:
        """
        Adds a swerve drive kinematics constraint to ensure that
        no wheel velocity of a swerve drive goes above the max velocity.
        
        :param kinematics: The swerve drive kinematics.
        """
    @typing.overload
    def setKinematics(self, kinematics: wpimath.kinematics._kinematics.SwerveDrive4Kinematics) -> None:
        """
        Adds a swerve drive kinematics constraint to ensure that
        no wheel velocity of a swerve drive goes above the max velocity.
        
        :param kinematics: The swerve drive kinematics.
        """
    @typing.overload
    def setKinematics(self, kinematics: wpimath.kinematics._kinematics.SwerveDrive6Kinematics) -> None:
        """
        Adds a swerve drive kinematics constraint to ensure that
        no wheel velocity of a swerve drive goes above the max velocity.
        
        :param kinematics: The swerve drive kinematics.
        """
    def setReversed(self, reversed: bool) -> None:
        """
        Sets the reversed flag of the trajectory.
        
        :param reversed: Whether the trajectory should be reversed or not.
        """
    def setStartVelocity(self, startVelocity: wpimath.units.meters_per_second) -> None:
        """
        Sets the start velocity of the trajectory.
        
        :param startVelocity: The start velocity of the trajectory.
        """
    def startVelocity(self) -> wpimath.units.meters_per_second:
        """
        Returns the starting velocity of the trajectory.
        
        :returns: The starting velocity of the trajectory.
        """
class TrajectoryGenerator:
    """
    Helper class used to generate trajectories with various constraints.
    """
    @staticmethod
    @typing.overload
    def generateTrajectory(initial: wpimath.spline._spline.Spline3.ControlVector, interiorWaypoints: list[wpimath.geometry._geometry.Translation2d], end: wpimath.spline._spline.Spline3.ControlVector, config: TrajectoryConfig) -> Trajectory:
        """
        Generates a trajectory from the given control vectors and config. This
        method uses clamped cubic splines -- a method in which the exterior control
        vectors and interior waypoints are provided. The headings are automatically
        determined at the interior points to ensure continuous curvature.
        
        :param initial:           The initial control vector.
        :param interiorWaypoints: The interior waypoints.
        :param end:               The ending control vector.
        :param config:            The configuration for the trajectory.
        
        :returns: The generated trajectory.
        """
    @staticmethod
    @typing.overload
    def generateTrajectory(start: wpimath.geometry._geometry.Pose2d, interiorWaypoints: list[wpimath.geometry._geometry.Translation2d], end: wpimath.geometry._geometry.Pose2d, config: TrajectoryConfig) -> Trajectory:
        """
        Generates a trajectory from the given waypoints and config. This method
        uses clamped cubic splines -- a method in which the initial pose, final
        pose, and interior waypoints are provided.  The headings are automatically
        determined at the interior points to ensure continuous curvature.
        
        :param start:             The starting pose.
        :param interiorWaypoints: The interior waypoints.
        :param end:               The ending pose.
        :param config:            The configuration for the trajectory.
        
        :returns: The generated trajectory.
        """
    @staticmethod
    @typing.overload
    def generateTrajectory(controlVectors: list[wpimath.spline._spline.Spline5.ControlVector], config: TrajectoryConfig) -> Trajectory:
        """
        Generates a trajectory from the given quintic control vectors and config.
        This method uses quintic hermite splines -- therefore, all points must be
        represented by control vectors. Continuous curvature is guaranteed in this
        method.
        
        :param controlVectors: List of quintic control vectors.
        :param config:         The configuration for the trajectory.
        
        :returns: The generated trajectory.
        """
    @staticmethod
    @typing.overload
    def generateTrajectory(waypoints: list[wpimath.geometry._geometry.Pose2d], config: TrajectoryConfig) -> Trajectory:
        """
        Generates a trajectory from the given waypoints and config. This method
        uses quintic hermite splines -- therefore, all points must be represented
        by Pose2d objects. Continuous curvature is guaranteed in this method.
        
        :param waypoints: List of waypoints..
        :param config:    The configuration for the trajectory.
        
        :returns: The generated trajectory.
        """
    @staticmethod
    def setErrorHandler(func: typing.Callable[[str], None]) -> None:
        """
        Set error reporting function. By default, it is output to stderr.
        
        :param func: Error reporting function.
        """
    @staticmethod
    @typing.overload
    def splinePointsFromSplines(splines: list[wpimath.spline._spline.CubicHermiteSpline]) -> list[tuple[wpimath.geometry._geometry.Pose2d, wpimath.units.radians_per_meter]]:
        """
        Generate spline points from a vector of splines by parameterizing the
        splines.
        
        :param splines: The splines to parameterize.
        
        :returns: The spline points for use in time parameterization of a trajectory.
        """
    @staticmethod
    @typing.overload
    def splinePointsFromSplines(splines: list[wpimath.spline._spline.QuinticHermiteSpline]) -> list[tuple[wpimath.geometry._geometry.Pose2d, wpimath.units.radians_per_meter]]:
        """
        Generate spline points from a vector of splines by parameterizing the
        splines.
        
        :param splines: The splines to parameterize.
        
        :returns: The spline points for use in time parameterization of a trajectory.
        """
    def __init__(self) -> None:
        ...
class TrajectoryParameterizer:
    """
    Class used to parameterize a trajectory by time.
    """
    @staticmethod
    def timeParameterizeTrajectory(points: list[tuple[wpimath.geometry._geometry.Pose2d, wpimath.units.radians_per_meter]], constraints: list[wpimath._controls._controls.constraint.TrajectoryConstraint], startVelocity: wpimath.units.meters_per_second, endVelocity: wpimath.units.meters_per_second, maxVelocity: wpimath.units.meters_per_second, maxAcceleration: wpimath.units.meters_per_second_squared, reversed: bool) -> Trajectory:
        """
        Parameterize the trajectory by time. This is where the velocity profile is
        generated.
        
        The derivation of the algorithm used can be found here:
        <http://www2.informatik.uni-freiburg.de/~lau/students/Sprunk2008.pdf>
        
        :param points:          Reference to the spline points.
        :param constraints:     A vector of various velocity and acceleration
                                constraints.
        :param startVelocity:   The start velocity for the trajectory.
        :param endVelocity:     The end velocity for the trajectory.
        :param maxVelocity:     The max velocity for the trajectory.
        :param maxAcceleration: The max acceleration for the trajectory.
        :param reversed:        Whether the robot should move backwards. Note that the
                                robot will still move from a -> b -> ... -> z as defined in the waypoints.
        
        :returns: The trajectory.
        """
    def __init__(self) -> None:
        ...
class TrajectoryUtil:
    """
    Trajectory utilities.
    """
    @staticmethod
    def deserializeTrajectory(jsonStr: str) -> Trajectory:
        """
        Serializes a Trajectory to PathWeaver-style JSON.
        
        :param jsonStr: the string containing the serialized JSON
        
        :returns: the trajectory represented by the JSON
        """
    @staticmethod
    def fromPathweaverJson(path: str) -> Trajectory:
        """
        Imports a Trajectory from a JSON file exported from PathWeaver.
        
        :param path: The path of the json file to import from.
        
        :returns: The trajectory represented by the file.
        """
    @staticmethod
    def serializeTrajectory(trajectory: Trajectory) -> str:
        """
        Deserializes a Trajectory from JSON exported from PathWeaver.
        
        :param trajectory: the trajectory to export
        
        :returns: the string containing the serialized JSON
        """
    @staticmethod
    def toPathweaverJson(trajectory: Trajectory, path: str) -> None:
        """
        Exports a Trajectory to a PathWeaver-style JSON file.
        
        :param trajectory: the trajectory to export
        :param path:       the path of the file to export to
        """
class TrapezoidProfile:
    """
    A trapezoid-shaped velocity profile.
    
    While this class can be used for a profiled movement from start to finish,
    the intended usage is to filter a reference's dynamics based on trapezoidal
    velocity constraints. To compute the reference obeying this constraint, do
    the following.
    
    Initialization::
    
      constraints = TrapezoidProfile.Constraints(kMaxV, kMaxA)
      previousProfiledReference = initialReference
    
    Run on update::
    
      profile = TrapezoidProfile(constraints, unprofiledReference, previousProfiledReference)
      previousProfiledReference = profile.calculate(timeSincePreviousUpdate)
    
    where ``unprofiledReference`` is free to change between calls. Note that
    when the unprofiled reference is within the constraints,
    :meth:`calculate` returns the unprofiled reference unchanged.
    
    Otherwise, a timer can be started to provide monotonic values for
    ``calculate()`` and to determine when the profile has completed via
    :meth:`isFinished`.
    """
    class Constraints:
        """
        Profile constraints.
        """
        def __init__(self, maxVelocity: wpimath.units.units_per_second, maxAcceleration: wpimath.units.units_per_second_squared) -> None:
            """
            Constructs constraints for a Trapezoid Profile.
            
            :param maxVelocity:     Maximum velocity.
            :param maxAcceleration: Maximum acceleration.
            """
        @property
        def maxAcceleration(self) -> wpimath.units.units_per_second_squared:
            """
            Maximum acceleration.
            """
        @property
        def maxVelocity(self) -> wpimath.units.units_per_second:
            """
            Maximum velocity.
            """
    class State:
        """
        Profile state.
        """
        __hash__: typing.ClassVar[None] = None
        position: float
        velocity: wpimath.units.units_per_second
        def __eq__(self, arg0: TrapezoidProfile.State) -> bool:
            ...
        def __init__(self, position: float = 0, velocity: wpimath.units.units_per_second = 0) -> None:
            ...
        def __repr__(self) -> str:
            ...
    @typing.overload
    def __init__(self, constraints: TrapezoidProfile.Constraints) -> None:
        """
        Construct a TrapezoidProfile.
        
        :param constraints: The constraints on the profile, like maximum velocity.
        """
    @typing.overload
    def __init__(self, constraints: TrapezoidProfile.Constraints, goal: TrapezoidProfile.State, initial: TrapezoidProfile.State = ...) -> None:
        """
        Construct a TrapezoidProfile.
        
        :deprecated: Pass the desired and current state into calculate instead of
                     constructing a new TrapezoidProfile with the desired and current state
        
        :param constraints: The constraints on the profile, like maximum velocity.
        :param goal:        The desired state when the profile is complete.
        :param initial:     The initial state (usually the current state).
        """
    @typing.overload
    def calculate(self, t: wpimath.units.seconds) -> TrapezoidProfile.State:
        """
        Calculate the correct position and velocity for the profile at a time t
        where the beginning of the profile was at time t = 0.
        
        :deprecated: Pass the desired and current state into calculate instead of
                     constructing a new TrapezoidProfile with the desired and current state
        
        :param t: The time since the beginning of the profile.
        """
    @typing.overload
    def calculate(self, t: wpimath.units.seconds, current: TrapezoidProfile.State, goal: TrapezoidProfile.State) -> TrapezoidProfile.State:
        """
        Calculate the correct position and velocity for the profile at a time t
        where the beginning of the profile was at time t = 0.
        
        :param t:       The time since the beginning of the profile.
        :param current: The initial state (usually the current state).
        :param goal:    The desired state when the profile is complete.
        """
    def isFinished(self, t: wpimath.units.seconds) -> bool:
        """
        Returns true if the profile has reached the goal.
        
        The profile has reached the goal if the time since the profile started
        has exceeded the profile's total time.
        
        :param t: The time since the beginning of the profile.
        """
    def timeLeftUntil(self, target: float) -> wpimath.units.seconds:
        """
        Returns the time left until a target distance in the profile is reached.
        
        :param target: The target distance.
        """
    def totalTime(self) -> wpimath.units.seconds:
        """
        Returns the total time the profile takes to reach the goal.
        """
class TrapezoidProfileRadians:
    """
    A trapezoid-shaped velocity profile.
    
    While this class can be used for a profiled movement from start to finish,
    the intended usage is to filter a reference's dynamics based on trapezoidal
    velocity constraints. To compute the reference obeying this constraint, do
    the following.
    
    Initialization::
    
      constraints = TrapezoidProfile.Constraints(kMaxV, kMaxA)
      previousProfiledReference = initialReference
    
    Run on update::
    
      profile = TrapezoidProfile(constraints, unprofiledReference, previousProfiledReference)
      previousProfiledReference = profile.calculate(timeSincePreviousUpdate)
    
    where ``unprofiledReference`` is free to change between calls. Note that
    when the unprofiled reference is within the constraints,
    :meth:`calculate` returns the unprofiled reference unchanged.
    
    Otherwise, a timer can be started to provide monotonic values for
    ``calculate()`` and to determine when the profile has completed via
    :meth:`isFinished`.
    """
    class Constraints:
        """
        Profile constraints.
        """
        def __init__(self, maxVelocity: wpimath.units.radians_per_second, maxAcceleration: wpimath.units.radians_per_second_squared) -> None:
            """
            Constructs constraints for a Trapezoid Profile.
            
            :param maxVelocity:     Maximum velocity.
            :param maxAcceleration: Maximum acceleration.
            """
        @property
        def maxAcceleration(self) -> wpimath.units.radians_per_second_squared:
            """
            Maximum acceleration.
            """
        @property
        def maxVelocity(self) -> wpimath.units.radians_per_second:
            """
            Maximum velocity.
            """
    class State:
        """
        Profile state.
        """
        __hash__: typing.ClassVar[None] = None
        position: wpimath.units.radians
        velocity: wpimath.units.radians_per_second
        def __eq__(self, arg0: TrapezoidProfileRadians.State) -> bool:
            ...
        def __init__(self, position: wpimath.units.radians = 0, velocity: wpimath.units.radians_per_second = 0) -> None:
            ...
        def __repr__(self) -> str:
            ...
    @typing.overload
    def __init__(self, constraints: TrapezoidProfileRadians.Constraints) -> None:
        """
        Construct a TrapezoidProfile.
        
        :param constraints: The constraints on the profile, like maximum velocity.
        """
    @typing.overload
    def __init__(self, constraints: TrapezoidProfileRadians.Constraints, goal: TrapezoidProfileRadians.State, initial: TrapezoidProfileRadians.State = ...) -> None:
        """
        Construct a TrapezoidProfile.
        
        :deprecated: Pass the desired and current state into calculate instead of
                     constructing a new TrapezoidProfile with the desired and current state
        
        :param constraints: The constraints on the profile, like maximum velocity.
        :param goal:        The desired state when the profile is complete.
        :param initial:     The initial state (usually the current state).
        """
    @typing.overload
    def calculate(self, t: wpimath.units.seconds) -> TrapezoidProfileRadians.State:
        """
        Calculate the correct position and velocity for the profile at a time t
        where the beginning of the profile was at time t = 0.
        
        :deprecated: Pass the desired and current state into calculate instead of
                     constructing a new TrapezoidProfile with the desired and current state
        
        :param t: The time since the beginning of the profile.
        """
    @typing.overload
    def calculate(self, t: wpimath.units.seconds, current: TrapezoidProfileRadians.State, goal: TrapezoidProfileRadians.State) -> TrapezoidProfileRadians.State:
        """
        Calculate the correct position and velocity for the profile at a time t
        where the beginning of the profile was at time t = 0.
        
        :param t:       The time since the beginning of the profile.
        :param current: The initial state (usually the current state).
        :param goal:    The desired state when the profile is complete.
        """
    def isFinished(self, t: wpimath.units.seconds) -> bool:
        """
        Returns true if the profile has reached the goal.
        
        The profile has reached the goal if the time since the profile started
        has exceeded the profile's total time.
        
        :param t: The time since the beginning of the profile.
        """
    def timeLeftUntil(self, target: wpimath.units.radians) -> wpimath.units.seconds:
        """
        Returns the time left until a target distance in the profile is reached.
        
        :param target: The target distance.
        """
    def totalTime(self) -> wpimath.units.seconds:
        """
        Returns the total time the profile takes to reach the goal.
        """
