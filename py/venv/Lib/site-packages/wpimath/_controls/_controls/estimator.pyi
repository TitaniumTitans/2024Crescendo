from __future__ import annotations
import numpy
import typing
import wpimath._controls._controls.system
import wpimath.geometry._geometry
import wpimath.kinematics._kinematics
import wpimath.units
__all__ = ['DifferentialDrivePoseEstimator', 'DifferentialDrivePoseEstimatorBase', 'ExtendedKalmanFilter_1_1_1', 'ExtendedKalmanFilter_2_1_1', 'ExtendedKalmanFilter_2_2_2', 'KalmanFilter_1_1_1', 'KalmanFilter_2_1_1', 'KalmanFilter_2_2_2', 'KalmanFilter_3_2_3', 'MecanumDrivePoseEstimator', 'MecanumDrivePoseEstimatorBase', 'SwerveDrive2PoseEstimator', 'SwerveDrive2PoseEstimatorBase', 'SwerveDrive3PoseEstimator', 'SwerveDrive3PoseEstimatorBase', 'SwerveDrive4PoseEstimator', 'SwerveDrive4PoseEstimatorBase', 'SwerveDrive6PoseEstimator', 'SwerveDrive6PoseEstimatorBase']
class DifferentialDrivePoseEstimator(DifferentialDrivePoseEstimatorBase):
    """
    This class wraps an Unscented Kalman Filter to fuse latency-compensated
    vision measurements with differential drive encoder measurements. It will
    correct for noisy vision measurements and encoder drift. It is intended to be
    an easy drop-in for :class:`DifferentialDriveOdometry`. In fact, if you never call
    :meth:`addVisionMeasurement`, and only call :meth:`update`, this will behave exactly the
    same as DifferentialDriveOdometry.
    
    :meth:`update` should be called every robot loop (if your robot loops are faster or
    slower than the default, then you should change the nominal delta time via
    the constructor).
    
    :meth:`addVisionMeasurement` can be called as infrequently as you want; if you
    never call it, then this class will behave like regular encoder odometry.
    
    The state-space system used internally has the following states (x), inputs
    (u), and outputs (y):
    
    :math:`x = [x, y, \theta, dist_l, dist_r]^T` in the field coordinate
    system containing x position, y position, heading, left encoder distance,
    and right encoder distance.
    
    :math:`u = [v_l, v_r, d\theta]^T` containing left wheel velocity,
    right wheel velocity, and change in gyro heading.
    
    NB: Using velocities make things considerably easier, because it means that
    teams don't have to worry about getting an accurate model. Basically, we
    suspect that it's easier for teams to get good encoder data than it is for
    them to perform system identification well enough to get a good model.
    
    :math:`y = [x, y, \theta]^T` from vision containing x position, y
    position, and heading; or :math:`y = [dist_l, dist_r, \theta]^T`
    containing left encoder position, right encoder position, and gyro heading.
    """
    @typing.overload
    def __init__(self, kinematics: wpimath.kinematics._kinematics.DifferentialDriveKinematics, gyroAngle: wpimath.geometry._geometry.Rotation2d, leftDistance: wpimath.units.meters, rightDistance: wpimath.units.meters, initialPose: wpimath.geometry._geometry.Pose2d) -> None:
        """
        Constructs a DifferentialDrivePoseEstimator with default standard
        deviations for the model and vision measurements.
        
        The default standard deviations of the model states are
        0.02 meters for x, 0.02 meters for y, and 0.01 radians for heading.
        The default standard deviations of the vision measurements are
        0.1 meters for x, 0.1 meters for y, and 0.1 radians for heading.
        
        :param kinematics:    A correctly-configured kinematics object for your
                              drivetrain.
        :param gyroAngle:     The gyro angle of the robot.
        :param leftDistance:  The distance traveled by the left encoder.
        :param rightDistance: The distance traveled by the right encoder.
        :param initialPose:   The estimated initial pose.
        """
    @typing.overload
    def __init__(self, kinematics: wpimath.kinematics._kinematics.DifferentialDriveKinematics, gyroAngle: wpimath.geometry._geometry.Rotation2d, leftDistance: wpimath.units.meters, rightDistance: wpimath.units.meters, initialPose: wpimath.geometry._geometry.Pose2d, stateStdDevs: tuple[float, float, float], visionMeasurementStdDevs: tuple[float, float, float]) -> None:
        """
        Constructs a DifferentialDrivePoseEstimator.
        
        :param kinematics:               A correctly-configured kinematics object for your
                                         drivetrain.
        :param gyroAngle:                The gyro angle of the robot.
        :param leftDistance:             The distance traveled by the left encoder.
        :param rightDistance:            The distance traveled by the right encoder.
        :param initialPose:              The estimated initial pose.
        :param stateStdDevs:             Standard deviations of the pose estimate (x position in
                                         meters, y position in meters, and heading in radians). Increase these
                                         numbers to trust your state estimate less.
        :param visionMeasurementStdDevs: Standard deviations of the vision pose
                                         measurement (x position in meters, y position in meters, and heading in
                                         radians). Increase these numbers to trust the vision pose measurement
                                         less.
        """
    def resetPosition(self, gyroAngle: wpimath.geometry._geometry.Rotation2d, leftDistance: wpimath.units.meters, rightDistance: wpimath.units.meters, pose: wpimath.geometry._geometry.Pose2d) -> None:
        """
        Resets the robot's position on the field.
        
        :param gyroAngle:     The current gyro angle.
        :param leftDistance:  The distance traveled by the left encoder.
        :param rightDistance: The distance traveled by the right encoder.
        :param pose:          The estimated pose of the robot on the field.
        """
    def update(self, gyroAngle: wpimath.geometry._geometry.Rotation2d, leftDistance: wpimath.units.meters, rightDistance: wpimath.units.meters) -> wpimath.geometry._geometry.Pose2d:
        """
        Updates the pose estimator with wheel encoder and gyro information. This
        should be called every loop.
        
        :param gyroAngle:     The current gyro angle.
        :param leftDistance:  The distance traveled by the left encoder.
        :param rightDistance: The distance traveled by the right encoder.
        
        :returns: The estimated pose of the robot.
        """
    def updateWithTime(self, currentTime: wpimath.units.seconds, gyroAngle: wpimath.geometry._geometry.Rotation2d, leftDistance: wpimath.units.meters, rightDistance: wpimath.units.meters) -> wpimath.geometry._geometry.Pose2d:
        """
        Updates the pose estimator with wheel encoder and gyro information. This
        should be called every loop.
        
        :param currentTime:   The time at which this method was called.
        :param gyroAngle:     The current gyro angle.
        :param leftDistance:  The distance traveled by the left encoder.
        :param rightDistance: The distance traveled by the right encoder.
        
        :returns: The estimated pose of the robot.
        """
class DifferentialDrivePoseEstimatorBase:
    """
    This class wraps odometry to fuse latency-compensated
    vision measurements with encoder measurements. Robot code should not use this
    directly- Instead, use the particular type for your drivetrain (e.g.,
    DifferentialDrivePoseEstimator). It will correct for noisy vision
    measurements and encoder drift. It is intended to be an easy drop-in for
    Odometry.
    
    Update() should be called every robot loop.
    
    AddVisionMeasurement() can be called as infrequently as you want; if you
    never call it, then this class will behave like regular encoder odometry.
    
    @tparam WheelSpeeds Wheel speeds type.
    @tparam WheelPositions Wheel positions type.
    """
    def __init__(self, kinematics: wpimath.kinematics._kinematics.DifferentialDriveKinematicsBase, odometry: wpimath.kinematics._kinematics.DifferentialDriveOdometryBase, stateStdDevs: tuple[float, float, float], visionMeasurementStdDevs: tuple[float, float, float]) -> None:
        """
        Constructs a PoseEstimator.
        
        :param kinematics:               A correctly-configured kinematics object for your
                                         drivetrain.
        :param odometry:                 A correctly-configured odometry object for your drivetrain.
        :param stateStdDevs:             Standard deviations of the pose estimate (x position in
                                         meters, y position in meters, and heading in radians). Increase these
                                         numbers to trust your state estimate less.
        :param visionMeasurementStdDevs: Standard deviations of the vision pose
                                         measurement (x position in meters, y position in meters, and heading in
                                         radians). Increase these numbers to trust the vision pose measurement
                                         less.
        """
    @typing.overload
    def addVisionMeasurement(self, visionRobotPose: wpimath.geometry._geometry.Pose2d, timestamp: wpimath.units.seconds) -> None:
        """
        Adds a vision measurement to the Kalman Filter. This will correct
        the odometry pose estimate while still accounting for measurement noise.
        
        This method can be called as infrequently as you want, as long as you are
        calling Update() every loop.
        
        To promote stability of the pose estimate and make it robust to bad vision
        data, we recommend only adding vision measurements that are already within
        one meter or so of the current pose estimate.
        
        :param visionRobotPose: The pose of the robot as measured by the vision
                                camera.
        :param timestamp:       The timestamp of the vision measurement in seconds. Note
                                that if you don't use your own time source by calling UpdateWithTime(),
                                then you must use a timestamp with an epoch since FPGA startup (i.e.,
                                the epoch of this timestamp is the same epoch as
                                frc::Timer::GetFPGATimestamp(). This means that you should use
                                frc::Timer::GetFPGATimestamp() as your time source in this case.
        """
    @typing.overload
    def addVisionMeasurement(self, visionRobotPose: wpimath.geometry._geometry.Pose2d, timestamp: wpimath.units.seconds, visionMeasurementStdDevs: tuple[float, float, float]) -> None:
        """
        Adds a vision measurement to the Kalman Filter. This will correct
        the odometry pose estimate while still accounting for measurement noise.
        
        This method can be called as infrequently as you want, as long as you are
        calling Update() every loop.
        
        To promote stability of the pose estimate and make it robust to bad vision
        data, we recommend only adding vision measurements that are already within
        one meter or so of the current pose estimate.
        
        Note that the vision measurement standard deviations passed into this
        method will continue to apply to future measurements until a subsequent
        call to SetVisionMeasurementStdDevs() or this method.
        
        :param visionRobotPose:          The pose of the robot as measured by the vision
                                         camera.
        :param timestamp:                The timestamp of the vision measurement in seconds. Note
                                         that if you don't use your own time source by calling UpdateWithTime(),
                                         then you must use a timestamp with an epoch since FPGA startup (i.e.,
                                         the epoch of this timestamp is the same epoch as
                                         frc::Timer::GetFPGATimestamp(). This means that you should use
                                         frc::Timer::GetFPGATimestamp() as your time source in this case.
        :param visionMeasurementStdDevs: Standard deviations of the vision pose
                                         measurement (x position in meters, y position in meters, and heading in
                                         radians). Increase these numbers to trust the vision pose measurement
                                         less.
        """
    def getEstimatedPosition(self) -> wpimath.geometry._geometry.Pose2d:
        """
        Gets the estimated robot pose.
        
        :returns: The estimated robot pose in meters.
        """
    def resetPosition(self, gyroAngle: wpimath.geometry._geometry.Rotation2d, wheelPositions: wpimath.kinematics._kinematics.DifferentialDriveWheelPositions, pose: wpimath.geometry._geometry.Pose2d) -> None:
        """
        Resets the robot's position on the field.
        
        The gyroscope angle does not need to be reset in the user's robot code.
        The library automatically takes care of offsetting the gyro angle.
        
        :param gyroAngle:      The current gyro angle.
        :param wheelPositions: The distances traveled by the encoders.
        :param pose:           The estimated pose of the robot on the field.
        """
    def setVisionMeasurementStdDevs(self, visionMeasurementStdDevs: tuple[float, float, float]) -> None:
        """
        Sets the pose estimator's trust in vision measurements. This might be used
        to change trust in vision measurements after the autonomous period, or to
        change trust as distance to a vision target increases.
        
        :param visionMeasurementStdDevs: Standard deviations of the vision pose
                                         measurement (x position in meters, y position in meters, and heading in
                                         radians). Increase these numbers to trust the vision pose measurement
                                         less.
        """
    def update(self, gyroAngle: wpimath.geometry._geometry.Rotation2d, wheelPositions: wpimath.kinematics._kinematics.DifferentialDriveWheelPositions) -> wpimath.geometry._geometry.Pose2d:
        """
        Updates the pose estimator with wheel encoder and gyro information. This
        should be called every loop.
        
        :param gyroAngle:      The current gyro angle.
        :param wheelPositions: The distances traveled by the encoders.
        
        :returns: The estimated pose of the robot in meters.
        """
    def updateWithTime(self, currentTime: wpimath.units.seconds, gyroAngle: wpimath.geometry._geometry.Rotation2d, wheelPositions: wpimath.kinematics._kinematics.DifferentialDriveWheelPositions) -> wpimath.geometry._geometry.Pose2d:
        """
        Updates the pose estimator with wheel encoder and gyro information. This
        should be called every loop.
        
        :param currentTime:    The time at which this method was called.
        :param gyroAngle:      The current gyro angle.
        :param wheelPositions: The distances traveled by the encoders.
        
        :returns: The estimated pose of the robot in meters.
        """
class ExtendedKalmanFilter_1_1_1:
    """
    A Kalman filter combines predictions from a model and measurements to give an
    estimate of the true system state. This is useful because many states cannot
    be measured directly as a result of sensor noise, or because the state is
    "hidden".
    
    Kalman filters use a K gain matrix to determine whether to trust the model or
    measurements more. Kalman filter theory uses statistics to compute an optimal
    K gain which minimizes the sum of squares error in the state estimate. This K
    gain is used to correct the state estimate by some amount of the difference
    between the actual measurements and the measurements predicted by the model.
    
    An extended Kalman filter supports nonlinear state and measurement models. It
    propagates the error covariance by linearizing the models around the state
    estimate, then applying the linear Kalman filter equations.
    
    For more on the underlying math, read
    https://file.tavsys.net/control/controls-engineering-in-frc.pdf chapter 9
    "Stochastic control theory".
    
    @tparam States Number of states.
    @tparam Inputs Number of inputs.
    @tparam Outputs Number of outputs.
    """
    @typing.overload
    def P(self) -> numpy.ndarray[numpy.float64[1, 1]]:
        """
        Returns the error covariance matrix P.
        """
    @typing.overload
    def P(self, i: int, j: int) -> float:
        """
        Returns an element of the error covariance matrix P.
        
        :param i: Row of P.
        :param j: Column of P.
        """
    @typing.overload
    def __init__(self, f: typing.Callable[[numpy.ndarray[numpy.float64[1, 1]], numpy.ndarray[numpy.float64[1, 1]]], numpy.ndarray[numpy.float64[1, 1]]], h: typing.Callable[[numpy.ndarray[numpy.float64[1, 1]], numpy.ndarray[numpy.float64[1, 1]]], numpy.ndarray[numpy.float64[1, 1]]], stateStdDevs: tuple[float], measurementStdDevs: tuple[float], dt: wpimath.units.seconds) -> None:
        """
        Constructs an extended Kalman filter.
        
        See
        https://docs.wpilib.org/en/stable/docs/software/advanced-controls/state-space/state-space-observers.html#process-and-measurement-noise-covariance-matrices
        for how to select the standard deviations.
        
        :param f:                  A vector-valued function of x and u that returns
                                   the derivative of the state vector.
        :param h:                  A vector-valued function of x and u that returns
                                   the measurement vector.
        :param stateStdDevs:       Standard deviations of model states.
        :param measurementStdDevs: Standard deviations of measurements.
        :param dt:                 Nominal discretization timestep.
        """
    @typing.overload
    def __init__(self, f: typing.Callable[[numpy.ndarray[numpy.float64[1, 1]], numpy.ndarray[numpy.float64[1, 1]]], numpy.ndarray[numpy.float64[1, 1]]], h: typing.Callable[[numpy.ndarray[numpy.float64[1, 1]], numpy.ndarray[numpy.float64[1, 1]]], numpy.ndarray[numpy.float64[1, 1]]], stateStdDevs: tuple[float], measurementStdDevs: tuple[float], residualFuncY: typing.Callable[[numpy.ndarray[numpy.float64[1, 1]], numpy.ndarray[numpy.float64[1, 1]]], numpy.ndarray[numpy.float64[1, 1]]], addFuncX: typing.Callable[[numpy.ndarray[numpy.float64[1, 1]], numpy.ndarray[numpy.float64[1, 1]]], numpy.ndarray[numpy.float64[1, 1]]], dt: wpimath.units.seconds) -> None:
        """
        Constructs an extended Kalman filter.
        
        See
        https://docs.wpilib.org/en/stable/docs/software/advanced-controls/state-space/state-space-observers.html#process-and-measurement-noise-covariance-matrices
        for how to select the standard deviations.
        
        :param f:                  A vector-valued function of x and u that returns
                                   the derivative of the state vector.
        :param h:                  A vector-valued function of x and u that returns
                                   the measurement vector.
        :param stateStdDevs:       Standard deviations of model states.
        :param measurementStdDevs: Standard deviations of measurements.
        :param residualFuncY:      A function that computes the residual of two
                                   measurement vectors (i.e. it subtracts them.)
        :param addFuncX:           A function that adds two state vectors.
        :param dt:                 Nominal discretization timestep.
        """
    @typing.overload
    def correct(self, u: numpy.ndarray[numpy.float64[1, 1]], y: numpy.ndarray[numpy.float64[1, 1]]) -> None:
        """
        Correct the state estimate x-hat using the measurements in y.
        
        :param u: Same control input used in the predict step.
        :param y: Measurement vector.
        """
    @typing.overload
    def correct(self, u: numpy.ndarray[numpy.float64[1, 1]], y: numpy.ndarray[numpy.float64[1, 1]], R: numpy.ndarray[numpy.float64[1, 1]]) -> None:
        """
        Correct the state estimate x-hat using the measurements in y.
        
        This is useful for when the measurement noise covariances vary.
        
        :param u: Same control input used in the predict step.
        :param y: Measurement vector.
        :param R: Continuous measurement noise covariance matrix.
        """
    def predict(self, u: numpy.ndarray[numpy.float64[1, 1]], dt: wpimath.units.seconds) -> None:
        """
        Project the model into the future with a new control input u.
        
        :param u:  New control input from controller.
        :param dt: Timestep for prediction.
        """
    def reset(self) -> None:
        """
        Resets the observer.
        """
    def setP(self, P: numpy.ndarray[numpy.float64[1, 1]]) -> None:
        """
        Set the current error covariance matrix P.
        
        :param P: The error covariance matrix P.
        """
    @typing.overload
    def setXhat(self, xHat: numpy.ndarray[numpy.float64[1, 1]]) -> None:
        """
        Set initial state estimate x-hat.
        
        :param xHat: The state estimate x-hat.
        """
    @typing.overload
    def setXhat(self, i: int, value: float) -> None:
        """
        Set an element of the initial state estimate x-hat.
        
        :param i:     Row of x-hat.
        :param value: Value for element of x-hat.
        """
    @typing.overload
    def xhat(self) -> numpy.ndarray[numpy.float64[1, 1]]:
        """
        Returns the state estimate x-hat.
        """
    @typing.overload
    def xhat(self, i: int) -> float:
        """
        Returns an element of the state estimate x-hat.
        
        :param i: Row of x-hat.
        """
class ExtendedKalmanFilter_2_1_1:
    """
    A Kalman filter combines predictions from a model and measurements to give an
    estimate of the true system state. This is useful because many states cannot
    be measured directly as a result of sensor noise, or because the state is
    "hidden".
    
    Kalman filters use a K gain matrix to determine whether to trust the model or
    measurements more. Kalman filter theory uses statistics to compute an optimal
    K gain which minimizes the sum of squares error in the state estimate. This K
    gain is used to correct the state estimate by some amount of the difference
    between the actual measurements and the measurements predicted by the model.
    
    An extended Kalman filter supports nonlinear state and measurement models. It
    propagates the error covariance by linearizing the models around the state
    estimate, then applying the linear Kalman filter equations.
    
    For more on the underlying math, read
    https://file.tavsys.net/control/controls-engineering-in-frc.pdf chapter 9
    "Stochastic control theory".
    
    @tparam States Number of states.
    @tparam Inputs Number of inputs.
    @tparam Outputs Number of outputs.
    """
    @typing.overload
    def P(self) -> numpy.ndarray[numpy.float64[2, 2]]:
        """
        Returns the error covariance matrix P.
        """
    @typing.overload
    def P(self, i: int, j: int) -> float:
        """
        Returns an element of the error covariance matrix P.
        
        :param i: Row of P.
        :param j: Column of P.
        """
    @typing.overload
    def __init__(self, f: typing.Callable[[numpy.ndarray[numpy.float64[2, 1]], numpy.ndarray[numpy.float64[1, 1]]], numpy.ndarray[numpy.float64[2, 1]]], h: typing.Callable[[numpy.ndarray[numpy.float64[2, 1]], numpy.ndarray[numpy.float64[1, 1]]], numpy.ndarray[numpy.float64[1, 1]]], stateStdDevs: tuple[float, float], measurementStdDevs: tuple[float], dt: wpimath.units.seconds) -> None:
        """
        Constructs an extended Kalman filter.
        
        See
        https://docs.wpilib.org/en/stable/docs/software/advanced-controls/state-space/state-space-observers.html#process-and-measurement-noise-covariance-matrices
        for how to select the standard deviations.
        
        :param f:                  A vector-valued function of x and u that returns
                                   the derivative of the state vector.
        :param h:                  A vector-valued function of x and u that returns
                                   the measurement vector.
        :param stateStdDevs:       Standard deviations of model states.
        :param measurementStdDevs: Standard deviations of measurements.
        :param dt:                 Nominal discretization timestep.
        """
    @typing.overload
    def __init__(self, f: typing.Callable[[numpy.ndarray[numpy.float64[2, 1]], numpy.ndarray[numpy.float64[1, 1]]], numpy.ndarray[numpy.float64[2, 1]]], h: typing.Callable[[numpy.ndarray[numpy.float64[2, 1]], numpy.ndarray[numpy.float64[1, 1]]], numpy.ndarray[numpy.float64[1, 1]]], stateStdDevs: tuple[float, float], measurementStdDevs: tuple[float], residualFuncY: typing.Callable[[numpy.ndarray[numpy.float64[1, 1]], numpy.ndarray[numpy.float64[1, 1]]], numpy.ndarray[numpy.float64[1, 1]]], addFuncX: typing.Callable[[numpy.ndarray[numpy.float64[2, 1]], numpy.ndarray[numpy.float64[2, 1]]], numpy.ndarray[numpy.float64[2, 1]]], dt: wpimath.units.seconds) -> None:
        """
        Constructs an extended Kalman filter.
        
        See
        https://docs.wpilib.org/en/stable/docs/software/advanced-controls/state-space/state-space-observers.html#process-and-measurement-noise-covariance-matrices
        for how to select the standard deviations.
        
        :param f:                  A vector-valued function of x and u that returns
                                   the derivative of the state vector.
        :param h:                  A vector-valued function of x and u that returns
                                   the measurement vector.
        :param stateStdDevs:       Standard deviations of model states.
        :param measurementStdDevs: Standard deviations of measurements.
        :param residualFuncY:      A function that computes the residual of two
                                   measurement vectors (i.e. it subtracts them.)
        :param addFuncX:           A function that adds two state vectors.
        :param dt:                 Nominal discretization timestep.
        """
    @typing.overload
    def correct(self, u: numpy.ndarray[numpy.float64[1, 1]], y: numpy.ndarray[numpy.float64[1, 1]]) -> None:
        """
        Correct the state estimate x-hat using the measurements in y.
        
        :param u: Same control input used in the predict step.
        :param y: Measurement vector.
        """
    @typing.overload
    def correct(self, u: numpy.ndarray[numpy.float64[1, 1]], y: numpy.ndarray[numpy.float64[1, 1]], R: numpy.ndarray[numpy.float64[1, 1]]) -> None:
        """
        Correct the state estimate x-hat using the measurements in y.
        
        This is useful for when the measurement noise covariances vary.
        
        :param u: Same control input used in the predict step.
        :param y: Measurement vector.
        :param R: Continuous measurement noise covariance matrix.
        """
    def predict(self, u: numpy.ndarray[numpy.float64[1, 1]], dt: wpimath.units.seconds) -> None:
        """
        Project the model into the future with a new control input u.
        
        :param u:  New control input from controller.
        :param dt: Timestep for prediction.
        """
    def reset(self) -> None:
        """
        Resets the observer.
        """
    def setP(self, P: numpy.ndarray[numpy.float64[2, 2]]) -> None:
        """
        Set the current error covariance matrix P.
        
        :param P: The error covariance matrix P.
        """
    @typing.overload
    def setXhat(self, xHat: numpy.ndarray[numpy.float64[2, 1]]) -> None:
        """
        Set initial state estimate x-hat.
        
        :param xHat: The state estimate x-hat.
        """
    @typing.overload
    def setXhat(self, i: int, value: float) -> None:
        """
        Set an element of the initial state estimate x-hat.
        
        :param i:     Row of x-hat.
        :param value: Value for element of x-hat.
        """
    @typing.overload
    def xhat(self) -> numpy.ndarray[numpy.float64[2, 1]]:
        """
        Returns the state estimate x-hat.
        """
    @typing.overload
    def xhat(self, i: int) -> float:
        """
        Returns an element of the state estimate x-hat.
        
        :param i: Row of x-hat.
        """
class ExtendedKalmanFilter_2_2_2:
    """
    A Kalman filter combines predictions from a model and measurements to give an
    estimate of the true system state. This is useful because many states cannot
    be measured directly as a result of sensor noise, or because the state is
    "hidden".
    
    Kalman filters use a K gain matrix to determine whether to trust the model or
    measurements more. Kalman filter theory uses statistics to compute an optimal
    K gain which minimizes the sum of squares error in the state estimate. This K
    gain is used to correct the state estimate by some amount of the difference
    between the actual measurements and the measurements predicted by the model.
    
    An extended Kalman filter supports nonlinear state and measurement models. It
    propagates the error covariance by linearizing the models around the state
    estimate, then applying the linear Kalman filter equations.
    
    For more on the underlying math, read
    https://file.tavsys.net/control/controls-engineering-in-frc.pdf chapter 9
    "Stochastic control theory".
    
    @tparam States Number of states.
    @tparam Inputs Number of inputs.
    @tparam Outputs Number of outputs.
    """
    @typing.overload
    def P(self) -> numpy.ndarray[numpy.float64[2, 2]]:
        """
        Returns the error covariance matrix P.
        """
    @typing.overload
    def P(self, i: int, j: int) -> float:
        """
        Returns an element of the error covariance matrix P.
        
        :param i: Row of P.
        :param j: Column of P.
        """
    @typing.overload
    def __init__(self, f: typing.Callable[[numpy.ndarray[numpy.float64[2, 1]], numpy.ndarray[numpy.float64[2, 1]]], numpy.ndarray[numpy.float64[2, 1]]], h: typing.Callable[[numpy.ndarray[numpy.float64[2, 1]], numpy.ndarray[numpy.float64[2, 1]]], numpy.ndarray[numpy.float64[2, 1]]], stateStdDevs: tuple[float, float], measurementStdDevs: tuple[float, float], dt: wpimath.units.seconds) -> None:
        """
        Constructs an extended Kalman filter.
        
        See
        https://docs.wpilib.org/en/stable/docs/software/advanced-controls/state-space/state-space-observers.html#process-and-measurement-noise-covariance-matrices
        for how to select the standard deviations.
        
        :param f:                  A vector-valued function of x and u that returns
                                   the derivative of the state vector.
        :param h:                  A vector-valued function of x and u that returns
                                   the measurement vector.
        :param stateStdDevs:       Standard deviations of model states.
        :param measurementStdDevs: Standard deviations of measurements.
        :param dt:                 Nominal discretization timestep.
        """
    @typing.overload
    def __init__(self, f: typing.Callable[[numpy.ndarray[numpy.float64[2, 1]], numpy.ndarray[numpy.float64[2, 1]]], numpy.ndarray[numpy.float64[2, 1]]], h: typing.Callable[[numpy.ndarray[numpy.float64[2, 1]], numpy.ndarray[numpy.float64[2, 1]]], numpy.ndarray[numpy.float64[2, 1]]], stateStdDevs: tuple[float, float], measurementStdDevs: tuple[float, float], residualFuncY: typing.Callable[[numpy.ndarray[numpy.float64[2, 1]], numpy.ndarray[numpy.float64[2, 1]]], numpy.ndarray[numpy.float64[2, 1]]], addFuncX: typing.Callable[[numpy.ndarray[numpy.float64[2, 1]], numpy.ndarray[numpy.float64[2, 1]]], numpy.ndarray[numpy.float64[2, 1]]], dt: wpimath.units.seconds) -> None:
        """
        Constructs an extended Kalman filter.
        
        See
        https://docs.wpilib.org/en/stable/docs/software/advanced-controls/state-space/state-space-observers.html#process-and-measurement-noise-covariance-matrices
        for how to select the standard deviations.
        
        :param f:                  A vector-valued function of x and u that returns
                                   the derivative of the state vector.
        :param h:                  A vector-valued function of x and u that returns
                                   the measurement vector.
        :param stateStdDevs:       Standard deviations of model states.
        :param measurementStdDevs: Standard deviations of measurements.
        :param residualFuncY:      A function that computes the residual of two
                                   measurement vectors (i.e. it subtracts them.)
        :param addFuncX:           A function that adds two state vectors.
        :param dt:                 Nominal discretization timestep.
        """
    @typing.overload
    def correct(self, u: numpy.ndarray[numpy.float64[2, 1]], y: numpy.ndarray[numpy.float64[2, 1]]) -> None:
        """
        Correct the state estimate x-hat using the measurements in y.
        
        :param u: Same control input used in the predict step.
        :param y: Measurement vector.
        """
    @typing.overload
    def correct(self, u: numpy.ndarray[numpy.float64[2, 1]], y: numpy.ndarray[numpy.float64[2, 1]], R: numpy.ndarray[numpy.float64[2, 2]]) -> None:
        """
        Correct the state estimate x-hat using the measurements in y.
        
        This is useful for when the measurement noise covariances vary.
        
        :param u: Same control input used in the predict step.
        :param y: Measurement vector.
        :param R: Continuous measurement noise covariance matrix.
        """
    def predict(self, u: numpy.ndarray[numpy.float64[2, 1]], dt: wpimath.units.seconds) -> None:
        """
        Project the model into the future with a new control input u.
        
        :param u:  New control input from controller.
        :param dt: Timestep for prediction.
        """
    def reset(self) -> None:
        """
        Resets the observer.
        """
    def setP(self, P: numpy.ndarray[numpy.float64[2, 2]]) -> None:
        """
        Set the current error covariance matrix P.
        
        :param P: The error covariance matrix P.
        """
    @typing.overload
    def setXhat(self, xHat: numpy.ndarray[numpy.float64[2, 1]]) -> None:
        """
        Set initial state estimate x-hat.
        
        :param xHat: The state estimate x-hat.
        """
    @typing.overload
    def setXhat(self, i: int, value: float) -> None:
        """
        Set an element of the initial state estimate x-hat.
        
        :param i:     Row of x-hat.
        :param value: Value for element of x-hat.
        """
    @typing.overload
    def xhat(self) -> numpy.ndarray[numpy.float64[2, 1]]:
        """
        Returns the state estimate x-hat.
        """
    @typing.overload
    def xhat(self, i: int) -> float:
        """
        Returns an element of the state estimate x-hat.
        
        :param i: Row of x-hat.
        """
class KalmanFilter_1_1_1:
    """
    A Kalman filter combines predictions from a model and measurements to give an
    estimate of the true system state. This is useful because many states cannot
    be measured directly as a result of sensor noise, or because the state is
    "hidden".
    
    Kalman filters use a K gain matrix to determine whether to trust the model or
    measurements more. Kalman filter theory uses statistics to compute an optimal
    K gain which minimizes the sum of squares error in the state estimate. This K
    gain is used to correct the state estimate by some amount of the difference
    between the actual measurements and the measurements predicted by the model.
    
    For more on the underlying math, read
    https://file.tavsys.net/control/controls-engineering-in-frc.pdf chapter 9
    "Stochastic control theory".
    
    @tparam States Number of states.
    @tparam Inputs Number of inputs.
    @tparam Outputs Number of outputs.
    """
    @typing.overload
    def P(self) -> numpy.ndarray[numpy.float64[1, 1]]:
        """
        Returns the error covariance matrix P.
        """
    @typing.overload
    def P(self, i: int, j: int) -> float:
        """
        Returns an element of the error covariance matrix P.
        
        :param i: Row of P.
        :param j: Column of P.
        """
    def __init__(self, plant: wpimath._controls._controls.system.LinearSystem_1_1_1, stateStdDevs: tuple[float], measurementStdDevs: tuple[float], dt: wpimath.units.seconds) -> None:
        """
        Constructs a Kalman filter with the given plant.
        
        See
        https://docs.wpilib.org/en/stable/docs/software/advanced-controls/state-space/state-space-observers.html#process-and-measurement-noise-covariance-matrices
        for how to select the standard deviations.
        
        :param plant:              The plant used for the prediction step.
        :param stateStdDevs:       Standard deviations of model states.
        :param measurementStdDevs: Standard deviations of measurements.
        :param dt:                 Nominal discretization timestep.
                                   @throws std::invalid_argument If the system is unobservable.
        """
    @typing.overload
    def correct(self, u: numpy.ndarray[numpy.float64[1, 1]], y: numpy.ndarray[numpy.float64[1, 1]]) -> None:
        """
        Correct the state estimate x-hat using the measurements in y.
        
        :param u: Same control input used in the predict step.
        :param y: Measurement vector.
        """
    @typing.overload
    def correct(self, u: numpy.ndarray[numpy.float64[1, 1]], y: numpy.ndarray[numpy.float64[1, 1]], R: numpy.ndarray[numpy.float64[1, 1]]) -> None:
        """
        Correct the state estimate x-hat using the measurements in y.
        
        This is useful for when the measurement noise covariances vary.
        
        :param u: Same control input used in the predict step.
        :param y: Measurement vector.
        :param R: Continuous measurement noise covariance matrix.
        """
    def predict(self, u: numpy.ndarray[numpy.float64[1, 1]], dt: wpimath.units.seconds) -> None:
        """
        Project the model into the future with a new control input u.
        
        :param u:  New control input from controller.
        :param dt: Timestep for prediction.
        """
    def reset(self) -> None:
        """
        Resets the observer.
        """
    def setP(self, P: numpy.ndarray[numpy.float64[1, 1]]) -> None:
        """
        Set the current error covariance matrix P.
        
        :param P: The error covariance matrix P.
        """
    @typing.overload
    def setXhat(self, xHat: numpy.ndarray[numpy.float64[1, 1]]) -> None:
        """
        Set initial state estimate x-hat.
        
        :param xHat: The state estimate x-hat.
        """
    @typing.overload
    def setXhat(self, i: int, value: float) -> None:
        """
        Set an element of the initial state estimate x-hat.
        
        :param i:     Row of x-hat.
        :param value: Value for element of x-hat.
        """
    @typing.overload
    def xhat(self) -> numpy.ndarray[numpy.float64[1, 1]]:
        """
        Returns the state estimate x-hat.
        """
    @typing.overload
    def xhat(self, i: int) -> float:
        """
        Returns an element of the state estimate x-hat.
        
        :param i: Row of x-hat.
        """
class KalmanFilter_2_1_1:
    """
    A Kalman filter combines predictions from a model and measurements to give an
    estimate of the true system state. This is useful because many states cannot
    be measured directly as a result of sensor noise, or because the state is
    "hidden".
    
    Kalman filters use a K gain matrix to determine whether to trust the model or
    measurements more. Kalman filter theory uses statistics to compute an optimal
    K gain which minimizes the sum of squares error in the state estimate. This K
    gain is used to correct the state estimate by some amount of the difference
    between the actual measurements and the measurements predicted by the model.
    
    For more on the underlying math, read
    https://file.tavsys.net/control/controls-engineering-in-frc.pdf chapter 9
    "Stochastic control theory".
    
    @tparam States Number of states.
    @tparam Inputs Number of inputs.
    @tparam Outputs Number of outputs.
    """
    @typing.overload
    def P(self) -> numpy.ndarray[numpy.float64[2, 2]]:
        """
        Returns the error covariance matrix P.
        """
    @typing.overload
    def P(self, i: int, j: int) -> float:
        """
        Returns an element of the error covariance matrix P.
        
        :param i: Row of P.
        :param j: Column of P.
        """
    def __init__(self, plant: wpimath._controls._controls.system.LinearSystem_2_1_1, stateStdDevs: tuple[float, float], measurementStdDevs: tuple[float], dt: wpimath.units.seconds) -> None:
        """
        Constructs a Kalman filter with the given plant.
        
        See
        https://docs.wpilib.org/en/stable/docs/software/advanced-controls/state-space/state-space-observers.html#process-and-measurement-noise-covariance-matrices
        for how to select the standard deviations.
        
        :param plant:              The plant used for the prediction step.
        :param stateStdDevs:       Standard deviations of model states.
        :param measurementStdDevs: Standard deviations of measurements.
        :param dt:                 Nominal discretization timestep.
                                   @throws std::invalid_argument If the system is unobservable.
        """
    @typing.overload
    def correct(self, u: numpy.ndarray[numpy.float64[1, 1]], y: numpy.ndarray[numpy.float64[1, 1]]) -> None:
        """
        Correct the state estimate x-hat using the measurements in y.
        
        :param u: Same control input used in the predict step.
        :param y: Measurement vector.
        """
    @typing.overload
    def correct(self, u: numpy.ndarray[numpy.float64[1, 1]], y: numpy.ndarray[numpy.float64[1, 1]], R: numpy.ndarray[numpy.float64[1, 1]]) -> None:
        """
        Correct the state estimate x-hat using the measurements in y.
        
        This is useful for when the measurement noise covariances vary.
        
        :param u: Same control input used in the predict step.
        :param y: Measurement vector.
        :param R: Continuous measurement noise covariance matrix.
        """
    def predict(self, u: numpy.ndarray[numpy.float64[1, 1]], dt: wpimath.units.seconds) -> None:
        """
        Project the model into the future with a new control input u.
        
        :param u:  New control input from controller.
        :param dt: Timestep for prediction.
        """
    def reset(self) -> None:
        """
        Resets the observer.
        """
    def setP(self, P: numpy.ndarray[numpy.float64[2, 2]]) -> None:
        """
        Set the current error covariance matrix P.
        
        :param P: The error covariance matrix P.
        """
    @typing.overload
    def setXhat(self, xHat: numpy.ndarray[numpy.float64[2, 1]]) -> None:
        """
        Set initial state estimate x-hat.
        
        :param xHat: The state estimate x-hat.
        """
    @typing.overload
    def setXhat(self, i: int, value: float) -> None:
        """
        Set an element of the initial state estimate x-hat.
        
        :param i:     Row of x-hat.
        :param value: Value for element of x-hat.
        """
    @typing.overload
    def xhat(self) -> numpy.ndarray[numpy.float64[2, 1]]:
        """
        Returns the state estimate x-hat.
        """
    @typing.overload
    def xhat(self, i: int) -> float:
        """
        Returns an element of the state estimate x-hat.
        
        :param i: Row of x-hat.
        """
class KalmanFilter_2_2_2:
    """
    A Kalman filter combines predictions from a model and measurements to give an
    estimate of the true system state. This is useful because many states cannot
    be measured directly as a result of sensor noise, or because the state is
    "hidden".
    
    Kalman filters use a K gain matrix to determine whether to trust the model or
    measurements more. Kalman filter theory uses statistics to compute an optimal
    K gain which minimizes the sum of squares error in the state estimate. This K
    gain is used to correct the state estimate by some amount of the difference
    between the actual measurements and the measurements predicted by the model.
    
    For more on the underlying math, read
    https://file.tavsys.net/control/controls-engineering-in-frc.pdf chapter 9
    "Stochastic control theory".
    
    @tparam States Number of states.
    @tparam Inputs Number of inputs.
    @tparam Outputs Number of outputs.
    """
    @typing.overload
    def P(self) -> numpy.ndarray[numpy.float64[2, 2]]:
        """
        Returns the error covariance matrix P.
        """
    @typing.overload
    def P(self, i: int, j: int) -> float:
        """
        Returns an element of the error covariance matrix P.
        
        :param i: Row of P.
        :param j: Column of P.
        """
    def __init__(self, plant: wpimath._controls._controls.system.LinearSystem_2_2_2, stateStdDevs: tuple[float, float], measurementStdDevs: tuple[float, float], dt: wpimath.units.seconds) -> None:
        """
        Constructs a Kalman filter with the given plant.
        
        See
        https://docs.wpilib.org/en/stable/docs/software/advanced-controls/state-space/state-space-observers.html#process-and-measurement-noise-covariance-matrices
        for how to select the standard deviations.
        
        :param plant:              The plant used for the prediction step.
        :param stateStdDevs:       Standard deviations of model states.
        :param measurementStdDevs: Standard deviations of measurements.
        :param dt:                 Nominal discretization timestep.
                                   @throws std::invalid_argument If the system is unobservable.
        """
    @typing.overload
    def correct(self, u: numpy.ndarray[numpy.float64[2, 1]], y: numpy.ndarray[numpy.float64[2, 1]]) -> None:
        """
        Correct the state estimate x-hat using the measurements in y.
        
        :param u: Same control input used in the predict step.
        :param y: Measurement vector.
        """
    @typing.overload
    def correct(self, u: numpy.ndarray[numpy.float64[2, 1]], y: numpy.ndarray[numpy.float64[2, 1]], R: numpy.ndarray[numpy.float64[2, 2]]) -> None:
        """
        Correct the state estimate x-hat using the measurements in y.
        
        This is useful for when the measurement noise covariances vary.
        
        :param u: Same control input used in the predict step.
        :param y: Measurement vector.
        :param R: Continuous measurement noise covariance matrix.
        """
    def predict(self, u: numpy.ndarray[numpy.float64[2, 1]], dt: wpimath.units.seconds) -> None:
        """
        Project the model into the future with a new control input u.
        
        :param u:  New control input from controller.
        :param dt: Timestep for prediction.
        """
    def reset(self) -> None:
        """
        Resets the observer.
        """
    def setP(self, P: numpy.ndarray[numpy.float64[2, 2]]) -> None:
        """
        Set the current error covariance matrix P.
        
        :param P: The error covariance matrix P.
        """
    @typing.overload
    def setXhat(self, xHat: numpy.ndarray[numpy.float64[2, 1]]) -> None:
        """
        Set initial state estimate x-hat.
        
        :param xHat: The state estimate x-hat.
        """
    @typing.overload
    def setXhat(self, i: int, value: float) -> None:
        """
        Set an element of the initial state estimate x-hat.
        
        :param i:     Row of x-hat.
        :param value: Value for element of x-hat.
        """
    @typing.overload
    def xhat(self) -> numpy.ndarray[numpy.float64[2, 1]]:
        """
        Returns the state estimate x-hat.
        """
    @typing.overload
    def xhat(self, i: int) -> float:
        """
        Returns an element of the state estimate x-hat.
        
        :param i: Row of x-hat.
        """
class KalmanFilter_3_2_3:
    """
    A Kalman filter combines predictions from a model and measurements to give an
    estimate of the true system state. This is useful because many states cannot
    be measured directly as a result of sensor noise, or because the state is
    "hidden".
    
    Kalman filters use a K gain matrix to determine whether to trust the model or
    measurements more. Kalman filter theory uses statistics to compute an optimal
    K gain which minimizes the sum of squares error in the state estimate. This K
    gain is used to correct the state estimate by some amount of the difference
    between the actual measurements and the measurements predicted by the model.
    
    For more on the underlying math, read
    https://file.tavsys.net/control/controls-engineering-in-frc.pdf chapter 9
    "Stochastic control theory".
    
    @tparam States Number of states.
    @tparam Inputs Number of inputs.
    @tparam Outputs Number of outputs.
    """
    @typing.overload
    def P(self) -> numpy.ndarray[numpy.float64[3, 3]]:
        """
        Returns the error covariance matrix P.
        """
    @typing.overload
    def P(self, i: int, j: int) -> float:
        """
        Returns an element of the error covariance matrix P.
        
        :param i: Row of P.
        :param j: Column of P.
        """
    def __init__(self, plant: wpimath._controls._controls.system.LinearSystem_3_2_3, stateStdDevs: tuple[float, float, float], measurementStdDevs: tuple[float, float, float], dt: wpimath.units.seconds) -> None:
        """
        Constructs a Kalman filter with the given plant.
        
        See
        https://docs.wpilib.org/en/stable/docs/software/advanced-controls/state-space/state-space-observers.html#process-and-measurement-noise-covariance-matrices
        for how to select the standard deviations.
        
        :param plant:              The plant used for the prediction step.
        :param stateStdDevs:       Standard deviations of model states.
        :param measurementStdDevs: Standard deviations of measurements.
        :param dt:                 Nominal discretization timestep.
                                   @throws std::invalid_argument If the system is unobservable.
        """
    @typing.overload
    def correct(self, u: numpy.ndarray[numpy.float64[2, 1]], y: numpy.ndarray[numpy.float64[3, 1]]) -> None:
        """
        Correct the state estimate x-hat using the measurements in y.
        
        :param u: Same control input used in the predict step.
        :param y: Measurement vector.
        """
    @typing.overload
    def correct(self, u: numpy.ndarray[numpy.float64[2, 1]], y: numpy.ndarray[numpy.float64[3, 1]], R: numpy.ndarray[numpy.float64[3, 3]]) -> None:
        """
        Correct the state estimate x-hat using the measurements in y.
        
        This is useful for when the measurement noise covariances vary.
        
        :param u: Same control input used in the predict step.
        :param y: Measurement vector.
        :param R: Continuous measurement noise covariance matrix.
        """
    def predict(self, u: numpy.ndarray[numpy.float64[2, 1]], dt: wpimath.units.seconds) -> None:
        """
        Project the model into the future with a new control input u.
        
        :param u:  New control input from controller.
        :param dt: Timestep for prediction.
        """
    def reset(self) -> None:
        """
        Resets the observer.
        """
    def setP(self, P: numpy.ndarray[numpy.float64[3, 3]]) -> None:
        """
        Set the current error covariance matrix P.
        
        :param P: The error covariance matrix P.
        """
    @typing.overload
    def setXhat(self, xHat: numpy.ndarray[numpy.float64[3, 1]]) -> None:
        """
        Set initial state estimate x-hat.
        
        :param xHat: The state estimate x-hat.
        """
    @typing.overload
    def setXhat(self, i: int, value: float) -> None:
        """
        Set an element of the initial state estimate x-hat.
        
        :param i:     Row of x-hat.
        :param value: Value for element of x-hat.
        """
    @typing.overload
    def xhat(self) -> numpy.ndarray[numpy.float64[3, 1]]:
        """
        Returns the state estimate x-hat.
        """
    @typing.overload
    def xhat(self, i: int) -> float:
        """
        Returns an element of the state estimate x-hat.
        
        :param i: Row of x-hat.
        """
class MecanumDrivePoseEstimator(MecanumDrivePoseEstimatorBase):
    """
    This class wraps an Unscented Kalman Filter to fuse latency-compensated
    vision measurements with mecanum drive encoder velocity measurements. It will
    correct for noisy measurements and encoder drift. It is intended to be an
    easy but more accurate drop-in for :class:`MecanumDriveOdometry`.
    
    :meth:`update` should be called every robot loop. If your loops are faster or
    slower than the default of 0.02s, then you should change the nominal delta
    time by specifying it in the constructor.
    
    :meth:`addVisionMeasurement` can be called as infrequently as you want; if you
    never call it, then this class will behave mostly like regular encoder
    odometry.
    
    The state-space system used internally has the following states (x), inputs
    (u), and outputs (y):
    
    :math:`x = [x, y, \theta]^T` in the field-coordinate system
    containing x position, y position, and heading.
    
    :math:`u = [v_x, v_y, \omega]^T` containing x velocity, y velocity,
    and angular velocity in the field-coordinate system.
    
    :math:`y = [x, y, \theta]^T` from vision containing x position, y
    position, and heading; or :math:`y = [theta]^T` containing gyro
    heading.
    """
    @typing.overload
    def __init__(self, kinematics: wpimath.kinematics._kinematics.MecanumDriveKinematics, gyroAngle: wpimath.geometry._geometry.Rotation2d, wheelPositions: wpimath.kinematics._kinematics.MecanumDriveWheelPositions, initialPose: wpimath.geometry._geometry.Pose2d) -> None:
        """
        Constructs a MecanumDrivePoseEstimator with default standard deviations
        for the model and vision measurements.
        
        The default standard deviations of the model states are
        0.1 meters for x, 0.1 meters for y, and 0.1 radians for heading.
        The default standard deviations of the vision measurements are
        0.45 meters for x, 0.45 meters for y, and 0.45 radians for heading.
        
        :param kinematics:     A correctly-configured kinematics object for your
                               drivetrain.
        :param gyroAngle:      The current gyro angle.
        :param wheelPositions: The distance measured by each wheel.
        :param initialPose:    The starting pose estimate.
        """
    @typing.overload
    def __init__(self, kinematics: wpimath.kinematics._kinematics.MecanumDriveKinematics, gyroAngle: wpimath.geometry._geometry.Rotation2d, wheelPositions: wpimath.kinematics._kinematics.MecanumDriveWheelPositions, initialPose: wpimath.geometry._geometry.Pose2d, stateStdDevs: tuple[float, float, float], visionMeasurementStdDevs: tuple[float, float, float]) -> None:
        """
        Constructs a MecanumDrivePoseEstimator.
        
        :param kinematics:               A correctly-configured kinematics object for your
                                         drivetrain.
        :param gyroAngle:                The current gyro angle.
        :param wheelPositions:           The distance measured by each wheel.
        :param initialPose:              The starting pose estimate.
        :param stateStdDevs:             Standard deviations of the pose estimate (x position in
                                         meters, y position in meters, and heading in radians). Increase these
                                         numbers to trust your state estimate less.
        :param visionMeasurementStdDevs: Standard deviations of the vision pose
                                         measurement (x position in meters, y position in meters, and heading in
                                         radians). Increase these numbers to trust the vision pose measurement
                                         less.
        """
class MecanumDrivePoseEstimatorBase:
    """
    This class wraps odometry to fuse latency-compensated
    vision measurements with encoder measurements. Robot code should not use this
    directly- Instead, use the particular type for your drivetrain (e.g.,
    DifferentialDrivePoseEstimator). It will correct for noisy vision
    measurements and encoder drift. It is intended to be an easy drop-in for
    Odometry.
    
    Update() should be called every robot loop.
    
    AddVisionMeasurement() can be called as infrequently as you want; if you
    never call it, then this class will behave like regular encoder odometry.
    
    @tparam WheelSpeeds Wheel speeds type.
    @tparam WheelPositions Wheel positions type.
    """
    def __init__(self, kinematics: wpimath.kinematics._kinematics.MecanumDriveKinematicsBase, odometry: wpimath.kinematics._kinematics.MecanumDriveOdometryBase, stateStdDevs: tuple[float, float, float], visionMeasurementStdDevs: tuple[float, float, float]) -> None:
        """
        Constructs a PoseEstimator.
        
        :param kinematics:               A correctly-configured kinematics object for your
                                         drivetrain.
        :param odometry:                 A correctly-configured odometry object for your drivetrain.
        :param stateStdDevs:             Standard deviations of the pose estimate (x position in
                                         meters, y position in meters, and heading in radians). Increase these
                                         numbers to trust your state estimate less.
        :param visionMeasurementStdDevs: Standard deviations of the vision pose
                                         measurement (x position in meters, y position in meters, and heading in
                                         radians). Increase these numbers to trust the vision pose measurement
                                         less.
        """
    @typing.overload
    def addVisionMeasurement(self, visionRobotPose: wpimath.geometry._geometry.Pose2d, timestamp: wpimath.units.seconds) -> None:
        """
        Adds a vision measurement to the Kalman Filter. This will correct
        the odometry pose estimate while still accounting for measurement noise.
        
        This method can be called as infrequently as you want, as long as you are
        calling Update() every loop.
        
        To promote stability of the pose estimate and make it robust to bad vision
        data, we recommend only adding vision measurements that are already within
        one meter or so of the current pose estimate.
        
        :param visionRobotPose: The pose of the robot as measured by the vision
                                camera.
        :param timestamp:       The timestamp of the vision measurement in seconds. Note
                                that if you don't use your own time source by calling UpdateWithTime(),
                                then you must use a timestamp with an epoch since FPGA startup (i.e.,
                                the epoch of this timestamp is the same epoch as
                                frc::Timer::GetFPGATimestamp(). This means that you should use
                                frc::Timer::GetFPGATimestamp() as your time source in this case.
        """
    @typing.overload
    def addVisionMeasurement(self, visionRobotPose: wpimath.geometry._geometry.Pose2d, timestamp: wpimath.units.seconds, visionMeasurementStdDevs: tuple[float, float, float]) -> None:
        """
        Adds a vision measurement to the Kalman Filter. This will correct
        the odometry pose estimate while still accounting for measurement noise.
        
        This method can be called as infrequently as you want, as long as you are
        calling Update() every loop.
        
        To promote stability of the pose estimate and make it robust to bad vision
        data, we recommend only adding vision measurements that are already within
        one meter or so of the current pose estimate.
        
        Note that the vision measurement standard deviations passed into this
        method will continue to apply to future measurements until a subsequent
        call to SetVisionMeasurementStdDevs() or this method.
        
        :param visionRobotPose:          The pose of the robot as measured by the vision
                                         camera.
        :param timestamp:                The timestamp of the vision measurement in seconds. Note
                                         that if you don't use your own time source by calling UpdateWithTime(),
                                         then you must use a timestamp with an epoch since FPGA startup (i.e.,
                                         the epoch of this timestamp is the same epoch as
                                         frc::Timer::GetFPGATimestamp(). This means that you should use
                                         frc::Timer::GetFPGATimestamp() as your time source in this case.
        :param visionMeasurementStdDevs: Standard deviations of the vision pose
                                         measurement (x position in meters, y position in meters, and heading in
                                         radians). Increase these numbers to trust the vision pose measurement
                                         less.
        """
    def getEstimatedPosition(self) -> wpimath.geometry._geometry.Pose2d:
        """
        Gets the estimated robot pose.
        
        :returns: The estimated robot pose in meters.
        """
    def resetPosition(self, gyroAngle: wpimath.geometry._geometry.Rotation2d, wheelPositions: wpimath.kinematics._kinematics.MecanumDriveWheelPositions, pose: wpimath.geometry._geometry.Pose2d) -> None:
        """
        Resets the robot's position on the field.
        
        The gyroscope angle does not need to be reset in the user's robot code.
        The library automatically takes care of offsetting the gyro angle.
        
        :param gyroAngle:      The current gyro angle.
        :param wheelPositions: The distances traveled by the encoders.
        :param pose:           The estimated pose of the robot on the field.
        """
    def setVisionMeasurementStdDevs(self, visionMeasurementStdDevs: tuple[float, float, float]) -> None:
        """
        Sets the pose estimator's trust in vision measurements. This might be used
        to change trust in vision measurements after the autonomous period, or to
        change trust as distance to a vision target increases.
        
        :param visionMeasurementStdDevs: Standard deviations of the vision pose
                                         measurement (x position in meters, y position in meters, and heading in
                                         radians). Increase these numbers to trust the vision pose measurement
                                         less.
        """
    def update(self, gyroAngle: wpimath.geometry._geometry.Rotation2d, wheelPositions: wpimath.kinematics._kinematics.MecanumDriveWheelPositions) -> wpimath.geometry._geometry.Pose2d:
        """
        Updates the pose estimator with wheel encoder and gyro information. This
        should be called every loop.
        
        :param gyroAngle:      The current gyro angle.
        :param wheelPositions: The distances traveled by the encoders.
        
        :returns: The estimated pose of the robot in meters.
        """
    def updateWithTime(self, currentTime: wpimath.units.seconds, gyroAngle: wpimath.geometry._geometry.Rotation2d, wheelPositions: wpimath.kinematics._kinematics.MecanumDriveWheelPositions) -> wpimath.geometry._geometry.Pose2d:
        """
        Updates the pose estimator with wheel encoder and gyro information. This
        should be called every loop.
        
        :param currentTime:    The time at which this method was called.
        :param gyroAngle:      The current gyro angle.
        :param wheelPositions: The distances traveled by the encoders.
        
        :returns: The estimated pose of the robot in meters.
        """
class SwerveDrive2PoseEstimator(SwerveDrive2PoseEstimatorBase):
    """
    This class wraps Swerve Drive Odometry to fuse latency-compensated
    vision measurements with swerve drive encoder distance measurements. It is
    intended to be a drop-in for :class:`SwerveDriveOdometry`.
    
    :meth:`update` should be called every robot loop.
    
    :meth:`addVisionMeasurement` can be called as infrequently as you want; if you
    never call it, then this class will behave as regular encoder odometry.
    
    The state-space system used internally has the following states (x) and outputs (y):
    
    :math:`x = [x, y, \theta]^T` in the field-coordinate system
    containing x position, y position, and heading.
    
    :math:`y = [x, y, \theta]^T` from vision containing x position, y
    position, and heading; or :math:`y = [theta]^T` containing gyro
    heading.
    """
    @typing.overload
    def __init__(self, kinematics: wpimath.kinematics._kinematics.SwerveDrive2Kinematics, gyroAngle: wpimath.geometry._geometry.Rotation2d, modulePositions: tuple[wpimath.kinematics._kinematics.SwerveModulePosition, wpimath.kinematics._kinematics.SwerveModulePosition], initialPose: wpimath.geometry._geometry.Pose2d) -> None:
        """
        Constructs a SwerveDrivePoseEstimator with default standard deviations
        for the model and vision measurements.
        
        The default standard deviations of the model states are
        0.1 meters for x, 0.1 meters for y, and 0.1 radians for heading.
        The default standard deviations of the vision measurements are
        0.9 meters for x, 0.9 meters for y, and 0.9 radians for heading.
        
        :param kinematics:      A correctly-configured kinematics object for your
                                drivetrain.
        :param gyroAngle:       The current gyro angle.
        :param modulePositions: The current distance and rotation measurements of
                                the swerve modules.
        :param initialPose:     The starting pose estimate.
        """
    @typing.overload
    def __init__(self, kinematics: wpimath.kinematics._kinematics.SwerveDrive2Kinematics, gyroAngle: wpimath.geometry._geometry.Rotation2d, modulePositions: tuple[wpimath.kinematics._kinematics.SwerveModulePosition, wpimath.kinematics._kinematics.SwerveModulePosition], initialPose: wpimath.geometry._geometry.Pose2d, stateStdDevs: tuple[float, float, float], visionMeasurementStdDevs: tuple[float, float, float]) -> None:
        """
        Constructs a SwerveDrivePoseEstimator.
        
        :param kinematics:               A correctly-configured kinematics object for your
                                         drivetrain.
        :param gyroAngle:                The current gyro angle.
        :param modulePositions:          The current distance and rotation measurements of
                                         the swerve modules.
        :param initialPose:              The starting pose estimate.
        :param stateStdDevs:             Standard deviations of the pose estimate (x position in
                                         meters, y position in meters, and heading in radians). Increase these
                                         numbers to trust your state estimate less.
        :param visionMeasurementStdDevs: Standard deviations of the vision pose
                                         measurement (x position in meters, y position in meters, and heading in
                                         radians). Increase these numbers to trust the vision pose measurement
                                         less.
        """
    def resetPosition(self, gyroAngle: wpimath.geometry._geometry.Rotation2d, modulePositions: tuple[wpimath.kinematics._kinematics.SwerveModulePosition, wpimath.kinematics._kinematics.SwerveModulePosition], pose: wpimath.geometry._geometry.Pose2d) -> None:
        """
        Resets the robot's position on the field.
        
        The gyroscope angle does not need to be reset in the user's robot code.
        The library automatically takes care of offsetting the gyro angle.
        
        :param gyroAngle:       The angle reported by the gyroscope.
        :param modulePositions: The current distance and rotation measurements of
                                the swerve modules.
        :param pose:            The position on the field that your robot is at.
        """
    def update(self, gyroAngle: wpimath.geometry._geometry.Rotation2d, modulePositions: tuple[wpimath.kinematics._kinematics.SwerveModulePosition, wpimath.kinematics._kinematics.SwerveModulePosition]) -> wpimath.geometry._geometry.Pose2d:
        """
        Updates the pose estimator with wheel encoder and gyro information. This
        should be called every loop.
        
        :param gyroAngle:       The current gyro angle.
        :param modulePositions: The current distance and rotation measurements of
                                the swerve modules.
        
        :returns: The estimated robot pose in meters.
        """
    def updateWithTime(self, currentTime: wpimath.units.seconds, gyroAngle: wpimath.geometry._geometry.Rotation2d, modulePositions: tuple[wpimath.kinematics._kinematics.SwerveModulePosition, wpimath.kinematics._kinematics.SwerveModulePosition]) -> wpimath.geometry._geometry.Pose2d:
        """
        Updates the pose estimator with wheel encoder and gyro information. This
        should be called every loop.
        
        :param currentTime:     Time at which this method was called, in seconds.
        :param gyroAngle:       The current gyro angle.
        :param modulePositions: The current distance traveled and rotations of
                                the swerve modules.
        
        :returns: The estimated robot pose in meters.
        """
class SwerveDrive2PoseEstimatorBase:
    """
    This class wraps odometry to fuse latency-compensated
    vision measurements with encoder measurements. Robot code should not use this
    directly- Instead, use the particular type for your drivetrain (e.g.,
    DifferentialDrivePoseEstimator). It will correct for noisy vision
    measurements and encoder drift. It is intended to be an easy drop-in for
    Odometry.
    
    Update() should be called every robot loop.
    
    AddVisionMeasurement() can be called as infrequently as you want; if you
    never call it, then this class will behave like regular encoder odometry.
    
    @tparam WheelSpeeds Wheel speeds type.
    @tparam WheelPositions Wheel positions type.
    """
    def __init__(self, kinematics: wpimath.kinematics._kinematics.SwerveDrive2KinematicsBase, odometry: wpimath.kinematics._kinematics.SwerveDrive2OdometryBase, stateStdDevs: tuple[float, float, float], visionMeasurementStdDevs: tuple[float, float, float]) -> None:
        """
        Constructs a PoseEstimator.
        
        :param kinematics:               A correctly-configured kinematics object for your
                                         drivetrain.
        :param odometry:                 A correctly-configured odometry object for your drivetrain.
        :param stateStdDevs:             Standard deviations of the pose estimate (x position in
                                         meters, y position in meters, and heading in radians). Increase these
                                         numbers to trust your state estimate less.
        :param visionMeasurementStdDevs: Standard deviations of the vision pose
                                         measurement (x position in meters, y position in meters, and heading in
                                         radians). Increase these numbers to trust the vision pose measurement
                                         less.
        """
    @typing.overload
    def addVisionMeasurement(self, visionRobotPose: wpimath.geometry._geometry.Pose2d, timestamp: wpimath.units.seconds) -> None:
        """
        Adds a vision measurement to the Kalman Filter. This will correct
        the odometry pose estimate while still accounting for measurement noise.
        
        This method can be called as infrequently as you want, as long as you are
        calling Update() every loop.
        
        To promote stability of the pose estimate and make it robust to bad vision
        data, we recommend only adding vision measurements that are already within
        one meter or so of the current pose estimate.
        
        :param visionRobotPose: The pose of the robot as measured by the vision
                                camera.
        :param timestamp:       The timestamp of the vision measurement in seconds. Note
                                that if you don't use your own time source by calling UpdateWithTime(),
                                then you must use a timestamp with an epoch since FPGA startup (i.e.,
                                the epoch of this timestamp is the same epoch as
                                frc::Timer::GetFPGATimestamp(). This means that you should use
                                frc::Timer::GetFPGATimestamp() as your time source in this case.
        """
    @typing.overload
    def addVisionMeasurement(self, visionRobotPose: wpimath.geometry._geometry.Pose2d, timestamp: wpimath.units.seconds, visionMeasurementStdDevs: tuple[float, float, float]) -> None:
        """
        Adds a vision measurement to the Kalman Filter. This will correct
        the odometry pose estimate while still accounting for measurement noise.
        
        This method can be called as infrequently as you want, as long as you are
        calling Update() every loop.
        
        To promote stability of the pose estimate and make it robust to bad vision
        data, we recommend only adding vision measurements that are already within
        one meter or so of the current pose estimate.
        
        Note that the vision measurement standard deviations passed into this
        method will continue to apply to future measurements until a subsequent
        call to SetVisionMeasurementStdDevs() or this method.
        
        :param visionRobotPose:          The pose of the robot as measured by the vision
                                         camera.
        :param timestamp:                The timestamp of the vision measurement in seconds. Note
                                         that if you don't use your own time source by calling UpdateWithTime(),
                                         then you must use a timestamp with an epoch since FPGA startup (i.e.,
                                         the epoch of this timestamp is the same epoch as
                                         frc::Timer::GetFPGATimestamp(). This means that you should use
                                         frc::Timer::GetFPGATimestamp() as your time source in this case.
        :param visionMeasurementStdDevs: Standard deviations of the vision pose
                                         measurement (x position in meters, y position in meters, and heading in
                                         radians). Increase these numbers to trust the vision pose measurement
                                         less.
        """
    def getEstimatedPosition(self) -> wpimath.geometry._geometry.Pose2d:
        """
        Gets the estimated robot pose.
        
        :returns: The estimated robot pose in meters.
        """
    def resetPosition(self, gyroAngle: wpimath.geometry._geometry.Rotation2d, wheelPositions: wpimath.kinematics._kinematics.SwerveDrive2WheelPositions, pose: wpimath.geometry._geometry.Pose2d) -> None:
        """
        Resets the robot's position on the field.
        
        The gyroscope angle does not need to be reset in the user's robot code.
        The library automatically takes care of offsetting the gyro angle.
        
        :param gyroAngle:      The current gyro angle.
        :param wheelPositions: The distances traveled by the encoders.
        :param pose:           The estimated pose of the robot on the field.
        """
    def setVisionMeasurementStdDevs(self, visionMeasurementStdDevs: tuple[float, float, float]) -> None:
        """
        Sets the pose estimator's trust in vision measurements. This might be used
        to change trust in vision measurements after the autonomous period, or to
        change trust as distance to a vision target increases.
        
        :param visionMeasurementStdDevs: Standard deviations of the vision pose
                                         measurement (x position in meters, y position in meters, and heading in
                                         radians). Increase these numbers to trust the vision pose measurement
                                         less.
        """
    def update(self, gyroAngle: wpimath.geometry._geometry.Rotation2d, wheelPositions: wpimath.kinematics._kinematics.SwerveDrive2WheelPositions) -> wpimath.geometry._geometry.Pose2d:
        """
        Updates the pose estimator with wheel encoder and gyro information. This
        should be called every loop.
        
        :param gyroAngle:      The current gyro angle.
        :param wheelPositions: The distances traveled by the encoders.
        
        :returns: The estimated pose of the robot in meters.
        """
    def updateWithTime(self, currentTime: wpimath.units.seconds, gyroAngle: wpimath.geometry._geometry.Rotation2d, wheelPositions: wpimath.kinematics._kinematics.SwerveDrive2WheelPositions) -> wpimath.geometry._geometry.Pose2d:
        """
        Updates the pose estimator with wheel encoder and gyro information. This
        should be called every loop.
        
        :param currentTime:    The time at which this method was called.
        :param gyroAngle:      The current gyro angle.
        :param wheelPositions: The distances traveled by the encoders.
        
        :returns: The estimated pose of the robot in meters.
        """
class SwerveDrive3PoseEstimator(SwerveDrive3PoseEstimatorBase):
    """
    This class wraps Swerve Drive Odometry to fuse latency-compensated
    vision measurements with swerve drive encoder distance measurements. It is
    intended to be a drop-in for :class:`SwerveDriveOdometry`.
    
    :meth:`update` should be called every robot loop.
    
    :meth:`addVisionMeasurement` can be called as infrequently as you want; if you
    never call it, then this class will behave as regular encoder odometry.
    
    The state-space system used internally has the following states (x) and outputs (y):
    
    :math:`x = [x, y, \theta]^T` in the field-coordinate system
    containing x position, y position, and heading.
    
    :math:`y = [x, y, \theta]^T` from vision containing x position, y
    position, and heading; or :math:`y = [theta]^T` containing gyro
    heading.
    """
    @typing.overload
    def __init__(self, kinematics: wpimath.kinematics._kinematics.SwerveDrive3Kinematics, gyroAngle: wpimath.geometry._geometry.Rotation2d, modulePositions: tuple[wpimath.kinematics._kinematics.SwerveModulePosition, wpimath.kinematics._kinematics.SwerveModulePosition, wpimath.kinematics._kinematics.SwerveModulePosition], initialPose: wpimath.geometry._geometry.Pose2d) -> None:
        """
        Constructs a SwerveDrivePoseEstimator with default standard deviations
        for the model and vision measurements.
        
        The default standard deviations of the model states are
        0.1 meters for x, 0.1 meters for y, and 0.1 radians for heading.
        The default standard deviations of the vision measurements are
        0.9 meters for x, 0.9 meters for y, and 0.9 radians for heading.
        
        :param kinematics:      A correctly-configured kinematics object for your
                                drivetrain.
        :param gyroAngle:       The current gyro angle.
        :param modulePositions: The current distance and rotation measurements of
                                the swerve modules.
        :param initialPose:     The starting pose estimate.
        """
    @typing.overload
    def __init__(self, kinematics: wpimath.kinematics._kinematics.SwerveDrive3Kinematics, gyroAngle: wpimath.geometry._geometry.Rotation2d, modulePositions: tuple[wpimath.kinematics._kinematics.SwerveModulePosition, wpimath.kinematics._kinematics.SwerveModulePosition, wpimath.kinematics._kinematics.SwerveModulePosition], initialPose: wpimath.geometry._geometry.Pose2d, stateStdDevs: tuple[float, float, float], visionMeasurementStdDevs: tuple[float, float, float]) -> None:
        """
        Constructs a SwerveDrivePoseEstimator.
        
        :param kinematics:               A correctly-configured kinematics object for your
                                         drivetrain.
        :param gyroAngle:                The current gyro angle.
        :param modulePositions:          The current distance and rotation measurements of
                                         the swerve modules.
        :param initialPose:              The starting pose estimate.
        :param stateStdDevs:             Standard deviations of the pose estimate (x position in
                                         meters, y position in meters, and heading in radians). Increase these
                                         numbers to trust your state estimate less.
        :param visionMeasurementStdDevs: Standard deviations of the vision pose
                                         measurement (x position in meters, y position in meters, and heading in
                                         radians). Increase these numbers to trust the vision pose measurement
                                         less.
        """
    def resetPosition(self, gyroAngle: wpimath.geometry._geometry.Rotation2d, modulePositions: tuple[wpimath.kinematics._kinematics.SwerveModulePosition, wpimath.kinematics._kinematics.SwerveModulePosition, wpimath.kinematics._kinematics.SwerveModulePosition], pose: wpimath.geometry._geometry.Pose2d) -> None:
        """
        Resets the robot's position on the field.
        
        The gyroscope angle does not need to be reset in the user's robot code.
        The library automatically takes care of offsetting the gyro angle.
        
        :param gyroAngle:       The angle reported by the gyroscope.
        :param modulePositions: The current distance and rotation measurements of
                                the swerve modules.
        :param pose:            The position on the field that your robot is at.
        """
    def update(self, gyroAngle: wpimath.geometry._geometry.Rotation2d, modulePositions: tuple[wpimath.kinematics._kinematics.SwerveModulePosition, wpimath.kinematics._kinematics.SwerveModulePosition, wpimath.kinematics._kinematics.SwerveModulePosition]) -> wpimath.geometry._geometry.Pose2d:
        """
        Updates the pose estimator with wheel encoder and gyro information. This
        should be called every loop.
        
        :param gyroAngle:       The current gyro angle.
        :param modulePositions: The current distance and rotation measurements of
                                the swerve modules.
        
        :returns: The estimated robot pose in meters.
        """
    def updateWithTime(self, currentTime: wpimath.units.seconds, gyroAngle: wpimath.geometry._geometry.Rotation2d, modulePositions: tuple[wpimath.kinematics._kinematics.SwerveModulePosition, wpimath.kinematics._kinematics.SwerveModulePosition, wpimath.kinematics._kinematics.SwerveModulePosition]) -> wpimath.geometry._geometry.Pose2d:
        """
        Updates the pose estimator with wheel encoder and gyro information. This
        should be called every loop.
        
        :param currentTime:     Time at which this method was called, in seconds.
        :param gyroAngle:       The current gyro angle.
        :param modulePositions: The current distance traveled and rotations of
                                the swerve modules.
        
        :returns: The estimated robot pose in meters.
        """
class SwerveDrive3PoseEstimatorBase:
    """
    This class wraps odometry to fuse latency-compensated
    vision measurements with encoder measurements. Robot code should not use this
    directly- Instead, use the particular type for your drivetrain (e.g.,
    DifferentialDrivePoseEstimator). It will correct for noisy vision
    measurements and encoder drift. It is intended to be an easy drop-in for
    Odometry.
    
    Update() should be called every robot loop.
    
    AddVisionMeasurement() can be called as infrequently as you want; if you
    never call it, then this class will behave like regular encoder odometry.
    
    @tparam WheelSpeeds Wheel speeds type.
    @tparam WheelPositions Wheel positions type.
    """
    def __init__(self, kinematics: wpimath.kinematics._kinematics.SwerveDrive3KinematicsBase, odometry: wpimath.kinematics._kinematics.SwerveDrive3OdometryBase, stateStdDevs: tuple[float, float, float], visionMeasurementStdDevs: tuple[float, float, float]) -> None:
        """
        Constructs a PoseEstimator.
        
        :param kinematics:               A correctly-configured kinematics object for your
                                         drivetrain.
        :param odometry:                 A correctly-configured odometry object for your drivetrain.
        :param stateStdDevs:             Standard deviations of the pose estimate (x position in
                                         meters, y position in meters, and heading in radians). Increase these
                                         numbers to trust your state estimate less.
        :param visionMeasurementStdDevs: Standard deviations of the vision pose
                                         measurement (x position in meters, y position in meters, and heading in
                                         radians). Increase these numbers to trust the vision pose measurement
                                         less.
        """
    @typing.overload
    def addVisionMeasurement(self, visionRobotPose: wpimath.geometry._geometry.Pose2d, timestamp: wpimath.units.seconds) -> None:
        """
        Adds a vision measurement to the Kalman Filter. This will correct
        the odometry pose estimate while still accounting for measurement noise.
        
        This method can be called as infrequently as you want, as long as you are
        calling Update() every loop.
        
        To promote stability of the pose estimate and make it robust to bad vision
        data, we recommend only adding vision measurements that are already within
        one meter or so of the current pose estimate.
        
        :param visionRobotPose: The pose of the robot as measured by the vision
                                camera.
        :param timestamp:       The timestamp of the vision measurement in seconds. Note
                                that if you don't use your own time source by calling UpdateWithTime(),
                                then you must use a timestamp with an epoch since FPGA startup (i.e.,
                                the epoch of this timestamp is the same epoch as
                                frc::Timer::GetFPGATimestamp(). This means that you should use
                                frc::Timer::GetFPGATimestamp() as your time source in this case.
        """
    @typing.overload
    def addVisionMeasurement(self, visionRobotPose: wpimath.geometry._geometry.Pose2d, timestamp: wpimath.units.seconds, visionMeasurementStdDevs: tuple[float, float, float]) -> None:
        """
        Adds a vision measurement to the Kalman Filter. This will correct
        the odometry pose estimate while still accounting for measurement noise.
        
        This method can be called as infrequently as you want, as long as you are
        calling Update() every loop.
        
        To promote stability of the pose estimate and make it robust to bad vision
        data, we recommend only adding vision measurements that are already within
        one meter or so of the current pose estimate.
        
        Note that the vision measurement standard deviations passed into this
        method will continue to apply to future measurements until a subsequent
        call to SetVisionMeasurementStdDevs() or this method.
        
        :param visionRobotPose:          The pose of the robot as measured by the vision
                                         camera.
        :param timestamp:                The timestamp of the vision measurement in seconds. Note
                                         that if you don't use your own time source by calling UpdateWithTime(),
                                         then you must use a timestamp with an epoch since FPGA startup (i.e.,
                                         the epoch of this timestamp is the same epoch as
                                         frc::Timer::GetFPGATimestamp(). This means that you should use
                                         frc::Timer::GetFPGATimestamp() as your time source in this case.
        :param visionMeasurementStdDevs: Standard deviations of the vision pose
                                         measurement (x position in meters, y position in meters, and heading in
                                         radians). Increase these numbers to trust the vision pose measurement
                                         less.
        """
    def getEstimatedPosition(self) -> wpimath.geometry._geometry.Pose2d:
        """
        Gets the estimated robot pose.
        
        :returns: The estimated robot pose in meters.
        """
    def resetPosition(self, gyroAngle: wpimath.geometry._geometry.Rotation2d, wheelPositions: wpimath.kinematics._kinematics.SwerveDrive3WheelPositions, pose: wpimath.geometry._geometry.Pose2d) -> None:
        """
        Resets the robot's position on the field.
        
        The gyroscope angle does not need to be reset in the user's robot code.
        The library automatically takes care of offsetting the gyro angle.
        
        :param gyroAngle:      The current gyro angle.
        :param wheelPositions: The distances traveled by the encoders.
        :param pose:           The estimated pose of the robot on the field.
        """
    def setVisionMeasurementStdDevs(self, visionMeasurementStdDevs: tuple[float, float, float]) -> None:
        """
        Sets the pose estimator's trust in vision measurements. This might be used
        to change trust in vision measurements after the autonomous period, or to
        change trust as distance to a vision target increases.
        
        :param visionMeasurementStdDevs: Standard deviations of the vision pose
                                         measurement (x position in meters, y position in meters, and heading in
                                         radians). Increase these numbers to trust the vision pose measurement
                                         less.
        """
    def update(self, gyroAngle: wpimath.geometry._geometry.Rotation2d, wheelPositions: wpimath.kinematics._kinematics.SwerveDrive3WheelPositions) -> wpimath.geometry._geometry.Pose2d:
        """
        Updates the pose estimator with wheel encoder and gyro information. This
        should be called every loop.
        
        :param gyroAngle:      The current gyro angle.
        :param wheelPositions: The distances traveled by the encoders.
        
        :returns: The estimated pose of the robot in meters.
        """
    def updateWithTime(self, currentTime: wpimath.units.seconds, gyroAngle: wpimath.geometry._geometry.Rotation2d, wheelPositions: wpimath.kinematics._kinematics.SwerveDrive3WheelPositions) -> wpimath.geometry._geometry.Pose2d:
        """
        Updates the pose estimator with wheel encoder and gyro information. This
        should be called every loop.
        
        :param currentTime:    The time at which this method was called.
        :param gyroAngle:      The current gyro angle.
        :param wheelPositions: The distances traveled by the encoders.
        
        :returns: The estimated pose of the robot in meters.
        """
class SwerveDrive4PoseEstimator(SwerveDrive4PoseEstimatorBase):
    """
    This class wraps Swerve Drive Odometry to fuse latency-compensated
    vision measurements with swerve drive encoder distance measurements. It is
    intended to be a drop-in for :class:`SwerveDriveOdometry`.
    
    :meth:`update` should be called every robot loop.
    
    :meth:`addVisionMeasurement` can be called as infrequently as you want; if you
    never call it, then this class will behave as regular encoder odometry.
    
    The state-space system used internally has the following states (x) and outputs (y):
    
    :math:`x = [x, y, \theta]^T` in the field-coordinate system
    containing x position, y position, and heading.
    
    :math:`y = [x, y, \theta]^T` from vision containing x position, y
    position, and heading; or :math:`y = [theta]^T` containing gyro
    heading.
    """
    @typing.overload
    def __init__(self, kinematics: wpimath.kinematics._kinematics.SwerveDrive4Kinematics, gyroAngle: wpimath.geometry._geometry.Rotation2d, modulePositions: tuple[wpimath.kinematics._kinematics.SwerveModulePosition, wpimath.kinematics._kinematics.SwerveModulePosition, wpimath.kinematics._kinematics.SwerveModulePosition, wpimath.kinematics._kinematics.SwerveModulePosition], initialPose: wpimath.geometry._geometry.Pose2d) -> None:
        """
        Constructs a SwerveDrivePoseEstimator with default standard deviations
        for the model and vision measurements.
        
        The default standard deviations of the model states are
        0.1 meters for x, 0.1 meters for y, and 0.1 radians for heading.
        The default standard deviations of the vision measurements are
        0.9 meters for x, 0.9 meters for y, and 0.9 radians for heading.
        
        :param kinematics:      A correctly-configured kinematics object for your
                                drivetrain.
        :param gyroAngle:       The current gyro angle.
        :param modulePositions: The current distance and rotation measurements of
                                the swerve modules.
        :param initialPose:     The starting pose estimate.
        """
    @typing.overload
    def __init__(self, kinematics: wpimath.kinematics._kinematics.SwerveDrive4Kinematics, gyroAngle: wpimath.geometry._geometry.Rotation2d, modulePositions: tuple[wpimath.kinematics._kinematics.SwerveModulePosition, wpimath.kinematics._kinematics.SwerveModulePosition, wpimath.kinematics._kinematics.SwerveModulePosition, wpimath.kinematics._kinematics.SwerveModulePosition], initialPose: wpimath.geometry._geometry.Pose2d, stateStdDevs: tuple[float, float, float], visionMeasurementStdDevs: tuple[float, float, float]) -> None:
        """
        Constructs a SwerveDrivePoseEstimator.
        
        :param kinematics:               A correctly-configured kinematics object for your
                                         drivetrain.
        :param gyroAngle:                The current gyro angle.
        :param modulePositions:          The current distance and rotation measurements of
                                         the swerve modules.
        :param initialPose:              The starting pose estimate.
        :param stateStdDevs:             Standard deviations of the pose estimate (x position in
                                         meters, y position in meters, and heading in radians). Increase these
                                         numbers to trust your state estimate less.
        :param visionMeasurementStdDevs: Standard deviations of the vision pose
                                         measurement (x position in meters, y position in meters, and heading in
                                         radians). Increase these numbers to trust the vision pose measurement
                                         less.
        """
    def resetPosition(self, gyroAngle: wpimath.geometry._geometry.Rotation2d, modulePositions: tuple[wpimath.kinematics._kinematics.SwerveModulePosition, wpimath.kinematics._kinematics.SwerveModulePosition, wpimath.kinematics._kinematics.SwerveModulePosition, wpimath.kinematics._kinematics.SwerveModulePosition], pose: wpimath.geometry._geometry.Pose2d) -> None:
        """
        Resets the robot's position on the field.
        
        The gyroscope angle does not need to be reset in the user's robot code.
        The library automatically takes care of offsetting the gyro angle.
        
        :param gyroAngle:       The angle reported by the gyroscope.
        :param modulePositions: The current distance and rotation measurements of
                                the swerve modules.
        :param pose:            The position on the field that your robot is at.
        """
    def update(self, gyroAngle: wpimath.geometry._geometry.Rotation2d, modulePositions: tuple[wpimath.kinematics._kinematics.SwerveModulePosition, wpimath.kinematics._kinematics.SwerveModulePosition, wpimath.kinematics._kinematics.SwerveModulePosition, wpimath.kinematics._kinematics.SwerveModulePosition]) -> wpimath.geometry._geometry.Pose2d:
        """
        Updates the pose estimator with wheel encoder and gyro information. This
        should be called every loop.
        
        :param gyroAngle:       The current gyro angle.
        :param modulePositions: The current distance and rotation measurements of
                                the swerve modules.
        
        :returns: The estimated robot pose in meters.
        """
    def updateWithTime(self, currentTime: wpimath.units.seconds, gyroAngle: wpimath.geometry._geometry.Rotation2d, modulePositions: tuple[wpimath.kinematics._kinematics.SwerveModulePosition, wpimath.kinematics._kinematics.SwerveModulePosition, wpimath.kinematics._kinematics.SwerveModulePosition, wpimath.kinematics._kinematics.SwerveModulePosition]) -> wpimath.geometry._geometry.Pose2d:
        """
        Updates the pose estimator with wheel encoder and gyro information. This
        should be called every loop.
        
        :param currentTime:     Time at which this method was called, in seconds.
        :param gyroAngle:       The current gyro angle.
        :param modulePositions: The current distance traveled and rotations of
                                the swerve modules.
        
        :returns: The estimated robot pose in meters.
        """
class SwerveDrive4PoseEstimatorBase:
    """
    This class wraps odometry to fuse latency-compensated
    vision measurements with encoder measurements. Robot code should not use this
    directly- Instead, use the particular type for your drivetrain (e.g.,
    DifferentialDrivePoseEstimator). It will correct for noisy vision
    measurements and encoder drift. It is intended to be an easy drop-in for
    Odometry.
    
    Update() should be called every robot loop.
    
    AddVisionMeasurement() can be called as infrequently as you want; if you
    never call it, then this class will behave like regular encoder odometry.
    
    @tparam WheelSpeeds Wheel speeds type.
    @tparam WheelPositions Wheel positions type.
    """
    def __init__(self, kinematics: wpimath.kinematics._kinematics.SwerveDrive4KinematicsBase, odometry: wpimath.kinematics._kinematics.SwerveDrive4OdometryBase, stateStdDevs: tuple[float, float, float], visionMeasurementStdDevs: tuple[float, float, float]) -> None:
        """
        Constructs a PoseEstimator.
        
        :param kinematics:               A correctly-configured kinematics object for your
                                         drivetrain.
        :param odometry:                 A correctly-configured odometry object for your drivetrain.
        :param stateStdDevs:             Standard deviations of the pose estimate (x position in
                                         meters, y position in meters, and heading in radians). Increase these
                                         numbers to trust your state estimate less.
        :param visionMeasurementStdDevs: Standard deviations of the vision pose
                                         measurement (x position in meters, y position in meters, and heading in
                                         radians). Increase these numbers to trust the vision pose measurement
                                         less.
        """
    @typing.overload
    def addVisionMeasurement(self, visionRobotPose: wpimath.geometry._geometry.Pose2d, timestamp: wpimath.units.seconds) -> None:
        """
        Adds a vision measurement to the Kalman Filter. This will correct
        the odometry pose estimate while still accounting for measurement noise.
        
        This method can be called as infrequently as you want, as long as you are
        calling Update() every loop.
        
        To promote stability of the pose estimate and make it robust to bad vision
        data, we recommend only adding vision measurements that are already within
        one meter or so of the current pose estimate.
        
        :param visionRobotPose: The pose of the robot as measured by the vision
                                camera.
        :param timestamp:       The timestamp of the vision measurement in seconds. Note
                                that if you don't use your own time source by calling UpdateWithTime(),
                                then you must use a timestamp with an epoch since FPGA startup (i.e.,
                                the epoch of this timestamp is the same epoch as
                                frc::Timer::GetFPGATimestamp(). This means that you should use
                                frc::Timer::GetFPGATimestamp() as your time source in this case.
        """
    @typing.overload
    def addVisionMeasurement(self, visionRobotPose: wpimath.geometry._geometry.Pose2d, timestamp: wpimath.units.seconds, visionMeasurementStdDevs: tuple[float, float, float]) -> None:
        """
        Adds a vision measurement to the Kalman Filter. This will correct
        the odometry pose estimate while still accounting for measurement noise.
        
        This method can be called as infrequently as you want, as long as you are
        calling Update() every loop.
        
        To promote stability of the pose estimate and make it robust to bad vision
        data, we recommend only adding vision measurements that are already within
        one meter or so of the current pose estimate.
        
        Note that the vision measurement standard deviations passed into this
        method will continue to apply to future measurements until a subsequent
        call to SetVisionMeasurementStdDevs() or this method.
        
        :param visionRobotPose:          The pose of the robot as measured by the vision
                                         camera.
        :param timestamp:                The timestamp of the vision measurement in seconds. Note
                                         that if you don't use your own time source by calling UpdateWithTime(),
                                         then you must use a timestamp with an epoch since FPGA startup (i.e.,
                                         the epoch of this timestamp is the same epoch as
                                         frc::Timer::GetFPGATimestamp(). This means that you should use
                                         frc::Timer::GetFPGATimestamp() as your time source in this case.
        :param visionMeasurementStdDevs: Standard deviations of the vision pose
                                         measurement (x position in meters, y position in meters, and heading in
                                         radians). Increase these numbers to trust the vision pose measurement
                                         less.
        """
    def getEstimatedPosition(self) -> wpimath.geometry._geometry.Pose2d:
        """
        Gets the estimated robot pose.
        
        :returns: The estimated robot pose in meters.
        """
    def resetPosition(self, gyroAngle: wpimath.geometry._geometry.Rotation2d, wheelPositions: wpimath.kinematics._kinematics.SwerveDrive4WheelPositions, pose: wpimath.geometry._geometry.Pose2d) -> None:
        """
        Resets the robot's position on the field.
        
        The gyroscope angle does not need to be reset in the user's robot code.
        The library automatically takes care of offsetting the gyro angle.
        
        :param gyroAngle:      The current gyro angle.
        :param wheelPositions: The distances traveled by the encoders.
        :param pose:           The estimated pose of the robot on the field.
        """
    def setVisionMeasurementStdDevs(self, visionMeasurementStdDevs: tuple[float, float, float]) -> None:
        """
        Sets the pose estimator's trust in vision measurements. This might be used
        to change trust in vision measurements after the autonomous period, or to
        change trust as distance to a vision target increases.
        
        :param visionMeasurementStdDevs: Standard deviations of the vision pose
                                         measurement (x position in meters, y position in meters, and heading in
                                         radians). Increase these numbers to trust the vision pose measurement
                                         less.
        """
    def update(self, gyroAngle: wpimath.geometry._geometry.Rotation2d, wheelPositions: wpimath.kinematics._kinematics.SwerveDrive4WheelPositions) -> wpimath.geometry._geometry.Pose2d:
        """
        Updates the pose estimator with wheel encoder and gyro information. This
        should be called every loop.
        
        :param gyroAngle:      The current gyro angle.
        :param wheelPositions: The distances traveled by the encoders.
        
        :returns: The estimated pose of the robot in meters.
        """
    def updateWithTime(self, currentTime: wpimath.units.seconds, gyroAngle: wpimath.geometry._geometry.Rotation2d, wheelPositions: wpimath.kinematics._kinematics.SwerveDrive4WheelPositions) -> wpimath.geometry._geometry.Pose2d:
        """
        Updates the pose estimator with wheel encoder and gyro information. This
        should be called every loop.
        
        :param currentTime:    The time at which this method was called.
        :param gyroAngle:      The current gyro angle.
        :param wheelPositions: The distances traveled by the encoders.
        
        :returns: The estimated pose of the robot in meters.
        """
class SwerveDrive6PoseEstimator(SwerveDrive6PoseEstimatorBase):
    """
    This class wraps Swerve Drive Odometry to fuse latency-compensated
    vision measurements with swerve drive encoder distance measurements. It is
    intended to be a drop-in for :class:`SwerveDriveOdometry`.
    
    :meth:`update` should be called every robot loop.
    
    :meth:`addVisionMeasurement` can be called as infrequently as you want; if you
    never call it, then this class will behave as regular encoder odometry.
    
    The state-space system used internally has the following states (x) and outputs (y):
    
    :math:`x = [x, y, \theta]^T` in the field-coordinate system
    containing x position, y position, and heading.
    
    :math:`y = [x, y, \theta]^T` from vision containing x position, y
    position, and heading; or :math:`y = [theta]^T` containing gyro
    heading.
    """
    @typing.overload
    def __init__(self, kinematics: wpimath.kinematics._kinematics.SwerveDrive6Kinematics, gyroAngle: wpimath.geometry._geometry.Rotation2d, modulePositions: tuple[wpimath.kinematics._kinematics.SwerveModulePosition, wpimath.kinematics._kinematics.SwerveModulePosition, wpimath.kinematics._kinematics.SwerveModulePosition, wpimath.kinematics._kinematics.SwerveModulePosition, wpimath.kinematics._kinematics.SwerveModulePosition, wpimath.kinematics._kinematics.SwerveModulePosition], initialPose: wpimath.geometry._geometry.Pose2d) -> None:
        """
        Constructs a SwerveDrivePoseEstimator with default standard deviations
        for the model and vision measurements.
        
        The default standard deviations of the model states are
        0.1 meters for x, 0.1 meters for y, and 0.1 radians for heading.
        The default standard deviations of the vision measurements are
        0.9 meters for x, 0.9 meters for y, and 0.9 radians for heading.
        
        :param kinematics:      A correctly-configured kinematics object for your
                                drivetrain.
        :param gyroAngle:       The current gyro angle.
        :param modulePositions: The current distance and rotation measurements of
                                the swerve modules.
        :param initialPose:     The starting pose estimate.
        """
    @typing.overload
    def __init__(self, kinematics: wpimath.kinematics._kinematics.SwerveDrive6Kinematics, gyroAngle: wpimath.geometry._geometry.Rotation2d, modulePositions: tuple[wpimath.kinematics._kinematics.SwerveModulePosition, wpimath.kinematics._kinematics.SwerveModulePosition, wpimath.kinematics._kinematics.SwerveModulePosition, wpimath.kinematics._kinematics.SwerveModulePosition, wpimath.kinematics._kinematics.SwerveModulePosition, wpimath.kinematics._kinematics.SwerveModulePosition], initialPose: wpimath.geometry._geometry.Pose2d, stateStdDevs: tuple[float, float, float], visionMeasurementStdDevs: tuple[float, float, float]) -> None:
        """
        Constructs a SwerveDrivePoseEstimator.
        
        :param kinematics:               A correctly-configured kinematics object for your
                                         drivetrain.
        :param gyroAngle:                The current gyro angle.
        :param modulePositions:          The current distance and rotation measurements of
                                         the swerve modules.
        :param initialPose:              The starting pose estimate.
        :param stateStdDevs:             Standard deviations of the pose estimate (x position in
                                         meters, y position in meters, and heading in radians). Increase these
                                         numbers to trust your state estimate less.
        :param visionMeasurementStdDevs: Standard deviations of the vision pose
                                         measurement (x position in meters, y position in meters, and heading in
                                         radians). Increase these numbers to trust the vision pose measurement
                                         less.
        """
    def resetPosition(self, gyroAngle: wpimath.geometry._geometry.Rotation2d, modulePositions: tuple[wpimath.kinematics._kinematics.SwerveModulePosition, wpimath.kinematics._kinematics.SwerveModulePosition, wpimath.kinematics._kinematics.SwerveModulePosition, wpimath.kinematics._kinematics.SwerveModulePosition, wpimath.kinematics._kinematics.SwerveModulePosition, wpimath.kinematics._kinematics.SwerveModulePosition], pose: wpimath.geometry._geometry.Pose2d) -> None:
        """
        Resets the robot's position on the field.
        
        The gyroscope angle does not need to be reset in the user's robot code.
        The library automatically takes care of offsetting the gyro angle.
        
        :param gyroAngle:       The angle reported by the gyroscope.
        :param modulePositions: The current distance and rotation measurements of
                                the swerve modules.
        :param pose:            The position on the field that your robot is at.
        """
    def update(self, gyroAngle: wpimath.geometry._geometry.Rotation2d, modulePositions: tuple[wpimath.kinematics._kinematics.SwerveModulePosition, wpimath.kinematics._kinematics.SwerveModulePosition, wpimath.kinematics._kinematics.SwerveModulePosition, wpimath.kinematics._kinematics.SwerveModulePosition, wpimath.kinematics._kinematics.SwerveModulePosition, wpimath.kinematics._kinematics.SwerveModulePosition]) -> wpimath.geometry._geometry.Pose2d:
        """
        Updates the pose estimator with wheel encoder and gyro information. This
        should be called every loop.
        
        :param gyroAngle:       The current gyro angle.
        :param modulePositions: The current distance and rotation measurements of
                                the swerve modules.
        
        :returns: The estimated robot pose in meters.
        """
    def updateWithTime(self, currentTime: wpimath.units.seconds, gyroAngle: wpimath.geometry._geometry.Rotation2d, modulePositions: tuple[wpimath.kinematics._kinematics.SwerveModulePosition, wpimath.kinematics._kinematics.SwerveModulePosition, wpimath.kinematics._kinematics.SwerveModulePosition, wpimath.kinematics._kinematics.SwerveModulePosition, wpimath.kinematics._kinematics.SwerveModulePosition, wpimath.kinematics._kinematics.SwerveModulePosition]) -> wpimath.geometry._geometry.Pose2d:
        """
        Updates the pose estimator with wheel encoder and gyro information. This
        should be called every loop.
        
        :param currentTime:     Time at which this method was called, in seconds.
        :param gyroAngle:       The current gyro angle.
        :param modulePositions: The current distance traveled and rotations of
                                the swerve modules.
        
        :returns: The estimated robot pose in meters.
        """
class SwerveDrive6PoseEstimatorBase:
    """
    This class wraps odometry to fuse latency-compensated
    vision measurements with encoder measurements. Robot code should not use this
    directly- Instead, use the particular type for your drivetrain (e.g.,
    DifferentialDrivePoseEstimator). It will correct for noisy vision
    measurements and encoder drift. It is intended to be an easy drop-in for
    Odometry.
    
    Update() should be called every robot loop.
    
    AddVisionMeasurement() can be called as infrequently as you want; if you
    never call it, then this class will behave like regular encoder odometry.
    
    @tparam WheelSpeeds Wheel speeds type.
    @tparam WheelPositions Wheel positions type.
    """
    def __init__(self, kinematics: wpimath.kinematics._kinematics.SwerveDrive6KinematicsBase, odometry: wpimath.kinematics._kinematics.SwerveDrive6OdometryBase, stateStdDevs: tuple[float, float, float], visionMeasurementStdDevs: tuple[float, float, float]) -> None:
        """
        Constructs a PoseEstimator.
        
        :param kinematics:               A correctly-configured kinematics object for your
                                         drivetrain.
        :param odometry:                 A correctly-configured odometry object for your drivetrain.
        :param stateStdDevs:             Standard deviations of the pose estimate (x position in
                                         meters, y position in meters, and heading in radians). Increase these
                                         numbers to trust your state estimate less.
        :param visionMeasurementStdDevs: Standard deviations of the vision pose
                                         measurement (x position in meters, y position in meters, and heading in
                                         radians). Increase these numbers to trust the vision pose measurement
                                         less.
        """
    @typing.overload
    def addVisionMeasurement(self, visionRobotPose: wpimath.geometry._geometry.Pose2d, timestamp: wpimath.units.seconds) -> None:
        """
        Adds a vision measurement to the Kalman Filter. This will correct
        the odometry pose estimate while still accounting for measurement noise.
        
        This method can be called as infrequently as you want, as long as you are
        calling Update() every loop.
        
        To promote stability of the pose estimate and make it robust to bad vision
        data, we recommend only adding vision measurements that are already within
        one meter or so of the current pose estimate.
        
        :param visionRobotPose: The pose of the robot as measured by the vision
                                camera.
        :param timestamp:       The timestamp of the vision measurement in seconds. Note
                                that if you don't use your own time source by calling UpdateWithTime(),
                                then you must use a timestamp with an epoch since FPGA startup (i.e.,
                                the epoch of this timestamp is the same epoch as
                                frc::Timer::GetFPGATimestamp(). This means that you should use
                                frc::Timer::GetFPGATimestamp() as your time source in this case.
        """
    @typing.overload
    def addVisionMeasurement(self, visionRobotPose: wpimath.geometry._geometry.Pose2d, timestamp: wpimath.units.seconds, visionMeasurementStdDevs: tuple[float, float, float]) -> None:
        """
        Adds a vision measurement to the Kalman Filter. This will correct
        the odometry pose estimate while still accounting for measurement noise.
        
        This method can be called as infrequently as you want, as long as you are
        calling Update() every loop.
        
        To promote stability of the pose estimate and make it robust to bad vision
        data, we recommend only adding vision measurements that are already within
        one meter or so of the current pose estimate.
        
        Note that the vision measurement standard deviations passed into this
        method will continue to apply to future measurements until a subsequent
        call to SetVisionMeasurementStdDevs() or this method.
        
        :param visionRobotPose:          The pose of the robot as measured by the vision
                                         camera.
        :param timestamp:                The timestamp of the vision measurement in seconds. Note
                                         that if you don't use your own time source by calling UpdateWithTime(),
                                         then you must use a timestamp with an epoch since FPGA startup (i.e.,
                                         the epoch of this timestamp is the same epoch as
                                         frc::Timer::GetFPGATimestamp(). This means that you should use
                                         frc::Timer::GetFPGATimestamp() as your time source in this case.
        :param visionMeasurementStdDevs: Standard deviations of the vision pose
                                         measurement (x position in meters, y position in meters, and heading in
                                         radians). Increase these numbers to trust the vision pose measurement
                                         less.
        """
    def getEstimatedPosition(self) -> wpimath.geometry._geometry.Pose2d:
        """
        Gets the estimated robot pose.
        
        :returns: The estimated robot pose in meters.
        """
    def resetPosition(self, gyroAngle: wpimath.geometry._geometry.Rotation2d, wheelPositions: wpimath.kinematics._kinematics.SwerveDrive6WheelPositions, pose: wpimath.geometry._geometry.Pose2d) -> None:
        """
        Resets the robot's position on the field.
        
        The gyroscope angle does not need to be reset in the user's robot code.
        The library automatically takes care of offsetting the gyro angle.
        
        :param gyroAngle:      The current gyro angle.
        :param wheelPositions: The distances traveled by the encoders.
        :param pose:           The estimated pose of the robot on the field.
        """
    def setVisionMeasurementStdDevs(self, visionMeasurementStdDevs: tuple[float, float, float]) -> None:
        """
        Sets the pose estimator's trust in vision measurements. This might be used
        to change trust in vision measurements after the autonomous period, or to
        change trust as distance to a vision target increases.
        
        :param visionMeasurementStdDevs: Standard deviations of the vision pose
                                         measurement (x position in meters, y position in meters, and heading in
                                         radians). Increase these numbers to trust the vision pose measurement
                                         less.
        """
    def update(self, gyroAngle: wpimath.geometry._geometry.Rotation2d, wheelPositions: wpimath.kinematics._kinematics.SwerveDrive6WheelPositions) -> wpimath.geometry._geometry.Pose2d:
        """
        Updates the pose estimator with wheel encoder and gyro information. This
        should be called every loop.
        
        :param gyroAngle:      The current gyro angle.
        :param wheelPositions: The distances traveled by the encoders.
        
        :returns: The estimated pose of the robot in meters.
        """
    def updateWithTime(self, currentTime: wpimath.units.seconds, gyroAngle: wpimath.geometry._geometry.Rotation2d, wheelPositions: wpimath.kinematics._kinematics.SwerveDrive6WheelPositions) -> wpimath.geometry._geometry.Pose2d:
        """
        Updates the pose estimator with wheel encoder and gyro information. This
        should be called every loop.
        
        :param currentTime:    The time at which this method was called.
        :param gyroAngle:      The current gyro angle.
        :param wheelPositions: The distances traveled by the encoders.
        
        :returns: The estimated pose of the robot in meters.
        """
