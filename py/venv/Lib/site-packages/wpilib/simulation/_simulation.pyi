from __future__ import annotations
import hal._wpiHal
import numpy
import pybind11_stubgen.typing_ext
import typing
import wpilib._wpilib
import wpilib.interfaces._interfaces
import wpimath._controls._controls.plant
import wpimath._controls._controls.system
import wpimath.geometry._geometry
import wpimath.units
__all__ = ['ADIS16448_IMUSim', 'ADIS16470_IMUSim', 'ADXL345Sim', 'ADXL362Sim', 'ADXRS450_GyroSim', 'AddressableLEDSim', 'AnalogEncoderSim', 'AnalogGyroSim', 'AnalogInputSim', 'AnalogOutputSim', 'AnalogTriggerSim', 'BatterySim', 'BuiltInAccelerometerSim', 'CTREPCMSim', 'CallbackStore', 'DCMotorSim', 'DIOSim', 'DifferentialDrivetrainSim', 'DigitalPWMSim', 'DoubleSolenoidSim', 'DriverStationSim', 'DutyCycleEncoderSim', 'DutyCycleSim', 'ElevatorSim', 'EncoderSim', 'FlywheelSim', 'GenericHIDSim', 'JoystickSim', 'LinearSystemSim_1_1_1', 'LinearSystemSim_1_1_2', 'LinearSystemSim_2_1_1', 'LinearSystemSim_2_1_2', 'LinearSystemSim_2_2_1', 'LinearSystemSim_2_2_2', 'PS4ControllerSim', 'PS5ControllerSim', 'PWMSim', 'PneumaticsBaseSim', 'PowerDistributionSim', 'REVPHSim', 'RelaySim', 'RoboRioSim', 'SPIAccelerometerSim', 'SimDeviceSim', 'SingleJointedArmSim', 'SolenoidSim', 'UltrasonicSim', 'XboxControllerSim', 'getProgramStarted', 'isTimingPaused', 'pauseTiming', 'restartTiming', 'resumeTiming', 'setProgramStarted', 'setRuntimeType', 'stepTiming', 'stepTimingAsync', 'waitForProgramStart']
class ADIS16448_IMUSim:
    """
    Class to control a simulated ADIS16448 IMU.
    """
    def __init__(self, imu: wpilib._wpilib.ADIS16448_IMU) -> None:
        """
        Constructs from a ADIS16448_IMU object.
        
        :param imu: ADIS16448_IMU to simulate
        """
    def setAccelX(self, accel: wpimath.units.meters_per_second_squared) -> None:
        """
        Sets the X axis acceleration.
        
        :param accel: The acceleration.
        """
    def setAccelY(self, accel: wpimath.units.meters_per_second_squared) -> None:
        """
        Sets the Y axis acceleration.
        
        :param accel: The acceleration.
        """
    def setAccelZ(self, accel: wpimath.units.meters_per_second_squared) -> None:
        """
        Sets the Z axis acceleration.
        
        :param accel: The acceleration.
        """
    def setGyroAngleX(self, angle: wpimath.units.degrees) -> None:
        """
        Sets the X axis angle (CCW positive).
        
        :param angle: The angle.
        """
    def setGyroAngleY(self, angle: wpimath.units.degrees) -> None:
        """
        Sets the Y axis angle (CCW positive).
        
        :param angle: The angle.
        """
    def setGyroAngleZ(self, angle: wpimath.units.degrees) -> None:
        """
        Sets the Z axis angle (CCW positive).
        
        :param angle: The angle.
        """
    def setGyroRateX(self, angularRate: wpimath.units.degrees_per_second) -> None:
        """
        Sets the X axis angular rate (CCW positive).
        
        :param angularRate: The angular rate.
        """
    def setGyroRateY(self, angularRate: wpimath.units.degrees_per_second) -> None:
        """
        Sets the Y axis angular rate (CCW positive).
        
        :param angularRate: The angular rate.
        """
    def setGyroRateZ(self, angularRate: wpimath.units.degrees_per_second) -> None:
        """
        Sets the Z axis angular rate (CCW positive).
        
        :param angularRate: The angular rate.
        """
class ADIS16470_IMUSim:
    """
    Class to control a simulated ADIS16470 IMU.
    """
    def __init__(self, imu: wpilib._wpilib.ADIS16470_IMU) -> None:
        """
        Constructs from a ADIS16470_IMU object.
        
        :param imu: ADIS16470_IMU to simulate
        """
    def setAccelX(self, accel: wpimath.units.meters_per_second_squared) -> None:
        """
        Sets the X axis acceleration.
        
        :param accel: The acceleration.
        """
    def setAccelY(self, accel: wpimath.units.meters_per_second_squared) -> None:
        """
        Sets the Y axis acceleration.
        
        :param accel: The acceleration.
        """
    def setAccelZ(self, accel: wpimath.units.meters_per_second_squared) -> None:
        """
        Sets the Z axis acceleration.
        
        :param accel: The acceleration.
        """
    def setGyroAngleX(self, angle: wpimath.units.degrees) -> None:
        """
        Sets the X axis angle (CCW positive).
        
        :param angle: The angle.
        """
    def setGyroAngleY(self, angle: wpimath.units.degrees) -> None:
        """
        Sets the Y axis angle (CCW positive).
        
        :param angle: The angle.
        """
    def setGyroAngleZ(self, angle: wpimath.units.degrees) -> None:
        """
        Sets the Z axis angle (CCW positive).
        
        :param angle: The angle.
        """
    def setGyroRateX(self, angularRate: wpimath.units.degrees_per_second) -> None:
        """
        Sets the X axis angular rate (CCW positive).
        
        :param angularRate: The angular rate.
        """
    def setGyroRateY(self, angularRate: wpimath.units.degrees_per_second) -> None:
        """
        Sets the Y axis angular rate (CCW positive).
        
        :param angularRate: The angular rate.
        """
    def setGyroRateZ(self, angularRate: wpimath.units.degrees_per_second) -> None:
        """
        Sets the Z axis angular rate (CCW positive).
        
        :param angularRate: The angular rate.
        """
class ADXL345Sim:
    """
    Class to control a simulated ADXL345.
    """
    @typing.overload
    def __init__(self, accel: wpilib._wpilib.ADXL345_I2C) -> None:
        """
        Constructs from a ADXL345_I2C object.
        
        :param accel: ADXL345 accel to simulate
        """
    @typing.overload
    def __init__(self, accel: wpilib._wpilib.ADXL345_SPI) -> None:
        """
        Constructs from a ADXL345_SPI object.
        
        :param accel: ADXL345 accel to simulate
        """
    def setX(self, accel: float) -> None:
        """
        Sets the X acceleration.
        
        :param accel: The X acceleration.
        """
    def setY(self, accel: float) -> None:
        """
        Sets the Y acceleration.
        
        :param accel: The Y acceleration.
        """
    def setZ(self, accel: float) -> None:
        """
        Sets the Z acceleration.
        
        :param accel: The Z acceleration.
        """
class ADXL362Sim:
    """
    Class to control a simulated ADXL362.
    """
    def __init__(self, accel: wpilib._wpilib.ADXL362) -> None:
        """
        Constructs from a ADXL362 object.
        
        :param accel: ADXL362 accel to simulate
        """
    def setX(self, accel: float) -> None:
        """
        Sets the X acceleration.
        
        :param accel: The X acceleration.
        """
    def setY(self, accel: float) -> None:
        """
        Sets the Y acceleration.
        
        :param accel: The Y acceleration.
        """
    def setZ(self, accel: float) -> None:
        """
        Sets the Z acceleration.
        
        :param accel: The Z acceleration.
        """
class ADXRS450_GyroSim:
    """
    Class to control a simulated ADXRS450 gyroscope.
    """
    def __init__(self, gyro: wpilib._wpilib.ADXRS450_Gyro) -> None:
        """
        Constructs from a ADXRS450_Gyro object.
        
        :param gyro: ADXRS450_Gyro to simulate
        """
    def setAngle(self, angle: wpimath.units.degrees) -> None:
        """
        Sets the angle.
        
        :param angle: The angle (clockwise positive).
        """
    def setRate(self, rate: wpimath.units.degrees_per_second) -> None:
        """
        Sets the angular rate (clockwise positive).
        
        :param rate: The angular rate.
        """
class AddressableLEDSim:
    """
    Class to control a simulated addressable LED.
    """
    @staticmethod
    def createForChannel(pwmChannel: int) -> AddressableLEDSim:
        """
        Creates an AddressableLEDSim for a PWM channel.
        
        :param pwmChannel: PWM channel
        
        :returns: Simulated object
                  @throws std::out_of_range if no AddressableLED is configured for that
                  channel
        """
    @staticmethod
    def createForIndex(index: int) -> AddressableLEDSim:
        """
        Creates an AddressableLEDSim for a simulated index.
        The index is incremented for each simulated AddressableLED.
        
        :param index: simulator index
        
        :returns: Simulated object
        """
    @typing.overload
    def __init__(self) -> None:
        """
        Constructs for the first addressable LED.
        """
    @typing.overload
    def __init__(self, addressableLED: wpilib._wpilib.AddressableLED) -> None:
        """
        Constructs from an AddressableLED object.
        
        :param addressableLED: AddressableLED to simulate
        """
    def getData(self, data: hal._wpiHal.AddressableLEDData) -> int:
        """
        Get the LED data.
        
        :param data: output parameter to fill with LED data
        
        :returns: the length of the LED data
        """
    def getInitialized(self) -> bool:
        """
        Check if initialized.
        
        :returns: true if initialized
        """
    def getLength(self) -> int:
        """
        Get the length of the LED strip.
        
        :returns: the length
        """
    def getOutputPort(self) -> int:
        """
        Get the output port.
        
        :returns: the output port
        """
    def getRunning(self) -> int:
        """
        Check if the LEDs are running.
        
        :returns: true if they are
        """
    def registerDataCallback(self, callback: typing.Callable[[str, int, int], None], initialNotify: bool) -> CallbackStore:
        """
        Register a callback on the LED data.
        
        :param callback:      the callback that will be called whenever the LED data is
                              changed
        :param initialNotify: if true, the callback will be run on the initial value
        
        :returns: the CallbackStore object associated with this callback
        """
    def registerInitializedCallback(self, callback: typing.Callable[[str, hal._wpiHal.Value], None], initialNotify: bool) -> CallbackStore:
        """
        Register a callback on the Initialized property.
        
        :param callback:      the callback that will be called whenever the Initialized
                              property is changed
        :param initialNotify: if true, the callback will be run on the initial value
        
        :returns: the CallbackStore object storing this callback
        """
    def registerLengthCallback(self, callback: typing.Callable[[str, hal._wpiHal.Value], None], initialNotify: bool) -> CallbackStore:
        """
        Register a callback on the length.
        
        :param callback:      the callback that will be called whenever the length is
                              changed
        :param initialNotify: if true, the callback will be run on the initial value
        
        :returns: the CallbackStore object associated with this callback
        """
    def registerOutputPortCallback(self, callback: typing.Callable[[str, hal._wpiHal.Value], None], initialNotify: bool) -> CallbackStore:
        """
        Register a callback on the output port.
        
        :param callback:      the callback that will be called whenever the output port
                              is changed
        :param initialNotify: if true, the callback will be run on the initial value
        
        :returns: the CallbackStore object associated with this callback
        """
    def registerRunningCallback(self, callback: typing.Callable[[str, hal._wpiHal.Value], None], initialNotify: bool) -> CallbackStore:
        """
        Register a callback on whether the LEDs are running.
        
        :param callback:      the callback that will be called whenever the LED state is
                              changed
        :param initialNotify: if true, the callback will be run on the initial value
        
        :returns: the CallbackStore object associated with this callback
        """
    def setData(self, data: hal._wpiHal.AddressableLEDData, length: int) -> None:
        """
        Change the LED data.
        
        :param data:   the new data
        :param length: the length of the LED data
        """
    def setInitialized(self, initialized: bool) -> None:
        """
        Change the Initialized value of the LED strip.
        
        :param initialized: the new value
        """
    def setLength(self, length: int) -> None:
        """
        Change the length of the LED strip.
        
        :param length: the new value
        """
    def setOutputPort(self, outputPort: int) -> None:
        """
        Change the output port.
        
        :param outputPort: the new output port
        """
    def setRunning(self, running: bool) -> None:
        """
        Change whether the LEDs are active.
        
        :param running: the new value
        """
class AnalogEncoderSim:
    """
    Class to control a simulated analog encoder.
    """
    def __init__(self, encoder: wpilib._wpilib.AnalogEncoder) -> None:
        """
        Constructs from an AnalogEncoder object.
        
        :param encoder: AnalogEncoder to simulate
        """
    def getPosition(self) -> wpimath.geometry._geometry.Rotation2d:
        """
        Get the position as a Rotation2d.
        """
    def getTurns(self) -> wpimath.units.turns:
        """
        Get the simulated position.
        """
    def setPosition(self, angle: wpimath.geometry._geometry.Rotation2d) -> None:
        """
        Set the position using an Rotation2d.
        
        :param angle: The angle.
        """
    def setTurns(self, turns: wpimath.units.turns) -> None:
        """
        Set the position of the encoder.
        
        :param turns: The position.
        """
class AnalogGyroSim:
    """
    Class to control a simulated analog gyro.
    """
    @typing.overload
    def __init__(self, gyro: wpilib._wpilib.AnalogGyro) -> None:
        """
        Constructs from an AnalogGyro object.
        
        :param gyro: AnalogGyro to simulate
        """
    @typing.overload
    def __init__(self, channel: int) -> None:
        """
        Constructs from an analog input channel number.
        
        :param channel: Channel number
        """
    def getAngle(self) -> float:
        """
        Get the current angle of the gyro.
        
        :returns: the angle measured by the gyro
        """
    def getInitialized(self) -> bool:
        """
        Check if the gyro is initialized.
        
        :returns: true if initialized
        """
    def getRate(self) -> float:
        """
        Get the rate of angle change on this gyro.
        
        :returns: the rate
        """
    def registerAngleCallback(self, callback: typing.Callable[[str, hal._wpiHal.Value], None], initialNotify: bool) -> CallbackStore:
        """
        Register a callback on the angle.
        
        :param callback:      the callback that will be called whenever the angle changes
        :param initialNotify: if true, the callback will be run on the initial value
        
        :returns: the CallbackStore object associated with this callback
        """
    def registerInitializedCallback(self, callback: typing.Callable[[str, hal._wpiHal.Value], None], initialNotify: bool) -> CallbackStore:
        """
        Register a callback on whether the gyro is initialized.
        
        :param callback:      the callback that will be called whenever the gyro is
                              initialized
        :param initialNotify: if true, the callback will be run on the initial value
        
        :returns: the CallbackStore object associated with this callback
        """
    def registerRateCallback(self, callback: typing.Callable[[str, hal._wpiHal.Value], None], initialNotify: bool) -> CallbackStore:
        """
        Register a callback on the rate.
        
        :param callback:      the callback that will be called whenever the rate changes
        :param initialNotify: if true, the callback will be run on the initial value
        
        :returns: the CallbackStore object associated with this callback
        """
    def resetData(self) -> None:
        """
        Reset all simulation data for this object.
        """
    def setAngle(self, angle: float) -> None:
        """
        Change the angle measured by the gyro.
        
        :param angle: the new value
        """
    def setInitialized(self, initialized: bool) -> None:
        """
        Set whether this gyro is initialized.
        
        :param initialized: the new value
        """
    def setRate(self, rate: float) -> None:
        """
        Change the rate of the gyro.
        
        :param rate: the new rate
        """
class AnalogInputSim:
    """
    Class to control a simulated analog input.
    """
    @typing.overload
    def __init__(self, analogInput: wpilib._wpilib.AnalogInput) -> None:
        """
        Constructs from an AnalogInput object.
        
        :param analogInput: AnalogInput to simulate
        """
    @typing.overload
    def __init__(self, channel: int) -> None:
        """
        Constructs from an analog input channel number.
        
        :param channel: Channel number
        """
    def getAccumulatorCenter(self) -> int:
        """
        Get the accumulator center.
        
        :returns: the accumulator center
        """
    def getAccumulatorCount(self) -> int:
        """
        Get the accumulator count.
        
        :returns: the accumulator count.
        """
    def getAccumulatorDeadband(self) -> int:
        """
        Get the accumulator deadband.
        
        :returns: the accumulator deadband
        """
    def getAccumulatorInitialized(self) -> bool:
        """
        Check if the accumulator has been initialized.
        
        :returns: true if initialized
        """
    def getAccumulatorValue(self) -> int:
        """
        Get the accumulator value.
        
        :returns: the accumulator value
        """
    def getAverageBits(self) -> int:
        """
        Get the number of average bits.
        
        :returns: the number of average bits
        """
    def getInitialized(self) -> bool:
        """
        Check if this analog input has been initialized.
        
        :returns: true if initialized
        """
    def getOversampleBits(self) -> int:
        """
        Get the amount of oversampling bits.
        
        :returns: the amount of oversampling bits
        """
    def getVoltage(self) -> float:
        """
        Get the voltage.
        
        :returns: the voltage
        """
    def registerAccumulatorCenterCallback(self, callback: typing.Callable[[str, hal._wpiHal.Value], None], initialNotify: bool) -> CallbackStore:
        """
        Register a callback on the accumulator center.
        
        :param callback:      the callback that will be called whenever the accumulator
                              center is changed
        :param initialNotify: if true, the callback will be run on the initial value
        
        :returns: the CallbackStore object associated with this callback
        """
    def registerAccumulatorCountCallback(self, callback: typing.Callable[[str, hal._wpiHal.Value], None], initialNotify: bool) -> CallbackStore:
        """
        Register a callback on the accumulator count.
        
        :param callback:      the callback that will be called whenever the accumulator
                              count is changed
        :param initialNotify: if true, the callback will be run on the initial value
        
        :returns: the CallbackStore object associated with this callback
        """
    def registerAccumulatorDeadbandCallback(self, callback: typing.Callable[[str, hal._wpiHal.Value], None], initialNotify: bool) -> CallbackStore:
        """
        Register a callback on the accumulator deadband.
        
        :param callback:      the callback that will be called whenever the accumulator
                              deadband is changed
        :param initialNotify: if true, the callback will be run on the initial value
        
        :returns: the CallbackStore object associated with this callback
        """
    def registerAccumulatorInitializedCallback(self, callback: typing.Callable[[str, hal._wpiHal.Value], None], initialNotify: bool) -> CallbackStore:
        """
        Register a callback on whether the accumulator is initialized.
        
        :param callback:      the callback that will be called whenever the accumulator
                              is initialized
        :param initialNotify: if true, the callback will be run on the initial value
        
        :returns: the CallbackStore object associated with this callback
        """
    def registerAccumulatorValueCallback(self, callback: typing.Callable[[str, hal._wpiHal.Value], None], initialNotify: bool) -> CallbackStore:
        """
        Register a callback on the accumulator value.
        
        :param callback:      the callback that will be called whenever the accumulator
                              value is changed
        :param initialNotify: if true, the callback will be run on the initial value
        
        :returns: the CallbackStore object associated with this callback
        """
    def registerAverageBitsCallback(self, callback: typing.Callable[[str, hal._wpiHal.Value], None], initialNotify: bool) -> CallbackStore:
        """
        Register a callback on the number of average bits.
        
        :param callback:      the callback that will be called whenever the number of
                              average bits is changed
        :param initialNotify: if true, the callback will be run on the initial value
        
        :returns: the CallbackStore object associated with this callback
        """
    def registerInitializedCallback(self, callback: typing.Callable[[str, hal._wpiHal.Value], None], initialNotify: bool) -> CallbackStore:
        """
        Register a callback on whether the analog input is initialized.
        
        :param callback:      the callback that will be called whenever the analog input
                              is initialized
        :param initialNotify: if true, the callback will be run on the initial value
        
        :returns: the CallbackStore object associated with this callback
        """
    def registerOversampleBitsCallback(self, callback: typing.Callable[[str, hal._wpiHal.Value], None], initialNotify: bool) -> CallbackStore:
        """
        Register a callback on the amount of oversampling bits.
        
        :param callback:      the callback that will be called whenever the oversampling
                              bits are changed
        :param initialNotify: if true, the callback will be run on the initial value
        
        :returns: the CallbackStore object associated with this callback
        """
    def registerVoltageCallback(self, callback: typing.Callable[[str, hal._wpiHal.Value], None], initialNotify: bool) -> CallbackStore:
        """
        Register a callback on the voltage.
        
        :param callback:      the callback that will be called whenever the voltage is
                              changed
        :param initialNotify: if true, the callback will be run on the initial value
        
        :returns: the CallbackStore object associated with this callback
        """
    def resetData(self) -> None:
        """
        Reset all simulation data for this object.
        """
    def setAccumulatorCenter(self, accumulatorCenter: int) -> None:
        """
        Change the accumulator center.
        
        :param accumulatorCenter: the new center
        """
    def setAccumulatorCount(self, accumulatorCount: int) -> None:
        """
        Change the accumulator count.
        
        :param accumulatorCount: the new count.
        """
    def setAccumulatorDeadband(self, accumulatorDeadband: int) -> None:
        """
        Change the accumulator deadband.
        
        :param accumulatorDeadband: the new deadband
        """
    def setAccumulatorInitialized(self, accumulatorInitialized: bool) -> None:
        """
        Change whether the accumulator has been initialized.
        
        :param accumulatorInitialized: the new value
        """
    def setAccumulatorValue(self, accumulatorValue: int) -> None:
        """
        Change the accumulator value.
        
        :param accumulatorValue: the new value
        """
    def setAverageBits(self, averageBits: int) -> None:
        """
        Change the number of average bits.
        
        :param averageBits: the new value
        """
    def setInitialized(self, initialized: bool) -> None:
        """
        Change whether this analog input has been initialized.
        
        :param initialized: the new value
        """
    def setOversampleBits(self, oversampleBits: int) -> None:
        """
        Change the amount of oversampling bits.
        
        :param oversampleBits: the new value
        """
    def setVoltage(self, voltage: float) -> None:
        """
        Change the voltage.
        
        :param voltage: the new value
        """
class AnalogOutputSim:
    """
    Class to control a simulated analog output.
    """
    @typing.overload
    def __init__(self, analogOutput: wpilib._wpilib.AnalogOutput) -> None:
        """
        Constructs from an AnalogOutput object.
        
        :param analogOutput: AnalogOutput to simulate
        """
    @typing.overload
    def __init__(self, channel: int) -> None:
        """
        Constructs from an analog output channel number.
        
        :param channel: Channel number
        """
    def getInitialized(self) -> bool:
        """
        Check whether this analog output has been initialized.
        
        :returns: true if initialized
        """
    def getVoltage(self) -> float:
        """
        Read the analog output voltage.
        
        :returns: the voltage on this analog output
        """
    def registerInitializedCallback(self, callback: typing.Callable[[str, hal._wpiHal.Value], None], initialNotify: bool) -> CallbackStore:
        """
        Register a callback to be run when this analog output is initialized.
        
        :param callback:      the callback
        :param initialNotify: whether to run the callback with the initial state
        
        :returns: the CallbackStore object associated with this callback
        """
    def registerVoltageCallback(self, callback: typing.Callable[[str, hal._wpiHal.Value], None], initialNotify: bool) -> CallbackStore:
        """
        Register a callback to be run whenever the voltage changes.
        
        :param callback:      the callback
        :param initialNotify: whether to call the callback with the initial state
        
        :returns: the CallbackStore object associated with this callback
        """
    def resetData(self) -> None:
        """
        Reset all simulation data on this object.
        """
    def setInitialized(self, initialized: bool) -> None:
        """
        Define whether this analog output has been initialized.
        
        :param initialized: whether this object is initialized
        """
    def setVoltage(self, voltage: float) -> None:
        """
        Set the analog output voltage.
        
        :param voltage: the new voltage on this analog output
        """
class AnalogTriggerSim:
    """
    Class to control a simulated analog trigger.
    """
    @staticmethod
    def createForChannel(channel: int) -> AnalogTriggerSim:
        """
        Creates an AnalogTriggerSim for an analog input channel.
        
        :param channel: analog input channel
        
        :returns: Simulated object
                  @throws std::out_of_range if no AnalogTrigger is configured for that
                  channel
        """
    @staticmethod
    def createForIndex(index: int) -> AnalogTriggerSim:
        """
        Creates an AnalogTriggerSim for a simulated index.
        The index is incremented for each simulated AnalogTrigger.
        
        :param index: simulator index
        
        :returns: Simulated object
        """
    def __init__(self, analogTrigger: wpilib._wpilib.AnalogTrigger) -> None:
        """
        Constructs from an AnalogTrigger object.
        
        :param analogTrigger: AnalogTrigger to simulate
        """
    def getInitialized(self) -> bool:
        """
        Check if this analog trigger has been initialized.
        
        :returns: true if initialized
        """
    def getTriggerLowerBound(self) -> float:
        """
        Get the lower bound.
        
        :returns: the lower bound
        """
    def getTriggerUpperBound(self) -> float:
        """
        Get the upper bound.
        
        :returns: the upper bound
        """
    def registerInitializedCallback(self, callback: typing.Callable[[str, hal._wpiHal.Value], None], initialNotify: bool) -> CallbackStore:
        """
        Register a callback on whether the analog trigger is initialized.
        
        :param callback:      the callback that will be called whenever the analog
                              trigger is initialized
        :param initialNotify: if true, the callback will be run on the initial value
        
        :returns: the CallbackStore object associated with this callback
        """
    def registerTriggerLowerBoundCallback(self, callback: typing.Callable[[str, hal._wpiHal.Value], None], initialNotify: bool) -> CallbackStore:
        """
        Register a callback on the lower bound.
        
        :param callback:      the callback that will be called whenever the lower bound
                              is changed
        :param initialNotify: if true, the callback will be run on the initial value
        
        :returns: the CallbackStore object associated with this callback
        """
    def registerTriggerUpperBoundCallback(self, callback: typing.Callable[[str, hal._wpiHal.Value], None], initialNotify: bool) -> CallbackStore:
        """
        Register a callback on the upper bound.
        
        :param callback:      the callback that will be called whenever the upper bound
                              is changed
        :param initialNotify: if true, the callback will be run on the initial value
        
        :returns: the CallbackStore object associated with this callback
        """
    def resetData(self) -> None:
        """
        Reset all simulation data for this object.
        """
    def setInitialized(self, initialized: bool) -> None:
        """
        Change whether this analog trigger has been initialized.
        
        :param initialized: the new value
        """
    def setTriggerLowerBound(self, triggerLowerBound: float) -> None:
        """
        Change the lower bound.
        
        :param triggerLowerBound: the new lower bound
        """
    def setTriggerUpperBound(self, triggerUpperBound: float) -> None:
        """
        Change the upper bound.
        
        :param triggerUpperBound: the new upper bound
        """
class BatterySim:
    """
    A utility class to simulate the robot battery.
    """
    @staticmethod
    @typing.overload
    def calculate(nominalVoltage: wpimath.units.volts, resistance: wpimath.units.ohms, currents: list[wpimath.units.amperes]) -> wpimath.units.volts:
        """
        Calculate the loaded battery voltage. Use this with
        RoboRioSim::SetVInVoltage(double) to set the simulated battery voltage,
        which can then be retrieved with the RobotController::GetBatteryVoltage()
        method.
        
        :param nominalVoltage: The nominal battery voltage. Usually 12v.
        :param resistance:     The forward resistance of the battery. Most batteries
                               are at or below 20 milliohms.
        :param currents:       The currents drawn from the battery.
        
        :returns: The battery's voltage under load.
        """
    @staticmethod
    @typing.overload
    def calculate(currents: list[wpimath.units.amperes]) -> wpimath.units.volts:
        """
        Calculate the loaded battery voltage. Use this with
        RoboRioSimSetVInVoltage(double) to set the simulated battery voltage, which
        can then be retrieved with the RobotController::GetBatteryVoltage() method.
        This function assumes a nominal voltage of 12V and a resistance of 20
        milliohms (0.020 ohms).
        
        :param currents: The currents drawn from the battery.
        
        :returns: The battery's voltage under load.
        """
    def __init__(self) -> None:
        ...
class BuiltInAccelerometerSim:
    """
    Class to control a simulated built-in accelerometer.
    """
    @typing.overload
    def __init__(self) -> None:
        """
        Constructs for the first built-in accelerometer.
        """
    @typing.overload
    def __init__(self, accel: wpilib._wpilib.BuiltInAccelerometer) -> None:
        """
        Constructs from a BuiltInAccelerometer object.
        
        :param accel: BuiltInAccelerometer to simulate
        """
    def getActive(self) -> bool:
        """
        Check whether the accelerometer is active.
        
        :returns: true if active
        """
    def getRange(self) -> hal._wpiHal.AccelerometerRange:
        """
        Check the range of this accelerometer.
        
        :returns: the accelerometer range
        """
    def getX(self) -> float:
        """
        Measure the X axis value.
        
        :returns: the X axis measurement
        """
    def getY(self) -> float:
        """
        Measure the Y axis value.
        
        :returns: the Y axis measurement
        """
    def getZ(self) -> float:
        """
        Measure the Z axis value.
        
        :returns: the Z axis measurement
        """
    def registerActiveCallback(self, callback: typing.Callable[[str, hal._wpiHal.Value], None], initialNotify: bool) -> CallbackStore:
        """
        Register a callback to be run when this accelerometer activates.
        
        :param callback:      the callback
        :param initialNotify: whether to run the callback with the initial state
        
        :returns: the CallbackStore object associated with this callback
        """
    def registerRangeCallback(self, callback: typing.Callable[[str, hal._wpiHal.Value], None], initialNotify: bool) -> CallbackStore:
        """
        Register a callback to be run whenever the range changes.
        
        :param callback:      the callback
        :param initialNotify: whether to call the callback with the initial state
        
        :returns: the CallbackStore object associated with this callback
        """
    def registerXCallback(self, callback: typing.Callable[[str, hal._wpiHal.Value], None], initialNotify: bool) -> CallbackStore:
        """
        Register a callback to be run whenever the X axis value changes.
        
        :param callback:      the callback
        :param initialNotify: whether to call the callback with the initial state
        
        :returns: the CallbackStore object associated with this callback
        """
    def registerYCallback(self, callback: typing.Callable[[str, hal._wpiHal.Value], None], initialNotify: bool) -> CallbackStore:
        """
        Register a callback to be run whenever the Y axis value changes.
        
        :param callback:      the callback
        :param initialNotify: whether to call the callback with the initial state
        
        :returns: the CallbackStore object associated with this callback
        """
    def registerZCallback(self, callback: typing.Callable[[str, hal._wpiHal.Value], None], initialNotify: bool) -> CallbackStore:
        """
        Register a callback to be run whenever the Z axis value changes.
        
        :param callback:      the callback
        :param initialNotify: whether to call the callback with the initial state
        
        :returns: the CallbackStore object associated with this callback
        """
    def resetData(self) -> None:
        """
        Reset all simulation data of this object.
        """
    def setActive(self, active: bool) -> None:
        """
        Define whether this accelerometer is active.
        
        :param active: the new state
        """
    def setRange(self, range: hal._wpiHal.AccelerometerRange) -> None:
        """
        Change the range of this accelerometer.
        
        :param range: the new accelerometer range
        """
    def setX(self, x: float) -> None:
        """
        Change the X axis value of the accelerometer.
        
        :param x: the new reading of the X axis
        """
    def setY(self, y: float) -> None:
        """
        Change the Y axis value of the accelerometer.
        
        :param y: the new reading of the Y axis
        """
    def setZ(self, z: float) -> None:
        """
        Change the Z axis value of the accelerometer.
        
        :param z: the new reading of the Z axis
        """
class CTREPCMSim(PneumaticsBaseSim):
    """
    Class to control a simulated Pneumatic Control Module (PCM).
    """
    @typing.overload
    def __init__(self) -> None:
        """
        Constructs with the default PCM module number (CAN ID).
        """
    @typing.overload
    def __init__(self, module: int) -> None:
        """
        Constructs from a PCM module number (CAN ID).
        
        :param module: module number
        """
    @typing.overload
    def __init__(self, pneumatics: wpilib._wpilib.PneumaticsBase) -> None:
        ...
    def getAllSolenoidOutputs(self) -> int:
        ...
    def getClosedLoopEnabled(self) -> bool:
        """
        Check whether the closed loop compressor control is active.
        
        :returns: true if active
        """
    def getCompressorCurrent(self) -> float:
        """
        Read the compressor current.
        
        :returns: the current of the compressor connected to this module
        """
    def getCompressorOn(self) -> bool:
        ...
    def getInitialized(self) -> bool:
        ...
    def getPressureSwitch(self) -> bool:
        """
        Check the value of the pressure switch.
        
        :returns: the pressure switch value
        """
    def getSolenoidOutput(self, channel: int) -> bool:
        ...
    def registerClosedLoopEnabledCallback(self, callback: typing.Callable[[str, hal._wpiHal.Value], None], initialNotify: bool) -> CallbackStore:
        """
        Register a callback to be run whenever the closed loop state changes.
        
        :param callback:      the callback
        :param initialNotify: whether the callback should be called with the
                              initial value
        
        :returns: the CallbackStore object associated with this callback
        """
    def registerCompressorCurrentCallback(self, callback: typing.Callable[[str, hal._wpiHal.Value], None], initialNotify: bool) -> CallbackStore:
        """
        Register a callback to be run whenever the compressor current changes.
        
        :param callback:      the callback
        :param initialNotify: whether to call the callback with the initial state
        
        :returns: the CallbackStore object associated with this callback
        """
    def registerCompressorOnCallback(self, callback: typing.Callable[[str, hal._wpiHal.Value], None], initialNotify: bool) -> CallbackStore:
        ...
    def registerInitializedCallback(self, callback: typing.Callable[[str, hal._wpiHal.Value], None], initialNotify: bool) -> CallbackStore:
        ...
    def registerPressureSwitchCallback(self, callback: typing.Callable[[str, hal._wpiHal.Value], None], initialNotify: bool) -> CallbackStore:
        """
        Register a callback to be run whenever the pressure switch value changes.
        
        :param callback:      the callback
        :param initialNotify: whether the callback should be called with the
                              initial value
        
        :returns: the CallbackStore object associated with this callback
        """
    def registerSolenoidOutputCallback(self, channel: int, callback: typing.Callable[[str, hal._wpiHal.Value], None], initialNotify: bool) -> CallbackStore:
        ...
    def resetData(self) -> None:
        ...
    def setAllSolenoidOutputs(self, outputs: int) -> None:
        ...
    def setClosedLoopEnabled(self, closedLoopEnabled: bool) -> None:
        """
        Turn on/off the closed loop control of the compressor.
        
        :param closedLoopEnabled: whether the control loop is active
        """
    def setCompressorCurrent(self, compressorCurrent: float) -> None:
        """
        Set the compressor current.
        
        :param compressorCurrent: the new compressor current
        """
    def setCompressorOn(self, compressorOn: bool) -> None:
        ...
    def setInitialized(self, initialized: bool) -> None:
        ...
    def setPressureSwitch(self, pressureSwitch: bool) -> None:
        """
        Set the value of the pressure switch.
        
        :param pressureSwitch: the new value
        """
    def setSolenoidOutput(self, channel: int, solenoidOutput: bool) -> None:
        ...
class CallbackStore:
    """
    Manages simulation callbacks; each object is associated with a callback.
    """
    def setUid(self, uid: int) -> None:
        ...
class DCMotorSim(LinearSystemSim_2_1_2):
    """
    Represents a simulated DC motor mechanism.
    """
    @typing.overload
    def __init__(self, plant: wpimath._controls._controls.system.LinearSystem_2_1_2, gearbox: wpimath._controls._controls.plant.DCMotor, gearing: float, measurementStdDevs: typing.Annotated[list[float], pybind11_stubgen.typing_ext.FixedSize(2)] = [0.0, 0.0]) -> None:
        """
        Creates a simulated DC motor mechanism.
        
        :param plant:              The linear system representing the DC motor. This
                                   system can be created with
                                   LinearSystemId::DCMotorSystem().
        :param gearbox:            The type of and number of motors in the DC motor
                                   gearbox.
        :param gearing:            The gearing of the DC motor (numbers greater than
                                   1 represent reductions).
        :param measurementStdDevs: The standard deviation of the measurement noise.
        """
    @typing.overload
    def __init__(self, gearbox: wpimath._controls._controls.plant.DCMotor, gearing: float, moi: wpimath.units.kilogram_square_meters, measurementStdDevs: typing.Annotated[list[float], pybind11_stubgen.typing_ext.FixedSize(2)] = [0.0, 0.0]) -> None:
        """
        Creates a simulated DC motor mechanism.
        
        :param gearbox:            The type of and number of motors in the DC motor
                                   gearbox.
        :param gearing:            The gearing of the DC motor (numbers greater than
                                   1 represent reductions).
        :param moi:                The moment of inertia of the DC motor.
        :param measurementStdDevs: The standard deviation of the measurement noise.
        """
    def getAngularPosition(self) -> wpimath.units.radians:
        """
        Returns the DC motor position.
        
        :returns: The DC motor position.
        """
    def getAngularVelocity(self) -> wpimath.units.radians_per_second:
        """
        Returns the DC motor velocity.
        
        :returns: The DC motor velocity.
        """
    def getCurrentDraw(self) -> wpimath.units.amperes:
        """
        Returns the DC motor current draw.
        
        :returns: The DC motor current draw.
        """
    def setInputVoltage(self, voltage: wpimath.units.volts) -> None:
        """
        Sets the input voltage for the DC motor.
        
        :param voltage: The input voltage.
        """
    def setState(self, angularPosition: wpimath.units.radians, angularVelocity: wpimath.units.radians_per_second) -> None:
        """
        Sets the state of the DC motor.
        
        :param angularPosition: The new position
        :param angularVelocity: The new velocity
        """
class DIOSim:
    """
    Class to control a simulated digital input or output.
    """
    @typing.overload
    def __init__(self, input: wpilib._wpilib.DigitalInput) -> None:
        """
        Constructs from a DigitalInput object.
        
        :param input: DigitalInput to simulate
        """
    @typing.overload
    def __init__(self, output: wpilib._wpilib.DigitalOutput) -> None:
        """
        Constructs from a DigitalOutput object.
        
        :param output: DigitalOutput to simulate
        """
    @typing.overload
    def __init__(self, channel: int) -> None:
        """
        Constructs from an digital I/O channel number.
        
        :param channel: Channel number
        """
    def getFilterIndex(self) -> int:
        """
        Read the filter index.
        
        :returns: the filter index of this DIO port
        """
    def getInitialized(self) -> bool:
        """
        Check whether this DIO has been initialized.
        
        :returns: true if initialized
        """
    def getIsInput(self) -> bool:
        """
        Check whether this DIO port is currently an Input.
        
        :returns: true if Input
        """
    def getPulseLength(self) -> float:
        """
        Read the pulse length.
        
        :returns: the pulse length of this DIO port
        """
    def getValue(self) -> bool:
        """
        Read the value of the DIO port.
        
        :returns: the DIO value
        """
    def registerFilterIndexCallback(self, callback: typing.Callable[[str, hal._wpiHal.Value], None], initialNotify: bool) -> CallbackStore:
        """
        Register a callback to be run whenever the filter index changes.
        
        :param callback:      the callback
        :param initialNotify: whether the callback should be called with the
                              initial value
        
        :returns: the CallbackStore object associated with this callback
        """
    def registerInitializedCallback(self, callback: typing.Callable[[str, hal._wpiHal.Value], None], initialNotify: bool) -> CallbackStore:
        """
        Register a callback to be run when this DIO is initialized.
        
        :param callback:      the callback
        :param initialNotify: whether to run the callback with the initial state
        
        :returns: the CallbackStore object associated with this callback
        """
    def registerIsInputCallback(self, callback: typing.Callable[[str, hal._wpiHal.Value], None], initialNotify: bool) -> CallbackStore:
        """
        Register a callback to be run whenever this DIO changes to be an input.
        
        :param callback:      the callback
        :param initialNotify: whether the callback should be called with the
                              initial state
        
        :returns: the CallbackStore object associated with this callback
        """
    def registerPulseLengthCallback(self, callback: typing.Callable[[str, hal._wpiHal.Value], None], initialNotify: bool) -> CallbackStore:
        """
        Register a callback to be run whenever the pulse length changes.
        
        :param callback:      the callback
        :param initialNotify: whether to call the callback with the initial state
        
        :returns: the CallbackStore object associated with this callback
        """
    def registerValueCallback(self, callback: typing.Callable[[str, hal._wpiHal.Value], None], initialNotify: bool) -> CallbackStore:
        """
        Register a callback to be run whenever the DIO value changes.
        
        :param callback:      the callback
        :param initialNotify: whether the callback should be called with the
                              initial value
        
        :returns: the CallbackStore object associated with this callback
        """
    def resetData(self) -> None:
        """
        Reset all simulation data of this object.
        """
    def setFilterIndex(self, filterIndex: int) -> None:
        """
        Change the filter index of this DIO port.
        
        :param filterIndex: the new filter index
        """
    def setInitialized(self, initialized: bool) -> None:
        """
        Define whether this DIO has been initialized.
        
        :param initialized: whether this object is initialized
        """
    def setIsInput(self, isInput: bool) -> None:
        """
        Define whether this DIO port is an Input.
        
        :param isInput: whether this DIO should be an Input
        """
    def setPulseLength(self, pulseLength: float) -> None:
        """
        Change the pulse length of this DIO port.
        
        :param pulseLength: the new pulse length
        """
    def setValue(self, value: bool) -> None:
        """
        Change the DIO value.
        
        :param value: the new value
        """
class DifferentialDrivetrainSim:
    class KitbotGearing:
        """
        Represents a gearing option of the Toughbox mini.
        12.75:1 -- 14:50 and 14:50
        10.71:1 -- 14:50 and 16:48
        8.45:1 -- 14:50 and 19:45
        7.31:1 -- 14:50 and 21:43
        5.95:1 -- 14:50 and 24:40
        """
        k10p71: typing.ClassVar[float] = 10.71
        k12p75: typing.ClassVar[float] = 12.75
        k5p95: typing.ClassVar[float] = 5.95
        k7p31: typing.ClassVar[float] = 7.31
        k8p45: typing.ClassVar[float] = 8.45
        def __init__(self) -> None:
            ...
    class KitbotMotor:
        """
        Represents common motor layouts of the kit drivetrain.
        """
        DualCIMPerSide: typing.ClassVar[wpimath._controls._controls.plant.DCMotor]  # value = <wpimath._controls._controls.plant.DCMotor object>
        DualFalcon500PerSide: typing.ClassVar[wpimath._controls._controls.plant.DCMotor]  # value = <wpimath._controls._controls.plant.DCMotor object>
        DualMiniCIMPerSide: typing.ClassVar[wpimath._controls._controls.plant.DCMotor]  # value = <wpimath._controls._controls.plant.DCMotor object>
        DualNEOPerSide: typing.ClassVar[wpimath._controls._controls.plant.DCMotor]  # value = <wpimath._controls._controls.plant.DCMotor object>
        SingleCIMPerSide: typing.ClassVar[wpimath._controls._controls.plant.DCMotor]  # value = <wpimath._controls._controls.plant.DCMotor object>
        SingleFalcon500PerSide: typing.ClassVar[wpimath._controls._controls.plant.DCMotor]  # value = <wpimath._controls._controls.plant.DCMotor object>
        SingleMiniCIMPerSide: typing.ClassVar[wpimath._controls._controls.plant.DCMotor]  # value = <wpimath._controls._controls.plant.DCMotor object>
        SingleNEOPerSide: typing.ClassVar[wpimath._controls._controls.plant.DCMotor]  # value = <wpimath._controls._controls.plant.DCMotor object>
        def __init__(self) -> None:
            ...
    class KitbotWheelSize:
        """
        Represents common wheel sizes of the kit drivetrain.
        """
        kEightInch: typing.ClassVar[float] = 0.2032
        kSixInch: typing.ClassVar[float] = 0.1524
        kTenInch: typing.ClassVar[float] = 0.254
        def __init__(self) -> None:
            ...
    class State:
        kHeading: typing.ClassVar[int] = 2
        kLeftPosition: typing.ClassVar[int] = 5
        kLeftVelocity: typing.ClassVar[int] = 3
        kRightPosition: typing.ClassVar[int] = 6
        kRightVelocity: typing.ClassVar[int] = 4
        kX: typing.ClassVar[int] = 0
        kY: typing.ClassVar[int] = 1
        def __init__(self) -> None:
            ...
    @staticmethod
    @typing.overload
    def createKitbotSim(motor: wpimath._controls._controls.plant.DCMotor, gearing: float, wheelSize: wpimath.units.meters, measurementStdDevs: typing.Annotated[list[float], pybind11_stubgen.typing_ext.FixedSize(7)] = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]) -> DifferentialDrivetrainSim:
        """
        Create a sim for the standard FRC kitbot.
        
        :param motor:              The motors installed in the bot.
        :param gearing:            The gearing reduction used.
        :param wheelSize:          The wheel size.
        :param measurementStdDevs: Standard deviations for measurements, in the form
                                   [x, y, heading, left velocity, right velocity, left distance, right
                                   distance]ᵀ. Can be omitted if no noise is desired. Gyro standard
                                   deviations of 0.0001 radians, velocity standard deviations of 0.05 m/s, and
                                   position measurement standard deviations of 0.005 meters are a reasonable
                                   starting point.
        """
    @staticmethod
    @typing.overload
    def createKitbotSim(motor: wpimath._controls._controls.plant.DCMotor, gearing: float, wheelSize: wpimath.units.meters, J: wpimath.units.kilogram_square_meters, measurementStdDevs: typing.Annotated[list[float], pybind11_stubgen.typing_ext.FixedSize(7)] = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]) -> DifferentialDrivetrainSim:
        """
        Create a sim for the standard FRC kitbot.
        
        :param motor:              The motors installed in the bot.
        :param gearing:            The gearing reduction used.
        :param wheelSize:          The wheel size.
        :param J:                  The moment of inertia of the drivebase. This can be
                                   calculated using SysId.
        :param measurementStdDevs: Standard deviations for measurements, in the form
                                   [x, y, heading, left velocity, right velocity, left distance, right
                                   distance]ᵀ. Can be omitted if no noise is desired. Gyro standard
                                   deviations of 0.0001 radians, velocity standard deviations of 0.05 m/s, and
                                   position measurement standard deviations of 0.005 meters are a reasonable
                                   starting point.
        """
    @typing.overload
    def __init__(self, plant: wpimath._controls._controls.system.LinearSystem_2_2_2, trackWidth: wpimath.units.meters, driveMotor: wpimath._controls._controls.plant.DCMotor, gearingRatio: float, wheelRadius: wpimath.units.meters, measurementStdDevs: typing.Annotated[list[float], pybind11_stubgen.typing_ext.FixedSize(7)] = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]) -> None:
        """
        Creates a simulated differential drivetrain.
        
        :param plant:              The LinearSystem representing the robot's drivetrain. This
                                   system can be created with
                                   LinearSystemId::DrivetrainVelocitySystem() or
                                   LinearSystemId::IdentifyDrivetrainSystem().
        :param trackWidth:         The robot's track width.
        :param driveMotor:         A DCMotor representing the left side of the drivetrain.
        :param gearingRatio:       The gearingRatio ratio of the left side, as output over
                                   input. This must be the same ratio as the ratio used to
                                   identify or create the plant.
        :param wheelRadius:        The radius of the wheels on the drivetrain, in meters.
        :param measurementStdDevs: Standard deviations for measurements, in the form
                                   [x, y, heading, left velocity, right velocity,
                                   left distance, right distance]ᵀ. Can be omitted
                                   if no noise is desired. Gyro standard deviations
                                   of 0.0001 radians, velocity standard deviations
                                   of 0.05 m/s, and position measurement standard
                                   deviations of 0.005 meters are a reasonable
                                   starting point.
        """
    @typing.overload
    def __init__(self, driveMotor: wpimath._controls._controls.plant.DCMotor, gearing: float, J: wpimath.units.kilogram_square_meters, mass: wpimath.units.kilograms, wheelRadius: wpimath.units.meters, trackWidth: wpimath.units.meters, measurementStdDevs: typing.Annotated[list[float], pybind11_stubgen.typing_ext.FixedSize(7)] = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]) -> None:
        """
        Creates a simulated differential drivetrain.
        
        :param driveMotor:         A DCMotor representing the left side of the drivetrain.
        :param gearing:            The gearing on the drive between motor and wheel, as
                                   output over input. This must be the same ratio as the
                                   ratio used to identify or create the plant.
        :param J:                  The moment of inertia of the drivetrain about its
                                   center.
        :param mass:               The mass of the drivebase.
        :param wheelRadius:        The radius of the wheels on the drivetrain.
        :param trackWidth:         The robot's track width, or distance between left and
                                   right wheels.
        :param measurementStdDevs: Standard deviations for measurements, in the form
                                   [x, y, heading, left velocity, right velocity,
                                   left distance, right distance]ᵀ. Can be omitted
                                   if no noise is desired. Gyro standard deviations
                                   of 0.0001 radians, velocity standard deviations
                                   of 0.05 m/s, and position measurement standard
                                   deviations of 0.005 meters are a reasonable
                                   starting point.
        """
    def clampInput(self, u: numpy.ndarray[numpy.float64[2, 1]]) -> numpy.ndarray[numpy.float64[2, 1]]:
        """
        Clamp the input vector such that no element exceeds the battery voltage.
        If any does, the relative magnitudes of the input will be maintained.
        
        :param u: The input vector.
        
        :returns: The normalized input.
        """
    def dynamics(self, x: numpy.ndarray[numpy.float64[7, 1]], u: numpy.ndarray[numpy.float64[2, 1]]) -> numpy.ndarray[numpy.float64[7, 1]]:
        """
        The differential drive dynamics function.
        
        :param x: The state.
        :param u: The input.
        
        :returns: The state derivative with respect to time.
        """
    def getCurrentDraw(self) -> wpimath.units.amperes:
        """
        Returns the currently drawn current.
        """
    def getGearing(self) -> float:
        """
        Returns the current gearing reduction of the drivetrain, as output over
        input.
        """
    def getHeading(self) -> wpimath.geometry._geometry.Rotation2d:
        """
        Returns the direction the robot is pointing.
        
        Note that this angle is counterclockwise-positive, while most gyros are
        clockwise positive.
        """
    def getLeftCurrentDraw(self) -> wpimath.units.amperes:
        """
        Returns the currently drawn current for the left side.
        """
    def getLeftPosition(self) -> wpimath.units.meters:
        """
        Get the left encoder position in meters.
        
        :returns: The encoder position.
        """
    def getLeftPositionFeet(self) -> wpimath.units.feet:
        ...
    def getLeftPositionInches(self) -> wpimath.units.inches:
        ...
    def getLeftVelocity(self) -> wpimath.units.meters_per_second:
        """
        Get the left encoder velocity in meters per second.
        
        :returns: The encoder velocity.
        """
    def getLeftVelocityFps(self) -> wpimath.units.feet_per_second:
        ...
    def getPose(self) -> wpimath.geometry._geometry.Pose2d:
        """
        Returns the current pose.
        """
    def getRightCurrentDraw(self) -> wpimath.units.amperes:
        """
        Returns the currently drawn current for the right side.
        """
    def getRightPosition(self) -> wpimath.units.meters:
        """
        Get the right encoder position in meters.
        
        :returns: The encoder position.
        """
    def getRightPositionFeet(self) -> wpimath.units.feet:
        ...
    def getRightPositionInches(self) -> wpimath.units.inches:
        ...
    def getRightVelocity(self) -> wpimath.units.meters_per_second:
        """
        Get the right encoder velocity in meters per second.
        
        :returns: The encoder velocity.
        """
    def getRightVelocityFps(self) -> wpimath.units.feet_per_second:
        ...
    def setGearing(self, newGearing: float) -> None:
        """
        Sets the gearing reduction on the drivetrain. This is commonly used for
        shifting drivetrains.
        
        :param newGearing: The new gear ratio, as output over input.
        """
    def setInputs(self, leftVoltage: wpimath.units.volts, rightVoltage: wpimath.units.volts) -> None:
        """
        Sets the applied voltage to the drivetrain. Note that positive voltage must
        make that side of the drivetrain travel forward (+X).
        
        :param leftVoltage:  The left voltage.
        :param rightVoltage: The right voltage.
        """
    def setPose(self, pose: wpimath.geometry._geometry.Pose2d) -> None:
        """
        Sets the system pose.
        
        :param pose: The pose.
        """
    def setState(self, state: numpy.ndarray[numpy.float64[7, 1]]) -> None:
        """
        Sets the system state.
        
        :param state: The state.
        """
    def update(self, dt: wpimath.units.seconds) -> None:
        """
        Updates the simulation.
        
        :param dt: The time that's passed since the last Update(units::second_t)
                   call.
        """
class DigitalPWMSim:
    """
    Class to control a simulated digital PWM output.
    
    This is for duty cycle PWM outputs on a DigitalOutput, not for the servo
    style PWM outputs on a PWM channel.
    """
    @staticmethod
    def createForChannel(channel: int) -> DigitalPWMSim:
        """
        Creates an DigitalPWMSim for a digital I/O channel.
        
        :param channel: DIO channel
        
        :returns: Simulated object
                  @throws std::out_of_range if no Digital PWM is configured for that channel
        """
    @staticmethod
    def createForIndex(index: int) -> DigitalPWMSim:
        """
        Creates an DigitalPWMSim for a simulated index.
        The index is incremented for each simulated DigitalPWM.
        
        :param index: simulator index
        
        :returns: Simulated object
        """
    def __init__(self, digitalOutput: wpilib._wpilib.DigitalOutput) -> None:
        """
        Constructs from a DigitalOutput object.
        
        :param digitalOutput: DigitalOutput to simulate
        """
    def getDutyCycle(self) -> float:
        """
        Read the duty cycle value.
        
        :returns: the duty cycle value of this PWM output
        """
    def getInitialized(self) -> bool:
        """
        Check whether this PWM output has been initialized.
        
        :returns: true if initialized
        """
    def getPin(self) -> int:
        """
        Check the pin number.
        
        :returns: the pin number
        """
    def registerDutyCycleCallback(self, callback: typing.Callable[[str, hal._wpiHal.Value], None], initialNotify: bool) -> CallbackStore:
        """
        Register a callback to be run whenever the duty cycle value changes.
        
        :param callback:      the callback
        :param initialNotify: whether to call the callback with the initial state
        
        :returns: the CallbackStore object associated with this callback
        """
    def registerInitializedCallback(self, callback: typing.Callable[[str, hal._wpiHal.Value], None], initialNotify: bool) -> CallbackStore:
        """
        Register a callback to be run when this PWM output is initialized.
        
        :param callback:      the callback
        :param initialNotify: whether to run the callback with the initial state
        
        :returns: the CallbackStore object associated with this callback
        """
    def registerPinCallback(self, callback: typing.Callable[[str, hal._wpiHal.Value], None], initialNotify: bool) -> CallbackStore:
        """
        Register a callback to be run whenever the pin changes.
        
        :param callback:      the callback
        :param initialNotify: whether to call the callback with the initial state
        
        :returns: the CallbackStore object associated with this callback
        """
    def resetData(self) -> None:
        """
        Reset all simulation data.
        """
    def setDutyCycle(self, dutyCycle: float) -> None:
        """
        Set the duty cycle value of this PWM output.
        
        :param dutyCycle: the new value
        """
    def setInitialized(self, initialized: bool) -> None:
        """
        Define whether this PWM output has been initialized.
        
        :param initialized: whether this object is initialized
        """
    def setPin(self, pin: int) -> None:
        """
        Change the pin number.
        
        :param pin: the new pin number
        """
class DoubleSolenoidSim:
    @typing.overload
    def __init__(self, moduleSim: PneumaticsBaseSim, fwd: int, rev: int) -> None:
        ...
    @typing.overload
    def __init__(self, module: int, type: wpilib._wpilib.PneumaticsModuleType, fwd: int, rev: int) -> None:
        ...
    @typing.overload
    def __init__(self, type: wpilib._wpilib.PneumaticsModuleType, fwd: int, rev: int) -> None:
        ...
    def get(self) -> wpilib._wpilib.DoubleSolenoid.Value:
        ...
    def getModuleSim(self) -> PneumaticsBaseSim:
        ...
    def set(self, output: wpilib._wpilib.DoubleSolenoid.Value) -> None:
        ...
class DriverStationSim:
    """
    Class to control a simulated driver station.
    """
    @staticmethod
    def getAllianceStationId() -> hal._wpiHal.AllianceStationID:
        """
        Get the alliance station ID (color + number).
        
        :returns: the alliance station color and number
        """
    @staticmethod
    def getAutonomous() -> bool:
        """
        Check if the DS is in autonomous.
        
        :returns: true if autonomous
        """
    @staticmethod
    def getDsAttached() -> bool:
        """
        Check if the DS is attached.
        
        :returns: true if attached
        """
    @staticmethod
    def getEStop() -> bool:
        """
        Check if eStop has been activated.
        
        :returns: true if eStopped
        """
    @staticmethod
    def getEnabled() -> bool:
        """
        Check if the DS is enabled.
        
        :returns: true if enabled
        """
    @staticmethod
    def getFmsAttached() -> bool:
        """
        Check if the FMS is connected.
        
        :returns: true if FMS is connected
        """
    @staticmethod
    def getJoystickOutputs(stick: int) -> int:
        """
        Gets the joystick outputs.
        
        :param stick: The joystick number
        
        :returns: The joystick outputs
        """
    @staticmethod
    def getJoystickRumble(stick: int, rumbleNum: int) -> int:
        """
        Gets the joystick rumble.
        
        :param stick:     The joystick number
        :param rumbleNum: Rumble to get (0=left, 1=right)
        
        :returns: The joystick rumble value
        """
    @staticmethod
    def getMatchTime() -> float:
        """
        Get the current value of the match timer.
        
        :returns: the current match time
        """
    @staticmethod
    def getTest() -> bool:
        """
        Check if the DS is in test.
        
        :returns: true if test
        """
    @staticmethod
    def notifyNewData() -> None:
        """
        Updates DriverStation data so that new values are visible to the user
        program.
        """
    @staticmethod
    def registerAllianceStationIdCallback(callback: typing.Callable[[str, hal._wpiHal.Value], None], initialNotify: bool) -> CallbackStore:
        """
        Register a callback on the alliance station ID.
        
        :param callback:      the callback that will be called whenever the alliance
                              station changes
        :param initialNotify: if true, the callback will be run on the initial value
        
        :returns: the CallbackStore object associated with this callback
        """
    @staticmethod
    def registerAutonomousCallback(callback: typing.Callable[[str, hal._wpiHal.Value], None], initialNotify: bool) -> CallbackStore:
        """
        Register a callback on whether the DS is in autonomous mode.
        
        :param callback:      the callback that will be called on autonomous mode
                              entrance/exit
        :param initialNotify: if true, the callback will be run on the initial value
        
        :returns: the CallbackStore object associated with this callback
        """
    @staticmethod
    def registerDsAttachedCallback(callback: typing.Callable[[str, hal._wpiHal.Value], None], initialNotify: bool) -> CallbackStore:
        """
        Register a callback on whether the DS is connected.
        
        :param callback:      the callback that will be called whenever the DS
                              connection changes
        :param initialNotify: if true, the callback will be run on the initial value
        
        :returns: the CallbackStore object associated with this callback
        """
    @staticmethod
    def registerEStopCallback(callback: typing.Callable[[str, hal._wpiHal.Value], None], initialNotify: bool) -> CallbackStore:
        """
        Register a callback on the eStop state.
        
        :param callback:      the callback that will be called whenever the eStop state
                              changes
        :param initialNotify: if true, the callback will be run on the initial value
        
        :returns: the CallbackStore object associated with this callback
        """
    @staticmethod
    def registerEnabledCallback(callback: typing.Callable[[str, hal._wpiHal.Value], None], initialNotify: bool) -> CallbackStore:
        """
        Register a callback on whether the DS is enabled.
        
        :param callback:      the callback that will be called whenever the enabled
                              state is changed
        :param initialNotify: if true, the callback will be run on the initial value
        
        :returns: the CallbackStore object associated with this callback
        """
    @staticmethod
    def registerFmsAttachedCallback(callback: typing.Callable[[str, hal._wpiHal.Value], None], initialNotify: bool) -> CallbackStore:
        """
        Register a callback on whether the FMS is connected.
        
        :param callback:      the callback that will be called whenever the FMS
                              connection changes
        :param initialNotify: if true, the callback will be run on the initial value
        
        :returns: the CallbackStore object associated with this callback
        """
    @staticmethod
    def registerMatchTimeCallback(callback: typing.Callable[[str, hal._wpiHal.Value], None], initialNotify: bool) -> CallbackStore:
        """
        Register a callback on match time.
        
        :param callback:      the callback that will be called whenever match time
                              changes
        :param initialNotify: if true, the callback will be run on the initial value
        
        :returns: the CallbackStore object associated with this callback
        """
    @staticmethod
    def registerTestCallback(callback: typing.Callable[[str, hal._wpiHal.Value], None], initialNotify: bool) -> CallbackStore:
        """
        Register a callback on whether the DS is in test mode.
        
        :param callback:      the callback that will be called whenever the test mode
                              is entered or left
        :param initialNotify: if true, the callback will be run on the initial value
        
        :returns: the CallbackStore object associated with this callback
        """
    @staticmethod
    def resetData() -> None:
        """
        Reset all simulation data for the Driver Station.
        """
    @staticmethod
    def setAllianceStationId(allianceStationId: hal._wpiHal.AllianceStationID) -> None:
        """
        Change the alliance station.
        
        :param allianceStationId: the new alliance station
        """
    @staticmethod
    def setAutonomous(autonomous: bool) -> None:
        """
        Change whether the DS is in autonomous.
        
        :param autonomous: the new value
        """
    @staticmethod
    def setDsAttached(dsAttached: bool) -> None:
        """
        Change whether the DS is attached.
        
        :param dsAttached: the new value
        """
    @staticmethod
    def setEStop(eStop: bool) -> None:
        """
        Set whether eStop is active.
        
        :param eStop: true to activate
        """
    @staticmethod
    def setEnabled(enabled: bool) -> None:
        """
        Change whether the DS is enabled.
        
        :param enabled: the new value
        """
    @staticmethod
    def setEventName(name: str) -> None:
        """
        Sets the event name.
        
        :param name: the event name
        """
    @staticmethod
    def setFmsAttached(fmsAttached: bool) -> None:
        """
        Change whether the FMS is connected.
        
        :param fmsAttached: the new value
        """
    @staticmethod
    def setGameSpecificMessage(message: str) -> None:
        """
        Sets the game specific message.
        
        :param message: the game specific message
        """
    @staticmethod
    def setJoystickAxis(stick: int, axis: int, value: float) -> None:
        """
        Gets the value of the axis on a joystick.
        
        :param stick: The joystick number
        :param axis:  The analog axis number
        :param value: The value of the axis on the joystick
        """
    @staticmethod
    def setJoystickAxisCount(stick: int, count: int) -> None:
        """
        Sets the number of axes for a joystick.
        
        :param stick: The joystick number
        :param count: The number of axes on the indicated joystick
        """
    @staticmethod
    def setJoystickAxisType(stick: int, axis: int, type: int) -> None:
        """
        Sets the types of Axes for a joystick.
        
        :param stick: The joystick number
        :param axis:  The target axis
        :param type:  The type of axis
        """
    @staticmethod
    def setJoystickButton(stick: int, button: int, state: bool) -> None:
        """
        Sets the state of one joystick button. %Button indexes begin at 1.
        
        :param stick:  The joystick number
        :param button: The button index, beginning at 1
        :param state:  The state of the joystick button
        """
    @staticmethod
    def setJoystickButtonCount(stick: int, count: int) -> None:
        """
        Sets the number of buttons for a joystick.
        
        :param stick: The joystick number
        :param count: The number of buttons on the indicated joystick
        """
    @staticmethod
    def setJoystickButtons(stick: int, buttons: int) -> None:
        """
        Sets the state of all the buttons on a joystick.
        
        :param stick:   The joystick number
        :param buttons: The bitmap state of the buttons on the joystick
        """
    @staticmethod
    def setJoystickIsXbox(stick: int, isXbox: bool) -> None:
        """
        Sets the value of isXbox for a joystick.
        
        :param stick:  The joystick number
        :param isXbox: The value of isXbox
        """
    @staticmethod
    def setJoystickName(stick: int, name: str) -> None:
        """
        Sets the name of a joystick.
        
        :param stick: The joystick number
        :param name:  The value of name
        """
    @staticmethod
    def setJoystickPOV(stick: int, pov: int, value: int) -> None:
        """
        Gets the state of a POV on a joystick.
        
        :param stick: The joystick number
        :param pov:   The POV number
        :param value: the angle of the POV in degrees, or -1 for not pressed
        """
    @staticmethod
    def setJoystickPOVCount(stick: int, count: int) -> None:
        """
        Sets the number of POVs for a joystick.
        
        :param stick: The joystick number
        :param count: The number of POVs on the indicated joystick
        """
    @staticmethod
    def setJoystickType(stick: int, type: int) -> None:
        """
        Sets the value of type for a joystick.
        
        :param stick: The joystick number
        :param type:  The value of type
        """
    @staticmethod
    def setMatchNumber(matchNumber: int) -> None:
        """
        Sets the match number.
        
        :param matchNumber: the match number
        """
    @staticmethod
    def setMatchTime(matchTime: float) -> None:
        """
        Sets the match timer.
        
        :param matchTime: the new match time
        """
    @staticmethod
    def setMatchType(type: wpilib._wpilib.DriverStation.MatchType) -> None:
        """
        Sets the match type.
        
        :param type: the match type
        """
    @staticmethod
    def setReplayNumber(replayNumber: int) -> None:
        """
        Sets the replay number.
        
        :param replayNumber: the replay number
        """
    @staticmethod
    def setSendConsoleLine(shouldSend: bool) -> None:
        """
        Sets suppression of DriverStation::SendConsoleLine messages.
        
        :param shouldSend: If false then messages will be suppressed.
        """
    @staticmethod
    def setSendError(shouldSend: bool) -> None:
        """
        Sets suppression of DriverStation::ReportError and ReportWarning messages.
        
        :param shouldSend: If false then messages will be suppressed.
        """
    @staticmethod
    def setTest(test: bool) -> None:
        """
        Change whether the DS is in test.
        
        :param test: the new value
        """
    def __init__(self) -> None:
        ...
class DutyCycleEncoderSim:
    """
    Class to control a simulated duty cycle encoder.
    """
    @typing.overload
    def __init__(self, encoder: wpilib._wpilib.DutyCycleEncoder) -> None:
        """
        Constructs from a DutyCycleEncoder object.
        
        :param encoder: DutyCycleEncoder to simulate
        """
    @typing.overload
    def __init__(self, channel: int) -> None:
        """
        Constructs from a digital input channel.
        
        :param channel: digital input channel
        """
    def get(self) -> float:
        """
        Get the position in turns.
        
        :returns: The position.
        """
    def getAbsolutePosition(self) -> float:
        """
        Get the absolute position.
        
        :returns: The absolute position
        """
    def getDistance(self) -> float:
        ...
    def getDistancePerRotation(self) -> float:
        """
        Get the distance per rotation for this encoder.
        
        :returns: The scale factor that will be used to convert rotation to useful
                  units.
        """
    def isConnected(self) -> bool:
        """
        Get if the encoder is connected.
        
        :returns: true if the encoder is connected.
        """
    def set(self, turns: wpimath.units.turns) -> None:
        """
        Set the position in turns.
        
        :param turns: The position.
        """
    def setAbsolutePosition(self, position: float) -> None:
        """
        Set the absolute position.
        
        :param position: The absolute position
        """
    def setConnected(self, isConnected: bool) -> None:
        """
        Set if the encoder is connected.
        
        :param isConnected: Whether or not the sensor is connected.
        """
    def setDistance(self, distance: float) -> None:
        """
        Set the distance.
        
        :param distance: The distance.
        """
class DutyCycleSim:
    """
    Class to control a simulated duty cycle digital input.
    """
    @staticmethod
    def createForChannel(channel: int) -> DutyCycleSim:
        """
        Creates a DutyCycleSim for a digital input channel.
        
        :param channel: digital input channel
        
        :returns: Simulated object
                  @throws std::out_of_range if no DutyCycle is configured for that channel
        """
    @staticmethod
    def createForIndex(index: int) -> DutyCycleSim:
        """
        Creates a DutyCycleSim for a simulated index.
        The index is incremented for each simulated DutyCycle.
        
        :param index: simulator index
        
        :returns: Simulated object
        """
    def __init__(self, dutyCycle: wpilib._wpilib.DutyCycle) -> None:
        """
        Constructs from a DutyCycle object.
        
        :param dutyCycle: DutyCycle to simulate
        """
    def getFrequency(self) -> int:
        """
        Measure the frequency.
        
        :returns: the duty cycle frequency
        """
    def getInitialized(self) -> bool:
        """
        Check whether this duty cycle input has been initialized.
        
        :returns: true if initialized
        """
    def getOutput(self) -> float:
        """
        Measure the output from this duty cycle port.
        
        :returns: the output value
        """
    def registerFrequencyCallback(self, callback: typing.Callable[[str, hal._wpiHal.Value], None], initialNotify: bool) -> CallbackStore:
        """
        Register a callback to be run whenever the frequency changes.
        
        :param callback:      the callback
        :param initialNotify: whether to call the callback with the initial state
        
        :returns: the CallbackStore object associated with this callback
        """
    def registerInitializedCallback(self, callback: typing.Callable[[str, hal._wpiHal.Value], None], initialNotify: bool) -> CallbackStore:
        """
        Register a callback to be run when this duty cycle input is initialized.
        
        :param callback:      the callback
        :param initialNotify: whether to run the callback with the initial state
        
        :returns: the CallbackStore object associated with this callback
        """
    def registerOutputCallback(self, callback: typing.Callable[[str, hal._wpiHal.Value], None], initialNotify: bool) -> CallbackStore:
        """
        Register a callback to be run whenever the output changes.
        
        :param callback:      the callback
        :param initialNotify: whether to call the callback with the initial state
        
        :returns: the CallbackStore object associated with this callback
        """
    def resetData(self) -> None:
        """
        Reset all simulation data for the duty cycle output.
        """
    def setFrequency(self, frequency: int) -> None:
        """
        Change the duty cycle frequency.
        
        :param frequency: the new frequency
        """
    def setInitialized(self, initialized: bool) -> None:
        """
        Define whether this duty cycle input has been initialized.
        
        :param initialized: whether this object is initialized
        """
    def setOutput(self, output: float) -> None:
        """
        Change the duty cycle output.
        
        :param output: the new output value
        """
class ElevatorSim(LinearSystemSim_2_1_1):
    """
    Represents a simulated elevator mechanism.
    """
    @typing.overload
    def __init__(self, plant: wpimath._controls._controls.system.LinearSystem_2_1_1, gearbox: wpimath._controls._controls.plant.DCMotor, minHeight: wpimath.units.meters, maxHeight: wpimath.units.meters, simulateGravity: bool, startingHeight: wpimath.units.meters, measurementStdDevs: typing.Annotated[list[float], pybind11_stubgen.typing_ext.FixedSize(1)] = [0.0]) -> None:
        """
        Constructs a simulated elevator mechanism.
        
        :param plant:              The linear system that represents the elevator.
                                   This system can be created with
                                   LinearSystemId::ElevatorSystem().
        :param gearbox:            The type of and number of motors in your
                                   elevator gearbox.
        :param minHeight:          The minimum allowed height of the elevator.
        :param maxHeight:          The maximum allowed height of the elevator.
        :param simulateGravity:    Whether gravity should be simulated or not.
        :param startingHeight:     The starting height of the elevator.
        :param measurementStdDevs: The standard deviation of the measurements.
        """
    @typing.overload
    def __init__(self, gearbox: wpimath._controls._controls.plant.DCMotor, gearing: float, carriageMass: wpimath.units.kilograms, drumRadius: wpimath.units.meters, minHeight: wpimath.units.meters, maxHeight: wpimath.units.meters, simulateGravity: bool, startingHeight: wpimath.units.meters, measurementStdDevs: typing.Annotated[list[float], pybind11_stubgen.typing_ext.FixedSize(1)] = [0.0]) -> None:
        """
        Constructs a simulated elevator mechanism.
        
        :param gearbox:            The type of and number of motors in your
                                   elevator gearbox.
        :param gearing:            The gearing of the elevator (numbers greater
                                   than 1 represent reductions).
        :param carriageMass:       The mass of the elevator carriage.
        :param drumRadius:         The radius of the drum that your cable is
                                   wrapped around.
        :param minHeight:          The minimum allowed height of the elevator.
        :param maxHeight:          The maximum allowed height of the elevator.
        :param simulateGravity:    Whether gravity should be simulated or not.
        :param startingHeight:     The starting height of the elevator.
        :param measurementStdDevs: The standard deviation of the measurements.
        """
    def _updateX(self, currentXhat: numpy.ndarray[numpy.float64[2, 1]], u: numpy.ndarray[numpy.float64[1, 1]], dt: wpimath.units.seconds) -> numpy.ndarray[numpy.float64[2, 1]]:
        """
        Updates the state estimate of the elevator.
        
        :param currentXhat: The current state estimate.
        :param u:           The system inputs (voltage).
        :param dt:          The time difference between controller updates.
        """
    def getCurrentDraw(self) -> wpimath.units.amperes:
        """
        Returns the elevator current draw.
        
        :returns: The elevator current draw.
        """
    def getPosition(self) -> wpimath.units.meters:
        """
        Returns the position of the elevator.
        
        :returns: The position of the elevator.
        """
    def getPositionFeet(self) -> wpimath.units.feet:
        ...
    def getPositionInches(self) -> wpimath.units.inches:
        ...
    def getVelocity(self) -> wpimath.units.meters_per_second:
        """
        Returns the velocity of the elevator.
        
        :returns: The velocity of the elevator.
        """
    def getVelocityFps(self) -> wpimath.units.feet_per_second:
        ...
    def hasHitLowerLimit(self) -> bool:
        """
        Returns whether the elevator has hit the lower limit.
        
        :returns: Whether the elevator has hit the lower limit.
        """
    def hasHitUpperLimit(self) -> bool:
        """
        Returns whether the elevator has hit the upper limit.
        
        :returns: Whether the elevator has hit the upper limit.
        """
    def setInputVoltage(self, voltage: wpimath.units.volts) -> None:
        """
        Sets the input voltage for the elevator.
        
        :param voltage: The input voltage.
        """
    def setState(self, position: wpimath.units.meters, velocity: wpimath.units.meters_per_second) -> None:
        """
        Sets the elevator's state. The new position will be limited between the
        minimum and maximum allowed heights.
        
        :param position: The new position
        :param velocity: The new velocity
        """
    def wouldHitLowerLimit(self, elevatorHeight: wpimath.units.meters) -> bool:
        """
        Returns whether the elevator would hit the lower limit.
        
        :param elevatorHeight: The elevator height.
        
        :returns: Whether the elevator would hit the lower limit.
        """
    def wouldHitUpperLimit(self, elevatorHeight: wpimath.units.meters) -> bool:
        """
        Returns whether the elevator would hit the upper limit.
        
        :param elevatorHeight: The elevator height.
        
        :returns: Whether the elevator would hit the upper limit.
        """
class EncoderSim:
    """
    Class to control a simulated encoder.
    """
    @staticmethod
    def createForChannel(channel: int) -> EncoderSim:
        """
        Creates an EncoderSim for a digital input channel.  Encoders take two
        channels, so either one may be specified.
        
        :param channel: digital input channel
        
        :returns: Simulated object
                  @throws NoSuchElementException if no Encoder is configured for that channel
        """
    @staticmethod
    def createForIndex(index: int) -> EncoderSim:
        """
        Creates an EncoderSim for a simulated index.
        The index is incremented for each simulated Encoder.
        
        :param index: simulator index
        
        :returns: Simulated object
        """
    def __init__(self, encoder: wpilib._wpilib.Encoder) -> None:
        """
        Constructs from an Encoder object.
        
        :param encoder: Encoder to simulate
        """
    def getCount(self) -> int:
        """
        Read the count of the encoder.
        
        :returns: the count
        """
    def getDirection(self) -> bool:
        """
        Get the direction of the encoder.
        
        :returns: the direction of the encoder
        """
    def getDistance(self) -> float:
        """
        Read the distance of the encoder.
        
        :returns: the encoder distance
        """
    def getDistancePerPulse(self) -> float:
        """
        Read the distance per pulse of the encoder.
        
        :returns: the encoder distance per pulse
        """
    def getInitialized(self) -> bool:
        """
        Read the Initialized value of the encoder.
        
        :returns: true if initialized
        """
    def getMaxPeriod(self) -> float:
        """
        Get the max period of the encoder.
        
        :returns: the max period of the encoder
        """
    def getPeriod(self) -> float:
        """
        Read the period of the encoder.
        
        :returns: the encoder period
        """
    def getRate(self) -> float:
        """
        Get the rate of the encoder.
        
        :returns: the rate of change
        """
    def getReset(self) -> bool:
        """
        Check if the encoder has been reset.
        
        :returns: true if reset
        """
    def getReverseDirection(self) -> bool:
        """
        Get the reverse direction of the encoder.
        
        :returns: the reverse direction of the encoder
        """
    def getSamplesToAverage(self) -> int:
        """
        Get the samples-to-average value.
        
        :returns: the samples-to-average value
        """
    def registerCountCallback(self, callback: typing.Callable[[str, hal._wpiHal.Value], None], initialNotify: bool) -> CallbackStore:
        """
        Register a callback on the count property of the encoder.
        
        :param callback:      the callback that will be called whenever the count
                              property is changed
        :param initialNotify: if true, the callback will be run on the initial value
        
        :returns: the CallbackStore object associated with this callback
        """
    def registerDirectionCallback(self, callback: typing.Callable[[str, hal._wpiHal.Value], None], initialNotify: bool) -> CallbackStore:
        """
        Register a callback on the direction of the encoder.
        
        :param callback:      the callback that will be called whenever the direction
                              is changed
        :param initialNotify: if true, the callback will be run on the initial value
        
        :returns: the CallbackStore object associated with this callback
        """
    def registerDistancePerPulseCallback(self, callback: typing.Callable[[str, hal._wpiHal.Value], None], initialNotify: bool) -> CallbackStore:
        """
        Register a callback on the distance per pulse value of this encoder.
        
        :param callback:      the callback that will be called whenever the
                              distance per pulse is changed
        :param initialNotify: if true, the callback will be run on the initial value
        
        :returns: the CallbackStore object associated with this callback
        """
    def registerInitializedCallback(self, callback: typing.Callable[[str, hal._wpiHal.Value], None], initialNotify: bool) -> CallbackStore:
        """
        Register a callback on the Initialized property of the encoder.
        
        :param callback:      the callback that will be called whenever the Initialized
                              property is changed
        :param initialNotify: if true, the callback will be run on the initial value
        
        :returns: the CallbackStore object associated with this callback
        """
    def registerMaxPeriodCallback(self, callback: typing.Callable[[str, hal._wpiHal.Value], None], initialNotify: bool) -> CallbackStore:
        """
        Register a callback to be run whenever the max period of the encoder is
        changed.
        
        :param callback:      the callback
        :param initialNotify: whether to run the callback on the initial value
        
        :returns: the CallbackStore object associated with this callback
        """
    def registerPeriodCallback(self, callback: typing.Callable[[str, hal._wpiHal.Value], None], initialNotify: bool) -> CallbackStore:
        """
        Register a callback on the period of the encoder.
        
        :param callback:      the callback that will be called whenever the period is
                              changed
        :param initialNotify: if true, the callback will be run on the initial value
        
        :returns: the CallbackStore object associated with this callback
        """
    def registerResetCallback(self, callback: typing.Callable[[str, hal._wpiHal.Value], None], initialNotify: bool) -> CallbackStore:
        """
        Register a callback to be called whenever the encoder is reset.
        
        :param callback:      the callback
        :param initialNotify: whether to run the callback on the initial value
        
        :returns: the CallbackStore object associated with this callback
        """
    def registerReverseDirectionCallback(self, callback: typing.Callable[[str, hal._wpiHal.Value], None], initialNotify: bool) -> CallbackStore:
        """
        Register a callback on the reverse direction.
        
        :param callback:      the callback that will be called whenever the reverse
                              direction is changed
        :param initialNotify: if true, the callback will be run on the initial value
        
        :returns: the CallbackStore object associated with this callback
        """
    def registerSamplesToAverageCallback(self, callback: typing.Callable[[str, hal._wpiHal.Value], None], initialNotify: bool) -> CallbackStore:
        """
        Register a callback on the samples-to-average value of this encoder.
        
        :param callback:      the callback that will be called whenever the
                              samples-to-average is changed
        :param initialNotify: if true, the callback will be run on the initial value
        
        :returns: the CallbackStore object associated with this callback
        """
    def resetData(self) -> None:
        """
        Resets all simulation data for this encoder.
        """
    def setCount(self, count: int) -> None:
        """
        Change the count of the encoder.
        
        :param count: the new count
        """
    def setDirection(self, direction: bool) -> None:
        """
        Set the direction of the encoder.
        
        :param direction: the new direction
        """
    def setDistance(self, distance: float) -> None:
        """
        Change the encoder distance.
        
        :param distance: the new distance
        """
    def setDistancePerPulse(self, distancePerPulse: float) -> None:
        """
        Change the encoder distance per pulse.
        
        :param distancePerPulse: the new distance per pulse
        """
    def setInitialized(self, initialized: bool) -> None:
        """
        Change the Initialized value of the encoder.
        
        :param initialized: the new value
        """
    def setMaxPeriod(self, maxPeriod: float) -> None:
        """
        Change the max period of the encoder.
        
        :param maxPeriod: the new value
        """
    def setPeriod(self, period: float) -> None:
        """
        Change the encoder period.
        
        :param period: the new period
        """
    def setRate(self, rate: float) -> None:
        """
        Change the rate of the encoder.
        
        :param rate: the new rate
        """
    def setReset(self, reset: bool) -> None:
        """
        Change the reset property of the encoder.
        
        :param reset: the new value
        """
    def setReverseDirection(self, reverseDirection: bool) -> None:
        """
        Set the reverse direction.
        
        :param reverseDirection: the new value
        """
    def setSamplesToAverage(self, samplesToAverage: int) -> None:
        """
        Set the samples-to-average value.
        
        :param samplesToAverage: the new value
        """
class FlywheelSim(LinearSystemSim_1_1_1):
    """
    Represents a simulated flywheel mechanism.
    """
    @typing.overload
    def __init__(self, plant: wpimath._controls._controls.system.LinearSystem_1_1_1, gearbox: wpimath._controls._controls.plant.DCMotor, gearing: float, measurementStdDevs: typing.Annotated[list[float], pybind11_stubgen.typing_ext.FixedSize(1)] = [0.0]) -> None:
        """
        Creates a simulated flywheel mechanism.
        
        :param plant:              The linear system representing the flywheel. This
                                   system can be created with
                                   LinearSystemId::FlywheelSystem().
        :param gearbox:            The type of and number of motors in the flywheel
                                   gearbox.
        :param gearing:            The gearing of the flywheel (numbers greater than
                                   1 represent reductions).
        :param measurementStdDevs: The standard deviation of the measurement noise.
        """
    @typing.overload
    def __init__(self, gearbox: wpimath._controls._controls.plant.DCMotor, gearing: float, moi: wpimath.units.kilogram_square_meters, measurementStdDevs: typing.Annotated[list[float], pybind11_stubgen.typing_ext.FixedSize(1)] = [0.0]) -> None:
        """
        Creates a simulated flywheel mechanism.
        
        :param gearbox:            The type of and number of motors in the flywheel
                                   gearbox.
        :param gearing:            The gearing of the flywheel (numbers greater than
                                   1 represent reductions).
        :param moi:                The moment of inertia of the flywheel.
        :param measurementStdDevs: The standard deviation of the measurement noise.
        """
    def getAngularVelocity(self) -> wpimath.units.radians_per_second:
        """
        Returns the flywheel velocity.
        
        :returns: The flywheel velocity.
        """
    def getCurrentDraw(self) -> wpimath.units.amperes:
        """
        Returns the flywheel current draw.
        
        :returns: The flywheel current draw.
        """
    def setInputVoltage(self, voltage: wpimath.units.volts) -> None:
        """
        Sets the input voltage for the flywheel.
        
        :param voltage: The input voltage.
        """
    def setState(self, velocity: wpimath.units.radians_per_second) -> None:
        """
        Sets the flywheel's state.
        
        :param velocity: The new velocity
        """
class GenericHIDSim:
    """
    Class to control a simulated generic joystick.
    """
    @typing.overload
    def __init__(self, joystick: wpilib.interfaces._interfaces.GenericHID) -> None:
        """
        Constructs from a GenericHID object.
        
        :param joystick: joystick to simulate
        """
    @typing.overload
    def __init__(self, port: int) -> None:
        """
        Constructs from a joystick port number.
        
        :param port: port number
        """
    def getOutput(self, outputNumber: int) -> bool:
        """
        Read the output of a button.
        
        :param outputNumber: the button number
        
        :returns: the value of the button (true = pressed)
        """
    def getOutputs(self) -> int:
        """
        Get the encoded 16-bit integer that passes button values.
        
        :returns: the button values
        """
    def getRumble(self, type: wpilib.interfaces._interfaces.GenericHID.RumbleType) -> float:
        """
        Get the joystick rumble.
        
        :param type: the rumble to read
        
        :returns: the rumble value
        """
    def notifyNewData(self) -> None:
        """
        Updates joystick data so that new values are visible to the user program.
        """
    def setAxisCount(self, count: int) -> None:
        """
        Set the axis count of this device.
        
        :param count: the new axis count
        """
    def setAxisType(self, axis: int, type: int) -> None:
        """
        Set the type of an axis.
        
        :param axis: the axis
        :param type: the type
        """
    def setButtonCount(self, count: int) -> None:
        """
        Set the button count of this device.
        
        :param count: the new button count
        """
    def setName(self, name: str) -> None:
        """
        Set the name of this device.
        
        :param name: the new device name
        """
    @typing.overload
    def setPOV(self, pov: int, value: int) -> None:
        """
        Set the value of a given POV.
        
        :param pov:   the POV to set
        :param value: the new value
        """
    @typing.overload
    def setPOV(self, value: int) -> None:
        """
        Set the value of the default POV (port 0).
        
        :param value: the new value
        """
    def setPOVCount(self, count: int) -> None:
        """
        Set the POV count of this device.
        
        :param count: the new POV count
        """
    def setRawAxis(self, axis: int, value: float) -> None:
        """
        Set the value of a given axis.
        
        :param axis:  the axis to set
        :param value: the new value
        """
    def setRawButton(self, button: int, value: bool) -> None:
        """
        Set the value of a given button.
        
        :param button: the button to set
        :param value:  the new value
        """
    def setType(self, type: wpilib.interfaces._interfaces.GenericHID.HIDType) -> None:
        """
        Set the type of this device.
        
        :param type: the new device type
        """
class JoystickSim(GenericHIDSim):
    """
    Class to control a simulated joystick.
    """
    @typing.overload
    def __init__(self, joystick: wpilib._wpilib.Joystick) -> None:
        """
        Constructs from a Joystick object.
        
        :param joystick: joystick to simulate
        """
    @typing.overload
    def __init__(self, port: int) -> None:
        """
        Constructs from a joystick port number.
        
        :param port: port number
        """
    def setThrottle(self, value: float) -> None:
        """
        Set the throttle value of the joystick.
        
        :param value: the new throttle value
        """
    def setTop(self, state: bool) -> None:
        """
        Set the top state of the joystick.
        
        :param state: the new state
        """
    def setTrigger(self, state: bool) -> None:
        """
        Set the trigger value of the joystick.
        
        :param state: the new value
        """
    def setTwist(self, value: float) -> None:
        """
        Set the twist value of the joystick.
        
        :param value: the new twist value
        """
    def setX(self, value: float) -> None:
        """
        Set the X value of the joystick.
        
        :param value: the new X value
        """
    def setY(self, value: float) -> None:
        """
        Set the Y value of the joystick.
        
        :param value: the new Y value
        """
    def setZ(self, value: float) -> None:
        """
        Set the Z value of the joystick.
        
        :param value: the new Z value
        """
class LinearSystemSim_1_1_1:
    """
    This class helps simulate linear systems. To use this class, do the following
    in the simulationPeriodic() method.
    
    Call the SetInput() method with the inputs to your system (generally
    voltage). Call the Update() method to update the simulation. Set simulated
    sensor readings with the simulated positions in the GetOutput() method.
    
    @tparam States  Number of states of the system.
    @tparam Inputs  Number of inputs to the system.
    @tparam Outputs Number of outputs of the system.
    """
    def __init__(self, system: wpimath._controls._controls.system.LinearSystem_1_1_1, measurementStdDevs: typing.Annotated[list[float], pybind11_stubgen.typing_ext.FixedSize(1)] = [0.0]) -> None:
        """
        Creates a simulated generic linear system.
        
        :param system:             The system to simulate.
        :param measurementStdDevs: The standard deviations of the measurements.
        """
    def _clampInput(self, u: numpy.ndarray[numpy.float64[1, 1]]) -> numpy.ndarray[numpy.float64[1, 1]]:
        """
        Clamp the input vector such that no element exceeds the given voltage. If
        any does, the relative magnitudes of the input will be maintained.
        
        :param u: The input vector.
        
        :returns: The normalized input.
        """
    def _updateX(self, currentXhat: numpy.ndarray[numpy.float64[1, 1]], u: numpy.ndarray[numpy.float64[1, 1]], dt: wpimath.units.seconds) -> numpy.ndarray[numpy.float64[1, 1]]:
        """
        Updates the state estimate of the system.
        
        :param currentXhat: The current state estimate.
        :param u:           The system inputs (usually voltage).
        :param dt:          The time difference between controller updates.
        """
    def getCurrentDraw(self) -> wpimath.units.amperes:
        """
        Returns the current drawn by this simulated system. Override this method to
        add a custom current calculation.
        
        :returns: The current drawn by this simulated mechanism.
        """
    @typing.overload
    def getOutput(self) -> numpy.ndarray[numpy.float64[1, 1]]:
        """
        Returns the current output of the plant.
        
        :returns: The current output of the plant.
        """
    @typing.overload
    def getOutput(self, row: int) -> float:
        """
        Returns an element of the current output of the plant.
        
        :param row: The row to return.
        
        :returns: An element of the current output of the plant.
        """
    @typing.overload
    def setInput(self, u: numpy.ndarray[numpy.float64[1, 1]]) -> None:
        """
        Sets the system inputs (usually voltages).
        
        :param u: The system inputs.
        """
    @typing.overload
    def setInput(self, row: int, value: float) -> None:
        ...
    def setState(self, state: numpy.ndarray[numpy.float64[1, 1]]) -> None:
        """
        Sets the system state.
        
        :param state: The new state.
        """
    def update(self, dt: wpimath.units.seconds) -> None:
        """
        Updates the simulation.
        
        :param dt: The time between updates.
        """
    @property
    def _m_measurementStdDevs(self) -> typing.Annotated[list[float], pybind11_stubgen.typing_ext.FixedSize(1)]:
        """
        The standard deviations of measurements, used for adding noise to the
        measurements.
        """
    @property
    def _m_plant(self) -> wpimath._controls._controls.system.LinearSystem_1_1_1:
        """
        The plant that represents the linear system.
        """
    @property
    def _m_u(self) -> numpy.ndarray[numpy.float64[1, 1]]:
        """
        Input vector.
        """
    @property
    def _m_x(self) -> numpy.ndarray[numpy.float64[1, 1]]:
        """
        State vector.
        """
    @property
    def _m_y(self) -> numpy.ndarray[numpy.float64[1, 1]]:
        """
        Output vector.
        """
class LinearSystemSim_1_1_2:
    """
    This class helps simulate linear systems. To use this class, do the following
    in the simulationPeriodic() method.
    
    Call the SetInput() method with the inputs to your system (generally
    voltage). Call the Update() method to update the simulation. Set simulated
    sensor readings with the simulated positions in the GetOutput() method.
    
    @tparam States  Number of states of the system.
    @tparam Inputs  Number of inputs to the system.
    @tparam Outputs Number of outputs of the system.
    """
    def __init__(self, system: wpimath._controls._controls.system.LinearSystem_1_1_2, measurementStdDevs: typing.Annotated[list[float], pybind11_stubgen.typing_ext.FixedSize(2)] = [0.0, 0.0]) -> None:
        """
        Creates a simulated generic linear system.
        
        :param system:             The system to simulate.
        :param measurementStdDevs: The standard deviations of the measurements.
        """
    def _clampInput(self, u: numpy.ndarray[numpy.float64[1, 1]]) -> numpy.ndarray[numpy.float64[1, 1]]:
        """
        Clamp the input vector such that no element exceeds the given voltage. If
        any does, the relative magnitudes of the input will be maintained.
        
        :param u: The input vector.
        
        :returns: The normalized input.
        """
    def _updateX(self, currentXhat: numpy.ndarray[numpy.float64[1, 1]], u: numpy.ndarray[numpy.float64[1, 1]], dt: wpimath.units.seconds) -> numpy.ndarray[numpy.float64[1, 1]]:
        """
        Updates the state estimate of the system.
        
        :param currentXhat: The current state estimate.
        :param u:           The system inputs (usually voltage).
        :param dt:          The time difference between controller updates.
        """
    def getCurrentDraw(self) -> wpimath.units.amperes:
        """
        Returns the current drawn by this simulated system. Override this method to
        add a custom current calculation.
        
        :returns: The current drawn by this simulated mechanism.
        """
    @typing.overload
    def getOutput(self) -> numpy.ndarray[numpy.float64[2, 1]]:
        """
        Returns the current output of the plant.
        
        :returns: The current output of the plant.
        """
    @typing.overload
    def getOutput(self, row: int) -> float:
        """
        Returns an element of the current output of the plant.
        
        :param row: The row to return.
        
        :returns: An element of the current output of the plant.
        """
    @typing.overload
    def setInput(self, u: numpy.ndarray[numpy.float64[1, 1]]) -> None:
        """
        Sets the system inputs (usually voltages).
        
        :param u: The system inputs.
        """
    @typing.overload
    def setInput(self, row: int, value: float) -> None:
        ...
    def setState(self, state: numpy.ndarray[numpy.float64[1, 1]]) -> None:
        """
        Sets the system state.
        
        :param state: The new state.
        """
    def update(self, dt: wpimath.units.seconds) -> None:
        """
        Updates the simulation.
        
        :param dt: The time between updates.
        """
    @property
    def _m_measurementStdDevs(self) -> typing.Annotated[list[float], pybind11_stubgen.typing_ext.FixedSize(2)]:
        """
        The standard deviations of measurements, used for adding noise to the
        measurements.
        """
    @property
    def _m_plant(self) -> wpimath._controls._controls.system.LinearSystem_1_1_2:
        """
        The plant that represents the linear system.
        """
    @property
    def _m_u(self) -> numpy.ndarray[numpy.float64[1, 1]]:
        """
        Input vector.
        """
    @property
    def _m_x(self) -> numpy.ndarray[numpy.float64[1, 1]]:
        """
        State vector.
        """
    @property
    def _m_y(self) -> numpy.ndarray[numpy.float64[2, 1]]:
        """
        Output vector.
        """
class LinearSystemSim_2_1_1:
    """
    This class helps simulate linear systems. To use this class, do the following
    in the simulationPeriodic() method.
    
    Call the SetInput() method with the inputs to your system (generally
    voltage). Call the Update() method to update the simulation. Set simulated
    sensor readings with the simulated positions in the GetOutput() method.
    
    @tparam States  Number of states of the system.
    @tparam Inputs  Number of inputs to the system.
    @tparam Outputs Number of outputs of the system.
    """
    def __init__(self, system: wpimath._controls._controls.system.LinearSystem_2_1_1, measurementStdDevs: typing.Annotated[list[float], pybind11_stubgen.typing_ext.FixedSize(1)] = [0.0]) -> None:
        """
        Creates a simulated generic linear system.
        
        :param system:             The system to simulate.
        :param measurementStdDevs: The standard deviations of the measurements.
        """
    def _clampInput(self, u: numpy.ndarray[numpy.float64[1, 1]]) -> numpy.ndarray[numpy.float64[1, 1]]:
        """
        Clamp the input vector such that no element exceeds the given voltage. If
        any does, the relative magnitudes of the input will be maintained.
        
        :param u: The input vector.
        
        :returns: The normalized input.
        """
    def _updateX(self, currentXhat: numpy.ndarray[numpy.float64[2, 1]], u: numpy.ndarray[numpy.float64[1, 1]], dt: wpimath.units.seconds) -> numpy.ndarray[numpy.float64[2, 1]]:
        """
        Updates the state estimate of the system.
        
        :param currentXhat: The current state estimate.
        :param u:           The system inputs (usually voltage).
        :param dt:          The time difference between controller updates.
        """
    def getCurrentDraw(self) -> wpimath.units.amperes:
        """
        Returns the current drawn by this simulated system. Override this method to
        add a custom current calculation.
        
        :returns: The current drawn by this simulated mechanism.
        """
    @typing.overload
    def getOutput(self) -> numpy.ndarray[numpy.float64[1, 1]]:
        """
        Returns the current output of the plant.
        
        :returns: The current output of the plant.
        """
    @typing.overload
    def getOutput(self, row: int) -> float:
        """
        Returns an element of the current output of the plant.
        
        :param row: The row to return.
        
        :returns: An element of the current output of the plant.
        """
    @typing.overload
    def setInput(self, u: numpy.ndarray[numpy.float64[1, 1]]) -> None:
        """
        Sets the system inputs (usually voltages).
        
        :param u: The system inputs.
        """
    @typing.overload
    def setInput(self, row: int, value: float) -> None:
        ...
    def setState(self, state: numpy.ndarray[numpy.float64[2, 1]]) -> None:
        """
        Sets the system state.
        
        :param state: The new state.
        """
    def update(self, dt: wpimath.units.seconds) -> None:
        """
        Updates the simulation.
        
        :param dt: The time between updates.
        """
    @property
    def _m_measurementStdDevs(self) -> typing.Annotated[list[float], pybind11_stubgen.typing_ext.FixedSize(1)]:
        """
        The standard deviations of measurements, used for adding noise to the
        measurements.
        """
    @property
    def _m_plant(self) -> wpimath._controls._controls.system.LinearSystem_2_1_1:
        """
        The plant that represents the linear system.
        """
    @property
    def _m_u(self) -> numpy.ndarray[numpy.float64[1, 1]]:
        """
        Input vector.
        """
    @property
    def _m_x(self) -> numpy.ndarray[numpy.float64[2, 1]]:
        """
        State vector.
        """
    @property
    def _m_y(self) -> numpy.ndarray[numpy.float64[1, 1]]:
        """
        Output vector.
        """
class LinearSystemSim_2_1_2:
    """
    This class helps simulate linear systems. To use this class, do the following
    in the simulationPeriodic() method.
    
    Call the SetInput() method with the inputs to your system (generally
    voltage). Call the Update() method to update the simulation. Set simulated
    sensor readings with the simulated positions in the GetOutput() method.
    
    @tparam States  Number of states of the system.
    @tparam Inputs  Number of inputs to the system.
    @tparam Outputs Number of outputs of the system.
    """
    def __init__(self, system: wpimath._controls._controls.system.LinearSystem_2_1_2, measurementStdDevs: typing.Annotated[list[float], pybind11_stubgen.typing_ext.FixedSize(2)] = [0.0, 0.0]) -> None:
        """
        Creates a simulated generic linear system.
        
        :param system:             The system to simulate.
        :param measurementStdDevs: The standard deviations of the measurements.
        """
    def _clampInput(self, u: numpy.ndarray[numpy.float64[1, 1]]) -> numpy.ndarray[numpy.float64[1, 1]]:
        """
        Clamp the input vector such that no element exceeds the given voltage. If
        any does, the relative magnitudes of the input will be maintained.
        
        :param u: The input vector.
        
        :returns: The normalized input.
        """
    def _updateX(self, currentXhat: numpy.ndarray[numpy.float64[2, 1]], u: numpy.ndarray[numpy.float64[1, 1]], dt: wpimath.units.seconds) -> numpy.ndarray[numpy.float64[2, 1]]:
        """
        Updates the state estimate of the system.
        
        :param currentXhat: The current state estimate.
        :param u:           The system inputs (usually voltage).
        :param dt:          The time difference between controller updates.
        """
    def getCurrentDraw(self) -> wpimath.units.amperes:
        """
        Returns the current drawn by this simulated system. Override this method to
        add a custom current calculation.
        
        :returns: The current drawn by this simulated mechanism.
        """
    @typing.overload
    def getOutput(self) -> numpy.ndarray[numpy.float64[2, 1]]:
        """
        Returns the current output of the plant.
        
        :returns: The current output of the plant.
        """
    @typing.overload
    def getOutput(self, row: int) -> float:
        """
        Returns an element of the current output of the plant.
        
        :param row: The row to return.
        
        :returns: An element of the current output of the plant.
        """
    @typing.overload
    def setInput(self, u: numpy.ndarray[numpy.float64[1, 1]]) -> None:
        """
        Sets the system inputs (usually voltages).
        
        :param u: The system inputs.
        """
    @typing.overload
    def setInput(self, row: int, value: float) -> None:
        ...
    def setState(self, state: numpy.ndarray[numpy.float64[2, 1]]) -> None:
        """
        Sets the system state.
        
        :param state: The new state.
        """
    def update(self, dt: wpimath.units.seconds) -> None:
        """
        Updates the simulation.
        
        :param dt: The time between updates.
        """
    @property
    def _m_measurementStdDevs(self) -> typing.Annotated[list[float], pybind11_stubgen.typing_ext.FixedSize(2)]:
        """
        The standard deviations of measurements, used for adding noise to the
        measurements.
        """
    @property
    def _m_plant(self) -> wpimath._controls._controls.system.LinearSystem_2_1_2:
        """
        The plant that represents the linear system.
        """
    @property
    def _m_u(self) -> numpy.ndarray[numpy.float64[1, 1]]:
        """
        Input vector.
        """
    @property
    def _m_x(self) -> numpy.ndarray[numpy.float64[2, 1]]:
        """
        State vector.
        """
    @property
    def _m_y(self) -> numpy.ndarray[numpy.float64[2, 1]]:
        """
        Output vector.
        """
class LinearSystemSim_2_2_1:
    """
    This class helps simulate linear systems. To use this class, do the following
    in the simulationPeriodic() method.
    
    Call the SetInput() method with the inputs to your system (generally
    voltage). Call the Update() method to update the simulation. Set simulated
    sensor readings with the simulated positions in the GetOutput() method.
    
    @tparam States  Number of states of the system.
    @tparam Inputs  Number of inputs to the system.
    @tparam Outputs Number of outputs of the system.
    """
    def __init__(self, system: wpimath._controls._controls.system.LinearSystem_2_2_1, measurementStdDevs: typing.Annotated[list[float], pybind11_stubgen.typing_ext.FixedSize(1)] = [0.0]) -> None:
        """
        Creates a simulated generic linear system.
        
        :param system:             The system to simulate.
        :param measurementStdDevs: The standard deviations of the measurements.
        """
    def _clampInput(self, u: numpy.ndarray[numpy.float64[2, 1]]) -> numpy.ndarray[numpy.float64[2, 1]]:
        """
        Clamp the input vector such that no element exceeds the given voltage. If
        any does, the relative magnitudes of the input will be maintained.
        
        :param u: The input vector.
        
        :returns: The normalized input.
        """
    def _updateX(self, currentXhat: numpy.ndarray[numpy.float64[2, 1]], u: numpy.ndarray[numpy.float64[2, 1]], dt: wpimath.units.seconds) -> numpy.ndarray[numpy.float64[2, 1]]:
        """
        Updates the state estimate of the system.
        
        :param currentXhat: The current state estimate.
        :param u:           The system inputs (usually voltage).
        :param dt:          The time difference between controller updates.
        """
    def getCurrentDraw(self) -> wpimath.units.amperes:
        """
        Returns the current drawn by this simulated system. Override this method to
        add a custom current calculation.
        
        :returns: The current drawn by this simulated mechanism.
        """
    @typing.overload
    def getOutput(self) -> numpy.ndarray[numpy.float64[1, 1]]:
        """
        Returns the current output of the plant.
        
        :returns: The current output of the plant.
        """
    @typing.overload
    def getOutput(self, row: int) -> float:
        """
        Returns an element of the current output of the plant.
        
        :param row: The row to return.
        
        :returns: An element of the current output of the plant.
        """
    @typing.overload
    def setInput(self, u: numpy.ndarray[numpy.float64[2, 1]]) -> None:
        """
        Sets the system inputs (usually voltages).
        
        :param u: The system inputs.
        """
    @typing.overload
    def setInput(self, row: int, value: float) -> None:
        ...
    def setState(self, state: numpy.ndarray[numpy.float64[2, 1]]) -> None:
        """
        Sets the system state.
        
        :param state: The new state.
        """
    def update(self, dt: wpimath.units.seconds) -> None:
        """
        Updates the simulation.
        
        :param dt: The time between updates.
        """
    @property
    def _m_measurementStdDevs(self) -> typing.Annotated[list[float], pybind11_stubgen.typing_ext.FixedSize(1)]:
        """
        The standard deviations of measurements, used for adding noise to the
        measurements.
        """
    @property
    def _m_plant(self) -> wpimath._controls._controls.system.LinearSystem_2_2_1:
        """
        The plant that represents the linear system.
        """
    @property
    def _m_u(self) -> numpy.ndarray[numpy.float64[2, 1]]:
        """
        Input vector.
        """
    @property
    def _m_x(self) -> numpy.ndarray[numpy.float64[2, 1]]:
        """
        State vector.
        """
    @property
    def _m_y(self) -> numpy.ndarray[numpy.float64[1, 1]]:
        """
        Output vector.
        """
class LinearSystemSim_2_2_2:
    """
    This class helps simulate linear systems. To use this class, do the following
    in the simulationPeriodic() method.
    
    Call the SetInput() method with the inputs to your system (generally
    voltage). Call the Update() method to update the simulation. Set simulated
    sensor readings with the simulated positions in the GetOutput() method.
    
    @tparam States  Number of states of the system.
    @tparam Inputs  Number of inputs to the system.
    @tparam Outputs Number of outputs of the system.
    """
    def __init__(self, system: wpimath._controls._controls.system.LinearSystem_2_2_2, measurementStdDevs: typing.Annotated[list[float], pybind11_stubgen.typing_ext.FixedSize(2)] = [0.0, 0.0]) -> None:
        """
        Creates a simulated generic linear system.
        
        :param system:             The system to simulate.
        :param measurementStdDevs: The standard deviations of the measurements.
        """
    def _clampInput(self, u: numpy.ndarray[numpy.float64[2, 1]]) -> numpy.ndarray[numpy.float64[2, 1]]:
        """
        Clamp the input vector such that no element exceeds the given voltage. If
        any does, the relative magnitudes of the input will be maintained.
        
        :param u: The input vector.
        
        :returns: The normalized input.
        """
    def _updateX(self, currentXhat: numpy.ndarray[numpy.float64[2, 1]], u: numpy.ndarray[numpy.float64[2, 1]], dt: wpimath.units.seconds) -> numpy.ndarray[numpy.float64[2, 1]]:
        """
        Updates the state estimate of the system.
        
        :param currentXhat: The current state estimate.
        :param u:           The system inputs (usually voltage).
        :param dt:          The time difference between controller updates.
        """
    def getCurrentDraw(self) -> wpimath.units.amperes:
        """
        Returns the current drawn by this simulated system. Override this method to
        add a custom current calculation.
        
        :returns: The current drawn by this simulated mechanism.
        """
    @typing.overload
    def getOutput(self) -> numpy.ndarray[numpy.float64[2, 1]]:
        """
        Returns the current output of the plant.
        
        :returns: The current output of the plant.
        """
    @typing.overload
    def getOutput(self, row: int) -> float:
        """
        Returns an element of the current output of the plant.
        
        :param row: The row to return.
        
        :returns: An element of the current output of the plant.
        """
    @typing.overload
    def setInput(self, u: numpy.ndarray[numpy.float64[2, 1]]) -> None:
        """
        Sets the system inputs (usually voltages).
        
        :param u: The system inputs.
        """
    @typing.overload
    def setInput(self, row: int, value: float) -> None:
        ...
    def setState(self, state: numpy.ndarray[numpy.float64[2, 1]]) -> None:
        """
        Sets the system state.
        
        :param state: The new state.
        """
    def update(self, dt: wpimath.units.seconds) -> None:
        """
        Updates the simulation.
        
        :param dt: The time between updates.
        """
    @property
    def _m_measurementStdDevs(self) -> typing.Annotated[list[float], pybind11_stubgen.typing_ext.FixedSize(2)]:
        """
        The standard deviations of measurements, used for adding noise to the
        measurements.
        """
    @property
    def _m_plant(self) -> wpimath._controls._controls.system.LinearSystem_2_2_2:
        """
        The plant that represents the linear system.
        """
    @property
    def _m_u(self) -> numpy.ndarray[numpy.float64[2, 1]]:
        """
        Input vector.
        """
    @property
    def _m_x(self) -> numpy.ndarray[numpy.float64[2, 1]]:
        """
        State vector.
        """
    @property
    def _m_y(self) -> numpy.ndarray[numpy.float64[2, 1]]:
        """
        Output vector.
        """
class PS4ControllerSim(GenericHIDSim):
    """
    Class to control a simulated PS4 controller.
    """
    @typing.overload
    def __init__(self, joystick: wpilib._wpilib.PS4Controller) -> None:
        """
        Constructs from a PS4Controller object.
        
        :param joystick: controller to simulate
        """
    @typing.overload
    def __init__(self, port: int) -> None:
        """
        Constructs from a joystick port number.
        
        :param port: port number
        """
    def setCircleButton(self, value: bool) -> None:
        """
        Change the value of the Circle button on the controller.
        
        :param value: the new value
        """
    def setCrossButton(self, value: bool) -> None:
        """
        Change the value of the Cross button on the controller.
        
        :param value: the new value
        """
    def setL1Button(self, value: bool) -> None:
        """
        Change the value of the L1 button on the controller.
        
        :param value: the new value
        """
    def setL2Axis(self, value: float) -> None:
        """
        Change the L2 axis axis value of the controller.
        
        :param value: the new value
        """
    def setL2Button(self, value: bool) -> None:
        """
        Change the value of the L2 button on the controller.
        
        :param value: the new value
        """
    def setL3Button(self, value: bool) -> None:
        """
        Change the value of the L3 (left stick) button on the controller.
        
        :param value: the new value
        """
    def setLeftX(self, value: float) -> None:
        """
        Change the X axis value of the controller's left stick.
        
        :param value: the new value
        """
    def setLeftY(self, value: float) -> None:
        """
        Change the Y axis value of the controller's left stick.
        
        :param value: the new value
        """
    def setOptionsButton(self, value: bool) -> None:
        """
        Change the value of the Options button on the controller.
        
        :param value: the new value
        """
    def setPSButton(self, value: bool) -> None:
        """
        Change the value of the PS button on the controller.
        
        :param value: the new value
        """
    def setR1Button(self, value: bool) -> None:
        """
        Change the value of the R1 button on the controller.
        
        :param value: the new value
        """
    def setR2Axis(self, value: float) -> None:
        """
        Change the R2 axis value of the controller.
        
        :param value: the new value
        """
    def setR2Button(self, value: bool) -> None:
        """
        Change the value of the R2 button on the controller.
        
        :param value: the new value
        """
    def setR3Button(self, value: bool) -> None:
        """
        Change the value of the R3 (right stick) button on the controller.
        
        :param value: the new value
        """
    def setRightX(self, value: float) -> None:
        """
        Change the X axis value of the controller's right stick.
        
        :param value: the new value
        """
    def setRightY(self, value: float) -> None:
        """
        Change the Y axis value of the controller's right stick.
        
        :param value: the new value
        """
    def setShareButton(self, value: bool) -> None:
        """
        Change the value of the Share button on the controller.
        
        :param value: the new value
        """
    def setSquareButton(self, value: bool) -> None:
        """
        Change the value of the Square button on the controller.
        
        :param value: the new value
        """
    def setTouchpad(self, value: bool) -> None:
        """
        Change the value of the touchpad button on the controller.
        
        :param value: the new value
        """
    def setTriangleButton(self, value: bool) -> None:
        """
        Change the value of the Triangle button on the controller.
        
        :param value: the new value
        """
class PS5ControllerSim(GenericHIDSim):
    """
    Class to control a simulated PS5 controller.
    """
    @typing.overload
    def __init__(self, joystick: wpilib._wpilib.PS5Controller) -> None:
        """
        Constructs from a PS5Controller object.
        
        :param joystick: controller to simulate
        """
    @typing.overload
    def __init__(self, port: int) -> None:
        """
        Constructs from a joystick port number.
        
        :param port: port number
        """
    def setCircleButton(self, value: bool) -> None:
        """
        Change the value of the Circle button on the controller.
        
        :param value: the new value
        """
    def setCreateButton(self, value: bool) -> None:
        """
        Change the value of the Create button on the controller.
        
        :param value: the new value
        """
    def setCrossButton(self, value: bool) -> None:
        """
        Change the value of the Cross button on the controller.
        
        :param value: the new value
        """
    def setL1Button(self, value: bool) -> None:
        """
        Change the value of the L1 button on the controller.
        
        :param value: the new value
        """
    def setL2Axis(self, value: float) -> None:
        """
        Change the L2 axis axis value of the controller.
        
        :param value: the new value
        """
    def setL2Button(self, value: bool) -> None:
        """
        Change the value of the L2 button on the controller.
        
        :param value: the new value
        """
    def setL3Button(self, value: bool) -> None:
        """
        Change the value of the L3 (left stick) button on the controller.
        
        :param value: the new value
        """
    def setLeftX(self, value: float) -> None:
        """
        Change the X axis value of the controller's left stick.
        
        :param value: the new value
        """
    def setLeftY(self, value: float) -> None:
        """
        Change the Y axis value of the controller's left stick.
        
        :param value: the new value
        """
    def setOptionsButton(self, value: bool) -> None:
        """
        Change the value of the Options button on the controller.
        
        :param value: the new value
        """
    def setPSButton(self, value: bool) -> None:
        """
        Change the value of the PS button on the controller.
        
        :param value: the new value
        """
    def setR1Button(self, value: bool) -> None:
        """
        Change the value of the R1 button on the controller.
        
        :param value: the new value
        """
    def setR2Axis(self, value: float) -> None:
        """
        Change the R2 axis value of the controller.
        
        :param value: the new value
        """
    def setR2Button(self, value: bool) -> None:
        """
        Change the value of the R2 button on the controller.
        
        :param value: the new value
        """
    def setR3Button(self, value: bool) -> None:
        """
        Change the value of the R3 (right stick) button on the controller.
        
        :param value: the new value
        """
    def setRightX(self, value: float) -> None:
        """
        Change the X axis value of the controller's right stick.
        
        :param value: the new value
        """
    def setRightY(self, value: float) -> None:
        """
        Change the Y axis value of the controller's right stick.
        
        :param value: the new value
        """
    def setSquareButton(self, value: bool) -> None:
        """
        Change the value of the Square button on the controller.
        
        :param value: the new value
        """
    def setTouchpad(self, value: bool) -> None:
        """
        Change the value of the touchpad button on the controller.
        
        :param value: the new value
        """
    def setTriangleButton(self, value: bool) -> None:
        """
        Change the value of the Triangle button on the controller.
        
        :param value: the new value
        """
class PWMSim:
    """
    Class to control a simulated PWM output.
    """
    @typing.overload
    def __init__(self, pwm: wpilib._wpilib.PWM) -> None:
        """
        Constructs from a PWM object.
        
        :param pwm: PWM to simulate
        """
    @typing.overload
    def __init__(self, motorctrl: wpilib._wpilib.PWMMotorController) -> None:
        """
        Constructs from a PWMMotorController object.
        
        :param motorctrl: PWMMotorController to simulate
        """
    @typing.overload
    def __init__(self, channel: int) -> None:
        """
        Constructs from a PWM channel number.
        
        :param channel: Channel number
        """
    def getInitialized(self) -> bool:
        """
        Check whether the PWM has been initialized.
        
        :returns: true if initialized
        """
    def getPeriodScale(self) -> int:
        """
        Get the PWM period scale.
        
        :returns: the PWM period scale
        """
    def getPosition(self) -> float:
        """
        Get the PWM position.
        
        :returns: the PWM position (0.0 to 1.0)
        """
    def getPulseMicrosecond(self) -> int:
        """
        Get the PWM pulse microsecond value.
        
        :returns: the PWM pulse microsecond value
        """
    def getSpeed(self) -> float:
        """
        Get the PWM speed.
        
        :returns: the PWM speed (-1.0 to 1.0)
        """
    def getZeroLatch(self) -> bool:
        """
        Check whether the PWM is zero latched.
        
        :returns: true if zero latched
        """
    def registerInitializedCallback(self, callback: typing.Callable[[str, hal._wpiHal.Value], None], initialNotify: bool) -> CallbackStore:
        """
        Register a callback to be run when the PWM is initialized.
        
        :param callback:      the callback
        :param initialNotify: whether to run the callback with the initial state
        
        :returns: the CallbackStore object associated with this callback
        """
    def registerPeriodScaleCallback(self, callback: typing.Callable[[str, hal._wpiHal.Value], None], initialNotify: bool) -> CallbackStore:
        """
        Register a callback to be run when the PWM period scale changes.
        
        :param callback:      the callback
        :param initialNotify: whether to run the callback with the initial value
        
        :returns: the CallbackStore object associated with this callback
        """
    def registerPositionCallback(self, callback: typing.Callable[[str, hal._wpiHal.Value], None], initialNotify: bool) -> CallbackStore:
        """
        Register a callback to be run when the PWM position changes.
        
        :param callback:      the callback
        :param initialNotify: whether to run the callback with the initial value
        
        :returns: the CallbackStore object associated with this callback
        """
    def registerPulseMicrosecondCallback(self, callback: typing.Callable[[str, hal._wpiHal.Value], None], initialNotify: bool) -> CallbackStore:
        """
        Register a callback to be run when the PWM pulse microsecond value changes.
        
        :param callback:      the callback
        :param initialNotify: whether to run the callback with the initial value
        
        :returns: the CallbackStore object associated with this callback
        """
    def registerSpeedCallback(self, callback: typing.Callable[[str, hal._wpiHal.Value], None], initialNotify: bool) -> CallbackStore:
        """
        Register a callback to be run when the PWM speed changes.
        
        :param callback:      the callback
        :param initialNotify: whether to run the callback with the initial value
        
        :returns: the CallbackStore object associated with this callback
        """
    def registerZeroLatchCallback(self, callback: typing.Callable[[str, hal._wpiHal.Value], None], initialNotify: bool) -> CallbackStore:
        """
        Register a callback to be run when the PWM zero latch state changes.
        
        :param callback:      the callback
        :param initialNotify: whether to run the callback with the initial state
        
        :returns: the CallbackStore object associated with this callback
        """
    def resetData(self) -> None:
        """
        Reset all simulation data.
        """
    def setInitialized(self, initialized: bool) -> None:
        """
        Define whether the PWM has been initialized.
        
        :param initialized: whether this object is initialized
        """
    def setPeriodScale(self, periodScale: int) -> None:
        """
        Set the PWM period scale.
        
        :param periodScale: the PWM period scale
        """
    def setPosition(self, position: float) -> None:
        """
        Set the PWM position.
        
        :param position: the PWM position (0.0 to 1.0)
        """
    def setPulseMicrosecond(self, microsecondPulseTime: int) -> None:
        """
        Set the PWM pulse microsecond value.
        
        :param microsecondPulseTime: the PWM pulse microsecond value
        """
    def setSpeed(self, speed: float) -> None:
        """
        Set the PWM speed.
        
        :param speed: the PWM speed (-1.0 to 1.0)
        """
    def setZeroLatch(self, zeroLatch: bool) -> None:
        """
        Define whether the PWM has been zero latched.
        
        :param zeroLatch: true to indicate zero latched
        """
class PneumaticsBaseSim:
    @staticmethod
    def getForType(module: int, type: wpilib._wpilib.PneumaticsModuleType) -> PneumaticsBaseSim:
        ...
    @typing.overload
    def __init__(self, index: int) -> None:
        """
        Constructs a PneumaticsBaseSim with the given index.
        
        :param index: The index.
        """
    @typing.overload
    def __init__(self, module: wpilib._wpilib.PneumaticsBase) -> None:
        """
        Constructs a PneumaticsBaseSim for the given module.
        
        :param module: The module.
        """
    def getAllSolenoidOutputs(self) -> int:
        """
        Get the current value of all solenoid outputs.
        
        :returns: the solenoid outputs (1 bit per output)
        """
    def getCompressorCurrent(self) -> float:
        """
        Read the compressor current.
        
        :returns: the current of the compressor connected to this module
        """
    def getCompressorOn(self) -> bool:
        """
        Check if the compressor is on.
        
        :returns: true if the compressor is active
        """
    def getInitialized(self) -> bool:
        """
        Check whether the PCM/PH has been initialized.
        
        :returns: true if initialized
        """
    def getPressureSwitch(self) -> bool:
        """
        Check the value of the pressure switch.
        
        :returns: the pressure switch value
        """
    def getSolenoidOutput(self, channel: int) -> bool:
        """
        Check the solenoid output on a specific channel.
        
        :param channel: the channel to check
        
        :returns: the solenoid output
        """
    def registerCompressorCurrentCallback(self, callback: typing.Callable[[str, hal._wpiHal.Value], None], initialNotify: bool) -> CallbackStore:
        """
        Register a callback to be run whenever the compressor current changes.
        
        :param callback:      the callback
        :param initialNotify: whether to call the callback with the initial state
        
        :returns: the :class:`.CallbackStore` object associated with this callback.
                  Save a reference to this object; it being deconstructed cancels the
                  callback.
        """
    def registerCompressorOnCallback(self, callback: typing.Callable[[str, hal._wpiHal.Value], None], initialNotify: bool) -> CallbackStore:
        """
        Register a callback to be run when the compressor activates.
        
        :param callback:      the callback
        :param initialNotify: whether to run the callback with the initial state
        
        :returns: the :class:`.CallbackStore` object associated with this callback.
                  Save a reference to this object; it being deconstructed cancels the
                  callback.
        """
    def registerInitializedCallback(self, callback: typing.Callable[[str, hal._wpiHal.Value], None], initialNotify: bool) -> CallbackStore:
        """
        Register a callback to be run when the PCM/PH is initialized.
        
        :param callback:      the callback
        :param initialNotify: whether to run the callback with the initial state
        
        :returns: the :class:`.CallbackStore` object associated with this callback.
                  Save a reference to this object; it being deconstructed cancels the
                  callback.
        """
    def registerPressureSwitchCallback(self, callback: typing.Callable[[str, hal._wpiHal.Value], None], initialNotify: bool) -> CallbackStore:
        """
        Register a callback to be run whenever the pressure switch value changes.
        
        :param callback:      the callback
        :param initialNotify: whether the callback should be called with the initial
                              value
        
        :returns: the :class:`.CallbackStore` object associated with this callback.
                  Save a reference to this object; it being deconstructed cancels the
                  callback.
        """
    def registerSolenoidOutputCallback(self, channel: int, callback: typing.Callable[[str, hal._wpiHal.Value], None], initialNotify: bool) -> CallbackStore:
        """
        Register a callback to be run when the solenoid output on a channel
        changes.
        
        :param channel:       the channel to monitor
        :param callback:      the callback
        :param initialNotify: should the callback be run with the initial value
        
        :returns: the :class:`.CallbackStore` object associated with this callback.
                  Save a reference to this object; it being deconstructed cancels the
                  callback.
        """
    def resetData(self) -> None:
        """
        Reset all simulation data for this object.
        """
    def setAllSolenoidOutputs(self, outputs: int) -> None:
        """
        Change all of the solenoid outputs.
        
        :param outputs: the new solenoid outputs (1 bit per output)
        """
    def setCompressorCurrent(self, compressorCurrent: float) -> None:
        """
        Set the compressor current.
        
        :param compressorCurrent: the new compressor current
        """
    def setCompressorOn(self, compressorOn: bool) -> None:
        """
        Set whether the compressor is active.
        
        :param compressorOn: the new value
        """
    def setInitialized(self, initialized: bool) -> None:
        """
        Define whether the PCM/PH has been initialized.
        
        :param initialized: true for initialized
        """
    def setPressureSwitch(self, pressureSwitch: bool) -> None:
        """
        Set the value of the pressure switch.
        
        :param pressureSwitch: the new value
        """
    def setSolenoidOutput(self, channel: int, solenoidOutput: bool) -> None:
        """
        Change the solenoid output on a specific channel.
        
        :param channel:        the channel to check
        :param solenoidOutput: the new solenoid output
        """
    @property
    def _m_index(self) -> int:
        """
        PneumaticsBase index.
        """
class PowerDistributionSim:
    """
    Class to control a simulated Power Distribution Panel (PowerDistribution).
    """
    @typing.overload
    def __init__(self, module: int = 0) -> None:
        """
        Constructs from a PowerDistribution module number (CAN ID).
        
        :param module: module number
        """
    @typing.overload
    def __init__(self, pdp: wpilib._wpilib.PowerDistribution) -> None:
        """
        Constructs from a PowerDistribution object.
        
        :param pdp: PowerDistribution to simulate
        """
    def getAllCurrents(self, length: int) -> float:
        """
        Read the current of all of the PowerDistribution channels.
        
        :param currents: output array; set to the current in each channel. The
                         array must be big enough to hold all the PowerDistribution
                         channels
        :param length:   length of output array
        """
    def getCurrent(self, channel: int) -> float:
        """
        Read the current in one of the PowerDistribution channels.
        
        :param channel: the channel to check
        
        :returns: the current in the given channel
        """
    def getInitialized(self) -> bool:
        """
        Check whether the PowerDistribution has been initialized.
        
        :returns: true if initialized
        """
    def getTemperature(self) -> float:
        """
        Check the temperature of the PowerDistribution.
        
        :returns: the PowerDistribution temperature
        """
    def getVoltage(self) -> float:
        """
        Check the PowerDistribution voltage.
        
        :returns: the PowerDistribution voltage.
        """
    def registerCurrentCallback(self, channel: int, callback: typing.Callable[[str, hal._wpiHal.Value], None], initialNotify: bool) -> CallbackStore:
        """
        Register a callback to be run whenever the current of a specific channel
        changes.
        
        :param channel:       the channel
        :param callback:      the callback
        :param initialNotify: whether to call the callback with the initial state
        
        :returns: the CallbackStore object associated with this callback
        """
    def registerInitializedCallback(self, callback: typing.Callable[[str, hal._wpiHal.Value], None], initialNotify: bool) -> CallbackStore:
        """
        Register a callback to be run when the PowerDistribution is initialized.
        
        :param callback:      the callback
        :param initialNotify: whether to run the callback with the initial state
        
        :returns: the CallbackStore object associated with this callback
        """
    def registerTemperatureCallback(self, callback: typing.Callable[[str, hal._wpiHal.Value], None], initialNotify: bool) -> CallbackStore:
        """
        Register a callback to be run whenever the PowerDistribution temperature
        changes.
        
        :param callback:      the callback
        :param initialNotify: whether to call the callback with the initial state
        
        :returns: the CallbackStore object associated with this callback
        """
    def registerVoltageCallback(self, callback: typing.Callable[[str, hal._wpiHal.Value], None], initialNotify: bool) -> CallbackStore:
        """
        Register a callback to be run whenever the PowerDistribution voltage
        changes.
        
        :param callback:      the callback
        :param initialNotify: whether to call the callback with the initial state
        
        :returns: the CallbackStore object associated with this callback
        """
    def resetData(self) -> None:
        """
        Reset all PowerDistribution simulation data.
        """
    def setAllCurrents(self, currents: float, length: int) -> None:
        """
        Change the current in all of the PowerDistribution channels.
        
        :param currents: array containing the current values for each channel. The
                         array must be big enough to hold all the PowerDistribution
                         channels
        :param length:   length of array
        """
    def setCurrent(self, channel: int, current: float) -> None:
        """
        Change the current in the given channel.
        
        :param channel: the channel to edit
        :param current: the new current for the channel
        """
    def setInitialized(self, initialized: bool) -> None:
        """
        Define whether the PowerDistribution has been initialized.
        
        :param initialized: whether this object is initialized
        """
    def setTemperature(self, temperature: float) -> None:
        """
        Define the PowerDistribution temperature.
        
        :param temperature: the new PowerDistribution temperature
        """
    def setVoltage(self, voltage: float) -> None:
        """
        Set the PowerDistribution voltage.
        
        :param voltage: the new PowerDistribution voltage
        """
class REVPHSim(PneumaticsBaseSim):
    """
    Class to control a simulated Pneumatic Control Module (PCM).
    """
    @typing.overload
    def __init__(self) -> None:
        """
        Constructs with the default PCM module number (CAN ID).
        """
    @typing.overload
    def __init__(self, module: int) -> None:
        """
        Constructs from a PCM module number (CAN ID).
        
        :param module: module number
        """
    @typing.overload
    def __init__(self, pneumatics: wpilib._wpilib.PneumaticsBase) -> None:
        ...
    def getAllSolenoidOutputs(self) -> int:
        ...
    def getCompressorConfigType(self) -> int:
        """
        Check whether the closed loop compressor control is active.
        
        :returns: compressor config type
        """
    def getCompressorCurrent(self) -> float:
        ...
    def getCompressorOn(self) -> bool:
        """
        Check if the compressor is on.
        
        :returns: true if the compressor is active
        """
    def getInitialized(self) -> bool:
        ...
    def getPressureSwitch(self) -> bool:
        ...
    def getSolenoidOutput(self, channel: int) -> bool:
        ...
    def registerCompressorConfigTypeCallback(self, callback: typing.Callable[[str, hal._wpiHal.Value], None], initialNotify: bool) -> CallbackStore:
        """
        Register a callback to be run whenever the closed loop state changes.
        
        :param callback:      the callback
        :param initialNotify: whether the callback should be called with the
                              initial value
        
        :returns: the CallbackStore object associated with this callback
        """
    def registerCompressorCurrentCallback(self, callback: typing.Callable[[str, hal._wpiHal.Value], None], initialNotify: bool) -> CallbackStore:
        ...
    def registerCompressorOnCallback(self, callback: typing.Callable[[str, hal._wpiHal.Value], None], initialNotify: bool) -> CallbackStore:
        ...
    def registerInitializedCallback(self, callback: typing.Callable[[str, hal._wpiHal.Value], None], initialNotify: bool) -> CallbackStore:
        ...
    def registerPressureSwitchCallback(self, callback: typing.Callable[[str, hal._wpiHal.Value], None], initialNotify: bool) -> CallbackStore:
        ...
    def registerSolenoidOutputCallback(self, channel: int, callback: typing.Callable[[str, hal._wpiHal.Value], None], initialNotify: bool) -> CallbackStore:
        ...
    def resetData(self) -> None:
        ...
    def setAllSolenoidOutputs(self, outputs: int) -> None:
        ...
    def setCompressorConfigType(self, compressorConfigType: int) -> None:
        """
        Turn on/off the closed loop control of the compressor.
        
        :param compressorConfigType: compressor config type
        """
    def setCompressorCurrent(self, compressorCurrent: float) -> None:
        ...
    def setCompressorOn(self, compressorOn: bool) -> None:
        """
        Set whether the compressor is active.
        
        :param compressorOn: the new value
        """
    def setInitialized(self, solenoidInitialized: bool) -> None:
        ...
    def setPressureSwitch(self, pressureSwitch: bool) -> None:
        ...
    def setSolenoidOutput(self, channel: int, solenoidOutput: bool) -> None:
        ...
class RelaySim:
    """
    Class to control a simulated relay.
    """
    @typing.overload
    def __init__(self, relay: wpilib._wpilib.Relay) -> None:
        """
        Constructs from a Relay object.
        
        :param relay: Relay to simulate
        """
    @typing.overload
    def __init__(self, channel: int) -> None:
        """
        Constructs from a relay channel number.
        
        :param channel: Channel number
        """
    def getForward(self) -> bool:
        """
        Check whether the forward direction is active.
        
        :returns: true if active
        """
    def getInitializedForward(self) -> bool:
        """
        Check whether the forward direction has been initialized.
        
        :returns: true if initialized
        """
    def getInitializedReverse(self) -> bool:
        """
        Check whether the reverse direction has been initialized.
        
        :returns: true if initialized
        """
    def getReverse(self) -> bool:
        """
        Check whether the reverse direction is active.
        
        :returns: true if active
        """
    def registerForwardCallback(self, callback: typing.Callable[[str, hal._wpiHal.Value], None], initialNotify: bool) -> CallbackStore:
        """
        Register a callback to be run when the forward direction changes state.
        
        :param callback:      the callback
        :param initialNotify: whether to run the callback with the initial state
        
        :returns: the CallbackStore object associated with this callback
        """
    def registerInitializedForwardCallback(self, callback: typing.Callable[[str, hal._wpiHal.Value], None], initialNotify: bool) -> CallbackStore:
        """
        Register a callback to be run when the forward direction is initialized.
        
        :param callback:      the callback
        :param initialNotify: whether to run the callback with the initial state
        
        :returns: the CallbackStore object associated with this callback
        """
    def registerInitializedReverseCallback(self, callback: typing.Callable[[str, hal._wpiHal.Value], None], initialNotify: bool) -> CallbackStore:
        """
        Register a callback to be run when the reverse direction is initialized.
        
        :param callback:      the callback
        :param initialNotify: whether to run the callback with the initial state
        
        :returns: the CallbackStore object associated with this callback
        """
    def registerReverseCallback(self, callback: typing.Callable[[str, hal._wpiHal.Value], None], initialNotify: bool) -> CallbackStore:
        """
        Register a callback to be run when the reverse direction changes state.
        
        :param callback:      the callback
        :param initialNotify: whether to run the callback with the initial state
        
        :returns: the CallbackStore object associated with this callback
        """
    def resetData(self) -> None:
        """
        Reset all simulation data.
        """
    def setForward(self, forward: bool) -> None:
        """
        Set whether the forward direction is active.
        
        :param forward: true to make active
        """
    def setInitializedForward(self, initializedForward: bool) -> None:
        """
        Define whether the forward direction has been initialized.
        
        :param initializedForward: whether this object is initialized
        """
    def setInitializedReverse(self, initializedReverse: bool) -> None:
        """
        Define whether the reverse direction has been initialized.
        
        :param initializedReverse: whether this object is initialized
        """
    def setReverse(self, reverse: bool) -> None:
        """
        Set whether the reverse direction is active.
        
        :param reverse: true to make active
        """
class RoboRioSim:
    """
    A utility class to control a simulated RoboRIO.
    """
    @staticmethod
    def getBrownoutVoltage() -> wpimath.units.volts:
        """
        Measure the brownout voltage.
        
        :returns: the brownout voltage
        """
    @staticmethod
    def getCPUTemp() -> wpimath.units.celsius:
        """
        Get the cpu temp.
        
        :returns: the cpu temp.
        """
    @staticmethod
    def getComments() -> str:
        """
        Get the comments.
        
        :returns: The comments.
        """
    @staticmethod
    def getFPGAButton() -> bool:
        """
        Query the state of the FPGA button.
        
        :returns: the FPGA button state
        """
    @staticmethod
    def getRadioLEDState() -> wpilib._wpilib.RadioLEDState:
        """
        Get the state of the radio led.
        
        :returns: The state of the radio led.
        """
    @staticmethod
    def getSerialNumber() -> str:
        """
        Get the serial number.
        
        :returns: The serial number.
        """
    @staticmethod
    def getTeamNumber() -> int:
        """
        Get the team number.
        
        :returns: the team number.
        """
    @staticmethod
    def getUserActive3V3() -> bool:
        """
        Get the 3.3V rail active state.
        
        :returns: true if the 3.3V rail is active
        """
    @staticmethod
    def getUserActive5V() -> bool:
        """
        Get the 5V rail active state.
        
        :returns: true if the 5V rail is active
        """
    @staticmethod
    def getUserActive6V() -> bool:
        """
        Get the 6V rail active state.
        
        :returns: true if the 6V rail is active
        """
    @staticmethod
    def getUserCurrent3V3() -> wpimath.units.amperes:
        """
        Measure the 3.3V rail current.
        
        :returns: the 3.3V rail current
        """
    @staticmethod
    def getUserCurrent5V() -> wpimath.units.amperes:
        """
        Measure the 5V rail current.
        
        :returns: the 5V rail current
        """
    @staticmethod
    def getUserCurrent6V() -> wpimath.units.amperes:
        """
        Measure the 6V rail current.
        
        :returns: the 6V rail current
        """
    @staticmethod
    def getUserFaults3V3() -> int:
        """
        Get the 3.3V rail number of faults.
        
        :returns: number of faults
        """
    @staticmethod
    def getUserFaults5V() -> int:
        """
        Get the 5V rail number of faults.
        
        :returns: number of faults
        """
    @staticmethod
    def getUserFaults6V() -> int:
        """
        Get the 6V rail number of faults.
        
        :returns: number of faults
        """
    @staticmethod
    def getUserVoltage3V3() -> wpimath.units.volts:
        """
        Measure the 3.3V rail voltage.
        
        :returns: the 3.3V rail voltage
        """
    @staticmethod
    def getUserVoltage5V() -> wpimath.units.volts:
        """
        Measure the 5V rail voltage.
        
        :returns: the 5V rail voltage
        """
    @staticmethod
    def getUserVoltage6V() -> wpimath.units.volts:
        """
        Measure the 6V rail voltage.
        
        :returns: the 6V rail voltage
        """
    @staticmethod
    def getVInCurrent() -> wpimath.units.amperes:
        """
        Measure the Vin current.
        
        :returns: the Vin current
        """
    @staticmethod
    def getVInVoltage() -> wpimath.units.volts:
        """
        Measure the Vin voltage.
        
        :returns: the Vin voltage
        """
    @staticmethod
    def registerBrownoutVoltageCallback(callback: typing.Callable[[str, hal._wpiHal.Value], None], initialNotify: bool) -> CallbackStore:
        """
        Register a callback to be run whenever the brownout voltage changes.
        
        :param callback:      the callback
        :param initialNotify: whether to call the callback with the initial state
        
        :returns: the CallbackStore object associated with this callback
        """
    @staticmethod
    def registerCPUTempCallback(callback: typing.Callable[[str, hal._wpiHal.Value], None], initialNotify: bool) -> CallbackStore:
        """
        Register a callback to be run whenever the cpu temp changes.
        
        :param callback:      the callback
        :param initialNotify: whether to call the callback with the initial state
        
        :returns: the CallbackStore object associated with this callback
        """
    @staticmethod
    def registerFPGAButtonCallback(callback: typing.Callable[[str, hal._wpiHal.Value], None], initialNotify: bool) -> CallbackStore:
        """
        Register a callback to be run when the FPGA button state changes.
        
        :param callback:      the callback
        :param initialNotify: whether to run the callback with the initial state
        
        :returns: the CallbackStore object associated with this callback
        """
    @staticmethod
    def registerRadioLEDStateCallback(callback: typing.Callable[[str, hal._wpiHal.Value], None], initialNotify: bool) -> CallbackStore:
        """
        Register a callback to be run whenever the Radio led state changes.
        
        :param callback:      the callback
        :param initialNotify: whether the callback should be called with the
                              initial value
        
        :returns: the CallbackStore object associated with this callback
        """
    @staticmethod
    def registerTeamNumberCallback(callback: typing.Callable[[str, hal._wpiHal.Value], None], initialNotify: bool) -> CallbackStore:
        """
        Register a callback to be run whenever the team number changes.
        
        :param callback:      the callback
        :param initialNotify: whether to call the callback with the initial state
        
        :returns: the CallbackStore object associated with this callback
        """
    @staticmethod
    def registerUserActive3V3Callback(callback: typing.Callable[[str, hal._wpiHal.Value], None], initialNotify: bool) -> CallbackStore:
        """
        Register a callback to be run whenever the 3.3V rail active state changes.
        
        :param callback:      the callback
        :param initialNotify: whether the callback should be called with the
                              initial state
        
        :returns: the CallbackStore object associated with this callback
        """
    @staticmethod
    def registerUserActive5VCallback(callback: typing.Callable[[str, hal._wpiHal.Value], None], initialNotify: bool) -> CallbackStore:
        """
        Register a callback to be run whenever the 5V rail active state changes.
        
        :param callback:      the callback
        :param initialNotify: whether the callback should be called with the
                              initial state
        
        :returns: the CallbackStore object associated with this callback
        """
    @staticmethod
    def registerUserActive6VCallback(callback: typing.Callable[[str, hal._wpiHal.Value], None], initialNotify: bool) -> CallbackStore:
        """
        Register a callback to be run whenever the 6V rail active state changes.
        
        :param callback:      the callback
        :param initialNotify: whether the callback should be called with the
                              initial state
        
        :returns: the CallbackStore object associated with this callback
        """
    @staticmethod
    def registerUserCurrent3V3Callback(callback: typing.Callable[[str, hal._wpiHal.Value], None], initialNotify: bool) -> CallbackStore:
        """
        Register a callback to be run whenever the 3.3V rail current changes.
        
        :param callback:      the callback
        :param initialNotify: whether the callback should be called with the
                              initial value
        
        :returns: the CallbackStore object associated with this callback
        """
    @staticmethod
    def registerUserCurrent5VCallback(callback: typing.Callable[[str, hal._wpiHal.Value], None], initialNotify: bool) -> CallbackStore:
        """
        Register a callback to be run whenever the 5V rail current changes.
        
        :param callback:      the callback
        :param initialNotify: whether the callback should be called with the
                              initial value
        
        :returns: the CallbackStore object associated with this callback
        """
    @staticmethod
    def registerUserCurrent6VCallback(callback: typing.Callable[[str, hal._wpiHal.Value], None], initialNotify: bool) -> CallbackStore:
        """
        Register a callback to be run whenever the 6V rail current changes.
        
        :param callback:      the callback
        :param initialNotify: whether the callback should be called with the
                              initial value
        
        :returns: the CallbackStore object associated with this callback
        """
    @staticmethod
    def registerUserFaults3V3Callback(callback: typing.Callable[[str, hal._wpiHal.Value], None], initialNotify: bool) -> CallbackStore:
        """
        Register a callback to be run whenever the 3.3V rail number of faults
        changes.
        
        :param callback:      the callback
        :param initialNotify: whether the callback should be called with the
                              initial value
        
        :returns: the CallbackStore object associated with this callback
        """
    @staticmethod
    def registerUserFaults5VCallback(callback: typing.Callable[[str, hal._wpiHal.Value], None], initialNotify: bool) -> CallbackStore:
        """
        Register a callback to be run whenever the 5V rail number of faults
        changes.
        
        :param callback:      the callback
        :param initialNotify: whether the callback should be called with the
                              initial value
        
        :returns: the CallbackStore object associated with this callback
        """
    @staticmethod
    def registerUserFaults6VCallback(callback: typing.Callable[[str, hal._wpiHal.Value], None], initialNotify: bool) -> CallbackStore:
        """
        Register a callback to be run whenever the 6V rail number of faults
        changes.
        
        :param callback:      the callback
        :param initialNotify: whether the callback should be called with the
                              initial value
        
        :returns: the CallbackStore object associated with this callback
        """
    @staticmethod
    def registerUserVoltage3V3Callback(callback: typing.Callable[[str, hal._wpiHal.Value], None], initialNotify: bool) -> CallbackStore:
        """
        Register a callback to be run whenever the 3.3V rail voltage changes.
        
        :param callback:      the callback
        :param initialNotify: whether the callback should be called with the
                              initial value
        
        :returns: the CallbackStore object associated with this callback
        """
    @staticmethod
    def registerUserVoltage5VCallback(callback: typing.Callable[[str, hal._wpiHal.Value], None], initialNotify: bool) -> CallbackStore:
        """
        Register a callback to be run whenever the 5V rail voltage changes.
        
        :param callback:      the callback
        :param initialNotify: whether the callback should be called with the
                              initial value
        
        :returns: the CallbackStore object associated with this callback
        """
    @staticmethod
    def registerUserVoltage6VCallback(callback: typing.Callable[[str, hal._wpiHal.Value], None], initialNotify: bool) -> CallbackStore:
        """
        Register a callback to be run whenever the 6V rail voltage changes.
        
        :param callback:      the callback
        :param initialNotify: whether the callback should be called with the
                              initial value
        
        :returns: the CallbackStore object associated with this callback
        """
    @staticmethod
    def registerVInCurrentCallback(callback: typing.Callable[[str, hal._wpiHal.Value], None], initialNotify: bool) -> CallbackStore:
        """
        Register a callback to be run whenever the Vin current changes.
        
        :param callback:      the callback
        :param initialNotify: whether the callback should be called with the
                              initial value
        
        :returns: the CallbackStore object associated with this callback
        """
    @staticmethod
    def registerVInVoltageCallback(callback: typing.Callable[[str, hal._wpiHal.Value], None], initialNotify: bool) -> CallbackStore:
        """
        Register a callback to be run whenever the Vin voltage changes.
        
        :param callback:      the callback
        :param initialNotify: whether to call the callback with the initial state
        
        :returns: the CallbackStore object associated with this callback
        """
    @staticmethod
    def resetData() -> None:
        """
        Reset all simulation data.
        """
    @staticmethod
    def setBrownoutVoltage(brownoutVoltage: wpimath.units.volts) -> None:
        """
        Define the brownout voltage.
        
        :param brownoutVoltage: the new voltage
        """
    @staticmethod
    def setCPUTemp(cpuTemp: wpimath.units.celsius) -> None:
        """
        Define the cpu temp.
        
        :param cpuTemp: the new cpu temp.
        """
    @staticmethod
    def setComments(comments: str) -> None:
        """
        Set the comments.
        
        :param comments: The comments.
        """
    @staticmethod
    def setFPGAButton(fPGAButton: bool) -> None:
        """
        Define the state of the FPGA button.
        
        :param fPGAButton: the new state
        """
    @staticmethod
    def setRadioLEDState(state: wpilib._wpilib.RadioLEDState) -> None:
        """
        Set the state of the radio led.
        
        :param state: The state of the radio led.
        """
    @staticmethod
    def setSerialNumber(serialNumber: str) -> None:
        """
        Set the serial number.
        
        :param serialNumber: The serial number.
        """
    @staticmethod
    def setTeamNumber(teamNumber: int) -> None:
        """
        Set the team number.
        
        :param teamNumber: the new team number.
        """
    @staticmethod
    def setUserActive3V3(userActive3V3: bool) -> None:
        """
        Set the 3.3V rail active state.
        
        :param userActive3V3: true to make rail active
        """
    @staticmethod
    def setUserActive5V(userActive5V: bool) -> None:
        """
        Set the 5V rail active state.
        
        :param userActive5V: true to make rail active
        """
    @staticmethod
    def setUserActive6V(userActive6V: bool) -> None:
        """
        Set the 6V rail active state.
        
        :param userActive6V: true to make rail active
        """
    @staticmethod
    def setUserCurrent3V3(userCurrent3V3: wpimath.units.amperes) -> None:
        """
        Define the 3.3V rail current.
        
        :param userCurrent3V3: the new current
        """
    @staticmethod
    def setUserCurrent5V(userCurrent5V: wpimath.units.amperes) -> None:
        """
        Define the 5V rail current.
        
        :param userCurrent5V: the new current
        """
    @staticmethod
    def setUserCurrent6V(userCurrent6V: wpimath.units.amperes) -> None:
        """
        Define the 6V rail current.
        
        :param userCurrent6V: the new current
        """
    @staticmethod
    def setUserFaults3V3(userFaults3V3: int) -> None:
        """
        Set the 3.3V rail number of faults.
        
        :param userFaults3V3: number of faults
        """
    @staticmethod
    def setUserFaults5V(userFaults5V: int) -> None:
        """
        Set the 5V rail number of faults.
        
        :param userFaults5V: number of faults
        """
    @staticmethod
    def setUserFaults6V(userFaults6V: int) -> None:
        """
        Set the 6V rail number of faults.
        
        :param userFaults6V: number of faults
        """
    @staticmethod
    def setUserVoltage3V3(userVoltage3V3: wpimath.units.volts) -> None:
        """
        Define the 3.3V rail voltage.
        
        :param userVoltage3V3: the new voltage
        """
    @staticmethod
    def setUserVoltage5V(userVoltage5V: wpimath.units.volts) -> None:
        """
        Define the 5V rail voltage.
        
        :param userVoltage5V: the new voltage
        """
    @staticmethod
    def setUserVoltage6V(userVoltage6V: wpimath.units.volts) -> None:
        """
        Define the 6V rail voltage.
        
        :param userVoltage6V: the new voltage
        """
    @staticmethod
    def setVInCurrent(vInCurrent: wpimath.units.amperes) -> None:
        """
        Define the Vin current.
        
        :param vInCurrent: the new current
        """
    @staticmethod
    def setVInVoltage(vInVoltage: wpimath.units.volts) -> None:
        """
        Define the Vin voltage.
        
        :param vInVoltage: the new voltage
        """
    def __init__(self) -> None:
        ...
class SPIAccelerometerSim:
    def __init__(self, index: int) -> None:
        """
        Construct a new simulation object.
        
        :param index: the HAL index of the accelerometer
        """
    def getActive(self) -> bool:
        """
        Check whether the accelerometer is active.
        
        :returns: true if active
        """
    def getRange(self) -> int:
        """
        Check the range of this accelerometer.
        
        :returns: the accelerometer range
        """
    def getX(self) -> float:
        """
        Measure the X axis value.
        
        :returns: the X axis measurement
        """
    def getY(self) -> float:
        """
        Measure the Y axis value.
        
        :returns: the Y axis measurement
        """
    def getZ(self) -> float:
        """
        Measure the Z axis value.
        
        :returns: the Z axis measurement
        """
    def registerActiveCallback(self, callback: typing.Callable[[str, hal._wpiHal.Value], None], initialNotify: bool) -> CallbackStore:
        """
        Register a callback to be run when this accelerometer activates.
        
        :param callback:      the callback
        :param initialNotify: whether to run the callback with the initial state
        
        :returns: the CallbackStore object associated with this callback
        """
    def registerRangeCallback(self, callback: typing.Callable[[str, hal._wpiHal.Value], None], initialNotify: bool) -> CallbackStore:
        """
        Register a callback to be run whenever the range changes.
        
        :param callback:      the callback
        :param initialNotify: whether to call the callback with the initial state
        
        :returns: the CallbackStore object associated with this callback
        """
    def registerXCallback(self, callback: typing.Callable[[str, hal._wpiHal.Value], None], initialNotify: bool) -> CallbackStore:
        """
        Register a callback to be run whenever the X axis value changes.
        
        :param callback:      the callback
        :param initialNotify: whether to call the callback with the initial state
        
        :returns: the CallbackStore object associated with this callback
        """
    def registerYCallback(self, callback: typing.Callable[[str, hal._wpiHal.Value], None], initialNotify: bool) -> CallbackStore:
        """
        Register a callback to be run whenever the Y axis value changes.
        
        :param callback:      the callback
        :param initialNotify: whether to call the callback with the initial state
        
        :returns: the CallbackStore object associated with this callback
        """
    def registerZCallback(self, callback: typing.Callable[[str, hal._wpiHal.Value], None], initialNotify: bool) -> CallbackStore:
        """
        Register a callback to be run whenever the Z axis value changes.
        
        :param callback:      the callback
        :param initialNotify: whether to call the callback with the initial state
        
        :returns: the CallbackStore object associated with this callback
        """
    def resetData(self) -> None:
        """
        Reset all simulation data of this object.
        """
    def setActive(self, active: bool) -> None:
        """
        Define whether this accelerometer is active.
        
        :param active: the new state
        """
    def setRange(self, range: int) -> None:
        """
        Change the range of this accelerometer.
        
        :param range: the new accelerometer range
        """
    def setX(self, x: float) -> None:
        """
        Change the X axis value of the accelerometer.
        
        :param x: the new reading of the X axis
        """
    def setY(self, y: float) -> None:
        """
        Change the Y axis value of the accelerometer.
        
        :param y: the new reading of the Y axis
        """
    def setZ(self, z: float) -> None:
        """
        Change the Z axis value of the accelerometer.
        
        :param z: the new reading of the Z axis
        """
class SimDeviceSim:
    """
    Interact with a generic simulated device
    
    Any devices that support simulation but don't have a dedicated sim
    object associated with it can be interacted with via this object.
    You just need to know the name of the associated object.
    
    Here are two ways to find the names of available devices:
    
    * The static function :meth:`.enumerateDevices` can give you a list of
      all available devices -- note that the device must be created first
      before this will return any results!
    * When running the WPILib simulation GUI, the names of the 'Other Devices'
      panel are names of devices that you can interact with via this class.
    
    Once you've created a simulated device, you can use the :meth:`.enumerateValues`
    method to determine what values you can interact with.
    
    
    .. note:: WPILib has simulation support for all of its devices. Some
              vendors may only have limited support for simulation -- read
              the vendor's documentation or contact them for more information.
    """
    @staticmethod
    def enumerateDevices(prefix: str = '') -> list[str]:
        """
        Returns a list of available device names
        """
    @staticmethod
    def getEnumOptions(val: hal._wpiHal.SimEnum) -> list[str]:
        """
        Get all options for the given enum.
        
        :param val: the enum
        
        :returns: names of the different values for that enum
        """
    @staticmethod
    def resetData() -> None:
        """
        Reset all SimDevice data.
        """
    @typing.overload
    def __init__(self, name: str) -> None:
        """
        Constructs a SimDeviceSim.
        
        :param name: name of the SimDevice
        """
    @typing.overload
    def __init__(self, name: str, index: int) -> None:
        """
        Constructs a SimDeviceSim.
        
        :param name:  name of the SimDevice
        :param index: device index number to append to name
        """
    @typing.overload
    def __init__(self, name: str, index: int, channel: int) -> None:
        """
        Constructs a SimDeviceSim.
        
        :param name:    name of the SimDevice
        :param index:   device index number to append to name
        :param channel: device channel number to append to name
        """
    @typing.overload
    def __init__(self, handle: int) -> None:
        """
        Constructs a SimDeviceSim.
        
        :param handle: the low level handle for the corresponding SimDevice.
        """
    def enumerateValues(self) -> list[tuple[str, bool]]:
        """
        Returns a list of (name, readonly) tuples of available values for this device
        """
    def getBoolean(self, name: str) -> hal._wpiHal.SimBoolean:
        """
        Retrieves an object that allows you to interact with simulated values
        represented as a boolean.
        """
    def getDouble(self, name: str) -> hal._wpiHal.SimDouble:
        """
        Retrieves an object that allows you to interact with simulated values
        represented as a double.
        """
    def getEnum(self, name: str) -> hal._wpiHal.SimEnum:
        """
        Get the property object with the given name.
        
        :param name: the property name
        
        :returns: the property object
        """
    def getInt(self, name: str) -> hal._wpiHal.SimInt:
        """
        Retrieves an object that allows you to interact with simulated values
        represented as an integer.
        """
    def getLong(self, name: str) -> hal._wpiHal.SimLong:
        """
        Retrieves an object that allows you to interact with simulated values
        represented as a long.
        """
    def getName(self) -> str:
        """
        Get the name of this object.
        
        :returns: name
        """
    def getValue(self, name: str) -> hal._wpiHal.SimValue:
        """
        Provides a readonly mechanism to retrieve all types of device values
        """
class SingleJointedArmSim(LinearSystemSim_2_1_1):
    """
    Represents a simulated arm mechanism.
    """
    @staticmethod
    def estimateMOI(length: wpimath.units.meters, mass: wpimath.units.kilograms) -> wpimath.units.kilogram_square_meters:
        """
        Calculates a rough estimate of the moment of inertia of an arm given its
        length and mass.
        
        :param length: The length of the arm.
        :param mass:   The mass of the arm.
        
        :returns: The calculated moment of inertia.
        """
    @typing.overload
    def __init__(self, system: wpimath._controls._controls.system.LinearSystem_2_1_1, gearbox: wpimath._controls._controls.plant.DCMotor, gearing: float, armLength: wpimath.units.meters, minAngle: wpimath.units.radians, maxAngle: wpimath.units.radians, simulateGravity: bool, startingAngle: wpimath.units.radians, measurementStdDevs: typing.Annotated[list[float], pybind11_stubgen.typing_ext.FixedSize(1)] = [0.0]) -> None:
        """
        Creates a simulated arm mechanism.
        
        :param system:             The system representing this arm. This system can
                                   be created with
                                   LinearSystemId::SingleJointedArmSystem().
        :param gearbox:            The type and number of motors on the arm gearbox.
        :param gearing:            The gear ratio of the arm (numbers greater than 1
                                   represent reductions).
        :param armLength:          The length of the arm.
        :param minAngle:           The minimum angle that the arm is capable of.
        :param maxAngle:           The maximum angle that the arm is capable of.
        :param simulateGravity:    Whether gravity should be simulated or not.
        :param startingAngle:      The initial position of the arm.
        :param measurementStdDevs: The standard deviations of the measurements.
        """
    @typing.overload
    def __init__(self, gearbox: wpimath._controls._controls.plant.DCMotor, gearing: float, moi: wpimath.units.kilogram_square_meters, armLength: wpimath.units.meters, minAngle: wpimath.units.radians, maxAngle: wpimath.units.radians, simulateGravity: bool, startingAngle: wpimath.units.radians, measurementStdDevs: typing.Annotated[list[float], pybind11_stubgen.typing_ext.FixedSize(1)] = [0.0]) -> None:
        """
        Creates a simulated arm mechanism.
        
        :param gearbox:            The type and number of motors on the arm gearbox.
        :param gearing:            The gear ratio of the arm (numbers greater than 1
                                   represent reductions).
        :param moi:                The moment of inertia of the arm. This can be
                                   calculated from CAD software.
        :param armLength:          The length of the arm.
        :param minAngle:           The minimum angle that the arm is capable of.
        :param maxAngle:           The maximum angle that the arm is capable of.
        :param simulateGravity:    Whether gravity should be simulated or not.
        :param startingAngle:      The initial position of the arm.
        :param measurementStdDevs: The standard deviation of the measurement noise.
        """
    def _updateX(self, currentXhat: numpy.ndarray[numpy.float64[2, 1]], u: numpy.ndarray[numpy.float64[1, 1]], dt: wpimath.units.seconds) -> numpy.ndarray[numpy.float64[2, 1]]:
        """
        Updates the state estimate of the arm.
        
        :param currentXhat: The current state estimate.
        :param u:           The system inputs (voltage).
        :param dt:          The time difference between controller updates.
        """
    def getAngle(self) -> wpimath.units.radians:
        """
        Returns the current arm angle.
        
        :returns: The current arm angle.
        """
    def getAngleDegrees(self) -> wpimath.units.degrees:
        ...
    def getCurrentDraw(self) -> wpimath.units.amperes:
        """
        Returns the arm current draw.
        
        :returns: The arm current draw.
        """
    def getVelocity(self) -> wpimath.units.radians_per_second:
        """
        Returns the current arm velocity.
        
        :returns: The current arm velocity.
        """
    def getVelocityDps(self) -> wpimath.units.degrees_per_second:
        ...
    def hasHitLowerLimit(self) -> bool:
        """
        Returns whether the arm has hit the lower limit.
        
        :returns: Whether the arm has hit the lower limit.
        """
    def hasHitUpperLimit(self) -> bool:
        """
        Returns whether the arm has hit the upper limit.
        
        :returns: Whether the arm has hit the upper limit.
        """
    def setInputVoltage(self, voltage: wpimath.units.volts) -> None:
        """
        Sets the input voltage for the arm.
        
        :param voltage: The input voltage.
        """
    def setState(self, angle: wpimath.units.radians, velocity: wpimath.units.radians_per_second) -> None:
        """
        Sets the arm's state. The new angle will be limited between the minimum and
        maximum allowed limits.
        
        :param angle:    The new angle.
        :param velocity: The new angular velocity.
        """
    def wouldHitLowerLimit(self, armAngle: wpimath.units.radians) -> bool:
        """
        Returns whether the arm would hit the lower limit.
        
        :param armAngle: The arm height.
        
        :returns: Whether the arm would hit the lower limit.
        """
    def wouldHitUpperLimit(self, armAngle: wpimath.units.radians) -> bool:
        """
        Returns whether the arm would hit the upper limit.
        
        :param armAngle: The arm height.
        
        :returns: Whether the arm would hit the upper limit.
        """
class SolenoidSim:
    @typing.overload
    def __init__(self, moduleSim: PneumaticsBaseSim, channel: int) -> None:
        ...
    @typing.overload
    def __init__(self, module: int, type: wpilib._wpilib.PneumaticsModuleType, channel: int) -> None:
        ...
    @typing.overload
    def __init__(self, type: wpilib._wpilib.PneumaticsModuleType, channel: int) -> None:
        ...
    def getModuleSim(self) -> PneumaticsBaseSim:
        ...
    def getOutput(self) -> bool:
        ...
    def registerOutputCallback(self, callback: typing.Callable[[str, hal._wpiHal.Value], None], initialNotify: bool) -> CallbackStore:
        """
        Register a callback to be run when the output of this solenoid has changed.
        
        :param callback:      the callback
        :param initialNotify: whether to run the callback with the initial state
        
        :returns: the :class:`.CallbackStore` object associated with this callback.
                  Save a reference to this object; it being deconstructed cancels the
                  callback.
        """
    def setOutput(self, output: bool) -> None:
        ...
class UltrasonicSim:
    """
    Class to control a simulated :class:`.Ultrasonic`.
    """
    @typing.overload
    def __init__(self, ultrasonic: wpilib._wpilib.Ultrasonic) -> None:
        """
        Constructor.
        
        :param ultrasonic: The real ultrasonic to simulate
        """
    @typing.overload
    def __init__(self, ping: int, echo: int) -> None:
        """
        Constructor.
        
        :param ping: unused.
        :param echo: the ultrasonic's echo channel.
        """
    def setRange(self, range: wpimath.units.inches) -> None:
        """
        Sets the range measurement.
        
        :param range: The range.
        """
    def setRangeValid(self, valid: bool) -> None:
        """
        Sets if the range measurement is valid.
        
        :param valid: True if valid
        """
class XboxControllerSim(GenericHIDSim):
    """
    Class to control a simulated Xbox 360 or Xbox One controller.
    """
    @typing.overload
    def __init__(self, joystick: wpilib._wpilib.XboxController) -> None:
        """
        Constructs from a XboxController object.
        
        :param joystick: controller to simulate
        """
    @typing.overload
    def __init__(self, port: int) -> None:
        """
        Constructs from a joystick port number.
        
        :param port: port number
        """
    def setAButton(self, value: bool) -> None:
        """
        Change the value of the A button.
        
        :param value: the new value
        """
    def setBButton(self, value: bool) -> None:
        """
        Change the value of the B button.
        
        :param value: the new value
        """
    def setBackButton(self, value: bool) -> None:
        """
        Change the value of the Back button.
        
        :param value: the new value
        """
    def setLeftBumper(self, value: bool) -> None:
        """
        Change the left bumper value of the joystick.
        
        :param value: the new value
        """
    def setLeftStickButton(self, value: bool) -> None:
        """
        Change the left button value of the joystick.
        
        :param value: the new value
        """
    def setLeftTriggerAxis(self, value: float) -> None:
        """
        Change the left trigger axis value of the joystick.
        
        :param value: the new value
        """
    def setLeftX(self, value: float) -> None:
        """
        Change the X axis value of the controller's left stick.
        
        :param value: the new value
        """
    def setLeftY(self, value: float) -> None:
        """
        Change the Y axis value of the controller's left stick.
        
        :param value: the new value
        """
    def setRightBumper(self, value: bool) -> None:
        """
        Change the right bumper value of the joystick.
        
        :param value: the new value
        """
    def setRightStickButton(self, value: bool) -> None:
        """
        Change the right button value of the joystick.
        
        :param value: the new value
        """
    def setRightTriggerAxis(self, value: float) -> None:
        """
        Change the right trigger axis value of the joystick.
        
        :param value: the new value
        """
    def setRightX(self, value: float) -> None:
        """
        Change the X axis value of the controller's right stick.
        
        :param value: the new value
        """
    def setRightY(self, value: float) -> None:
        """
        Change the Y axis value of the controller's right stick.
        
        :param value: the new value
        """
    def setStartButton(self, value: bool) -> None:
        """
        Change the value of the Start button.
        
        :param value: the new value
        """
    def setXButton(self, value: bool) -> None:
        """
        Change the value of the X button.
        
        :param value: the new value
        """
    def setYButton(self, value: bool) -> None:
        """
        Change the value of the Y button.
        
        :param value: the new value
        """
def _resetMotorSafety() -> None:
    ...
def _resetWpilibSimulationData() -> None:
    ...
def getProgramStarted() -> bool:
    ...
def isTimingPaused() -> bool:
    """
    Check if the simulator time is paused.
    
    :returns: true if paused
    """
def pauseTiming() -> None:
    """
    Pause the simulator time.
    """
def restartTiming() -> None:
    """
    Restart the simulator time.
    """
def resumeTiming() -> None:
    """
    Resume the simulator time.
    """
def setProgramStarted() -> None:
    ...
def setRuntimeType(type: hal._wpiHal.RuntimeType) -> None:
    """
    Override the HAL runtime type (simulated/real).
    
    :param type: runtime type
    """
def stepTiming(delta: wpimath.units.seconds) -> None:
    """
    Advance the simulator time and wait for all notifiers to run.
    
    :param delta: the amount to advance (in seconds)
    """
def stepTimingAsync(delta: wpimath.units.seconds) -> None:
    """
    Advance the simulator time and return immediately.
    
    :param delta: the amount to advance (in seconds)
    """
def waitForProgramStart() -> None:
    ...
