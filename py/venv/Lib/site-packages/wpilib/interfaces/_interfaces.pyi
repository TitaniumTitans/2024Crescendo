from __future__ import annotations
import typing
import wpilib.event._event
import wpimath.geometry._geometry
import wpimath.units
__all__ = ['Accelerometer', 'CounterBase', 'GenericHID', 'Gyro', 'MotorController']
class Accelerometer:
    """
    Interface for 3-axis accelerometers.
    
    :deprecated: This interface is being removed with no replacement.
    """
    class Range:
        """
        Accelerometer range.
        
        Members:
        
          kRange_2G : 2 Gs max.
        
          kRange_4G : 4 Gs max.
        
          kRange_8G : 8 Gs max.
        
          kRange_16G : 16 Gs max.
        """
        __members__: typing.ClassVar[dict[str, Accelerometer.Range]]  # value = {'kRange_2G': <Range.kRange_2G: 0>, 'kRange_4G': <Range.kRange_4G: 1>, 'kRange_8G': <Range.kRange_8G: 2>, 'kRange_16G': <Range.kRange_16G: 3>}
        kRange_16G: typing.ClassVar[Accelerometer.Range]  # value = <Range.kRange_16G: 3>
        kRange_2G: typing.ClassVar[Accelerometer.Range]  # value = <Range.kRange_2G: 0>
        kRange_4G: typing.ClassVar[Accelerometer.Range]  # value = <Range.kRange_4G: 1>
        kRange_8G: typing.ClassVar[Accelerometer.Range]  # value = <Range.kRange_8G: 2>
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
    def __init__(self) -> None:
        ...
    def getX(self) -> float:
        """
        Common interface for getting the x axis acceleration.
        
        :returns: The acceleration along the x axis in g-forces
        """
    def getY(self) -> float:
        """
        Common interface for getting the y axis acceleration.
        
        :returns: The acceleration along the y axis in g-forces
        """
    def getZ(self) -> float:
        """
        Common interface for getting the z axis acceleration.
        
        :returns: The acceleration along the z axis in g-forces
        """
    def setRange(self, range: Accelerometer.Range) -> None:
        """
        Common interface for setting the measuring range of an accelerometer.
        
        :param range: The maximum acceleration, positive or negative, that the
                      accelerometer will measure. Not all accelerometers support all
                      ranges.
        """
class CounterBase:
    """
    Interface for counting the number of ticks on a digital input channel.
    
    Encoders, Gear tooth sensors, and counters should all subclass this so it can
    be used to build more advanced classes for control and driving.
    
    All counters will immediately start counting - Reset() them if you need them
    to be zeroed before use.
    """
    class EncodingType:
        """
        Members:
        
          k1X
        
          k2X
        
          k4X
        """
        __members__: typing.ClassVar[dict[str, CounterBase.EncodingType]]  # value = {'k1X': <EncodingType.k1X: 0>, 'k2X': <EncodingType.k2X: 1>, 'k4X': <EncodingType.k4X: 2>}
        k1X: typing.ClassVar[CounterBase.EncodingType]  # value = <EncodingType.k1X: 0>
        k2X: typing.ClassVar[CounterBase.EncodingType]  # value = <EncodingType.k2X: 1>
        k4X: typing.ClassVar[CounterBase.EncodingType]  # value = <EncodingType.k4X: 2>
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
    def __init__(self) -> None:
        ...
    def get(self) -> int:
        ...
    def getDirection(self) -> bool:
        ...
    def getPeriod(self) -> wpimath.units.seconds:
        ...
    def getStopped(self) -> bool:
        ...
    def reset(self) -> None:
        ...
    def setMaxPeriod(self, maxPeriod: wpimath.units.seconds) -> None:
        ...
class GenericHID:
    """
    Handle input from standard HID devices connected to the Driver Station.
    
    This class handles standard input that comes from the Driver Station. Each
    time a value is requested the most recent value is returned. There is a
    single class instance for each device and the mapping of ports to hardware
    buttons depends on the code in the Driver Station.
    """
    class HIDType:
        """
        USB HID interface type.
        
        Members:
        
          kUnknown : Unknown.
        
          kXInputUnknown : XInputUnknown.
        
          kXInputGamepad : XInputGamepad.
        
          kXInputWheel : XInputWheel.
        
          kXInputArcadeStick : XInputArcadeStick.
        
          kXInputFlightStick : XInputFlightStick.
        
          kXInputDancePad : XInputDancePad.
        
          kXInputGuitar : XInputGuitar.
        
          kXInputGuitar2 : XInputGuitar2.
        
          kXInputDrumKit : XInputDrumKit.
        
          kXInputGuitar3 : XInputGuitar3.
        
          kXInputArcadePad : XInputArcadePad.
        
          kHIDJoystick : HIDJoystick.
        
          kHIDGamepad : HIDGamepad.
        
          kHIDDriving : HIDDriving.
        
          kHIDFlight : HIDFlight.
        
          kHID1stPerson : HID1stPerson.
        """
        __members__: typing.ClassVar[dict[str, GenericHID.HIDType]]  # value = {'kUnknown': <HIDType.kUnknown: -1>, 'kXInputUnknown': <HIDType.kXInputUnknown: 0>, 'kXInputGamepad': <HIDType.kXInputGamepad: 1>, 'kXInputWheel': <HIDType.kXInputWheel: 2>, 'kXInputArcadeStick': <HIDType.kXInputArcadeStick: 3>, 'kXInputFlightStick': <HIDType.kXInputFlightStick: 4>, 'kXInputDancePad': <HIDType.kXInputDancePad: 5>, 'kXInputGuitar': <HIDType.kXInputGuitar: 6>, 'kXInputGuitar2': <HIDType.kXInputGuitar2: 7>, 'kXInputDrumKit': <HIDType.kXInputDrumKit: 8>, 'kXInputGuitar3': <HIDType.kXInputGuitar3: 11>, 'kXInputArcadePad': <HIDType.kXInputArcadePad: 19>, 'kHIDJoystick': <HIDType.kHIDJoystick: 20>, 'kHIDGamepad': <HIDType.kHIDGamepad: 21>, 'kHIDDriving': <HIDType.kHIDDriving: 22>, 'kHIDFlight': <HIDType.kHIDFlight: 23>, 'kHID1stPerson': <HIDType.kHID1stPerson: 24>}
        kHID1stPerson: typing.ClassVar[GenericHID.HIDType]  # value = <HIDType.kHID1stPerson: 24>
        kHIDDriving: typing.ClassVar[GenericHID.HIDType]  # value = <HIDType.kHIDDriving: 22>
        kHIDFlight: typing.ClassVar[GenericHID.HIDType]  # value = <HIDType.kHIDFlight: 23>
        kHIDGamepad: typing.ClassVar[GenericHID.HIDType]  # value = <HIDType.kHIDGamepad: 21>
        kHIDJoystick: typing.ClassVar[GenericHID.HIDType]  # value = <HIDType.kHIDJoystick: 20>
        kUnknown: typing.ClassVar[GenericHID.HIDType]  # value = <HIDType.kUnknown: -1>
        kXInputArcadePad: typing.ClassVar[GenericHID.HIDType]  # value = <HIDType.kXInputArcadePad: 19>
        kXInputArcadeStick: typing.ClassVar[GenericHID.HIDType]  # value = <HIDType.kXInputArcadeStick: 3>
        kXInputDancePad: typing.ClassVar[GenericHID.HIDType]  # value = <HIDType.kXInputDancePad: 5>
        kXInputDrumKit: typing.ClassVar[GenericHID.HIDType]  # value = <HIDType.kXInputDrumKit: 8>
        kXInputFlightStick: typing.ClassVar[GenericHID.HIDType]  # value = <HIDType.kXInputFlightStick: 4>
        kXInputGamepad: typing.ClassVar[GenericHID.HIDType]  # value = <HIDType.kXInputGamepad: 1>
        kXInputGuitar: typing.ClassVar[GenericHID.HIDType]  # value = <HIDType.kXInputGuitar: 6>
        kXInputGuitar2: typing.ClassVar[GenericHID.HIDType]  # value = <HIDType.kXInputGuitar2: 7>
        kXInputGuitar3: typing.ClassVar[GenericHID.HIDType]  # value = <HIDType.kXInputGuitar3: 11>
        kXInputUnknown: typing.ClassVar[GenericHID.HIDType]  # value = <HIDType.kXInputUnknown: 0>
        kXInputWheel: typing.ClassVar[GenericHID.HIDType]  # value = <HIDType.kXInputWheel: 2>
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
    class RumbleType:
        """
        Represents a rumble output on the Joystick.
        
        Members:
        
          kLeftRumble : Left rumble motor.
        
          kRightRumble : Right rumble motor.
        
          kBothRumble : Both left and right rumble motors.
        """
        __members__: typing.ClassVar[dict[str, GenericHID.RumbleType]]  # value = {'kLeftRumble': <RumbleType.kLeftRumble: 0>, 'kRightRumble': <RumbleType.kRightRumble: 1>, 'kBothRumble': <RumbleType.kBothRumble: 2>}
        kBothRumble: typing.ClassVar[GenericHID.RumbleType]  # value = <RumbleType.kBothRumble: 2>
        kLeftRumble: typing.ClassVar[GenericHID.RumbleType]  # value = <RumbleType.kLeftRumble: 0>
        kRightRumble: typing.ClassVar[GenericHID.RumbleType]  # value = <RumbleType.kRightRumble: 1>
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
    @typing.overload
    def POV(self, angle: int, loop: wpilib.event._event.EventLoop) -> wpilib.event._event.BooleanEvent:
        """
        Constructs a BooleanEvent instance based around this angle of a POV on the
        HID.
        
        The POV angles start at 0 in the up direction, and increase clockwise
        (eg right is 90, upper-left is 315).
        
        :param loop:  the event loop instance to attach the event to.
        :param angle: POV angle in degrees, or -1 for the center / not pressed.
        
        :returns: a BooleanEvent instance based around this angle of a POV on the
                  HID.
        """
    @typing.overload
    def POV(self, pov: int, angle: int, loop: wpilib.event._event.EventLoop) -> wpilib.event._event.BooleanEvent:
        """
        Constructs a BooleanEvent instance based around this angle of a POV on the
        HID.
        
        The POV angles start at 0 in the up direction, and increase clockwise
        (eg right is 90, upper-left is 315).
        
        :param loop:  the event loop instance to attach the event to.
        :param pov:   index of the POV to read (starting at 0). Defaults to 0.
        :param angle: POV angle in degrees, or -1 for the center / not pressed.
        
        :returns: a BooleanEvent instance based around this angle of a POV on the
                  HID.
        """
    def POVCenter(self, loop: wpilib.event._event.EventLoop) -> wpilib.event._event.BooleanEvent:
        """
        Constructs a BooleanEvent instance based around the center (not pressed) of
        the default (index 0) POV on the HID.
        
        :returns: a BooleanEvent instance based around the center of a POV on the
                  HID.
        """
    def POVDown(self, loop: wpilib.event._event.EventLoop) -> wpilib.event._event.BooleanEvent:
        """
        Constructs a BooleanEvent instance based around the 180 degree angle (down)
        of the default (index 0) POV on the HID.
        
        :returns: a BooleanEvent instance based around the 180 degree angle of a POV
                  on the HID.
        """
    def POVDownLeft(self, loop: wpilib.event._event.EventLoop) -> wpilib.event._event.BooleanEvent:
        """
        Constructs a BooleanEvent instance based around the 225 degree angle (down
        left) of the default (index 0) POV on the HID.
        
        :returns: a BooleanEvent instance based around the 225 degree angle of a POV
                  on the HID.
        """
    def POVDownRight(self, loop: wpilib.event._event.EventLoop) -> wpilib.event._event.BooleanEvent:
        """
        Constructs a BooleanEvent instance based around the 135 degree angle (right
        down) of the default (index 0) POV on the HID.
        
        :returns: a BooleanEvent instance based around the 135 degree angle of a POV
                  on the HID.
        """
    def POVLeft(self, loop: wpilib.event._event.EventLoop) -> wpilib.event._event.BooleanEvent:
        """
        Constructs a BooleanEvent instance based around the 270 degree angle (left)
        of the default (index 0) POV on the HID.
        
        :returns: a BooleanEvent instance based around the 270 degree angle of a POV
                  on the HID.
        """
    def POVRight(self, loop: wpilib.event._event.EventLoop) -> wpilib.event._event.BooleanEvent:
        """
        Constructs a BooleanEvent instance based around the 90 degree angle (right)
        of the default (index 0) POV on the HID.
        
        :returns: a BooleanEvent instance based around the 90 degree angle of a POV
                  on the HID.
        """
    def POVUp(self, loop: wpilib.event._event.EventLoop) -> wpilib.event._event.BooleanEvent:
        """
        Constructs a BooleanEvent instance based around the 0 degree angle (up) of
        the default (index 0) POV on the HID.
        
        :returns: a BooleanEvent instance based around the 0 degree angle of a POV on
                  the HID.
        """
    def POVUpLeft(self, loop: wpilib.event._event.EventLoop) -> wpilib.event._event.BooleanEvent:
        """
        Constructs a BooleanEvent instance based around the 315 degree angle (left
        up) of the default (index 0) POV on the HID.
        
        :returns: a BooleanEvent instance based around the 315 degree angle of a POV
                  on the HID.
        """
    def POVUpRight(self, loop: wpilib.event._event.EventLoop) -> wpilib.event._event.BooleanEvent:
        """
        Constructs a BooleanEvent instance based around the 45 degree angle (right
        up) of the default (index 0) POV on the HID.
        
        :returns: a BooleanEvent instance based around the 45 degree angle of a POV
                  on the HID.
        """
    def __init__(self, port: int) -> None:
        ...
    def __repr__(self) -> str:
        ...
    def axisGreaterThan(self, axis: int, threshold: float, loop: wpilib.event._event.EventLoop) -> wpilib.event._event.BooleanEvent:
        """
        Constructs an event instance that is true when the axis value is greater
        than threshold
        
        :param axis:      The axis to read, starting at 0.
        :param threshold: The value above which this trigger should return true.
        :param loop:      the event loop instance to attach the event to.
        
        :returns: an event instance that is true when the axis value is greater than
                  the provided threshold.
        """
    def axisLessThan(self, axis: int, threshold: float, loop: wpilib.event._event.EventLoop) -> wpilib.event._event.BooleanEvent:
        """
        Constructs an event instance that is true when the axis value is less than
        threshold
        
        :param axis:      The axis to read, starting at 0.
        :param threshold: The value below which this trigger should return true.
        :param loop:      the event loop instance to attach the event to.
        
        :returns: an event instance that is true when the axis value is less than the
                  provided threshold.
        """
    def button(self, button: int, loop: wpilib.event._event.EventLoop) -> wpilib.event._event.BooleanEvent:
        """
        Constructs an event instance around this button's digital signal.
        
        :param button: the button index
        :param loop:   the event loop instance to attach the event to.
        
        :returns: an event instance representing the button's digital signal attached
                  to the given loop.
        """
    def getAxisCount(self) -> int:
        """
        Get the number of axes for the HID.
        
        :returns: the number of axis for the current HID
        """
    def getAxisType(self, axis: int) -> int:
        """
        Get the axis type of a joystick axis.
        
        :returns: the axis type of a joystick axis.
        """
    def getButtonCount(self) -> int:
        """
        Get the number of buttons for the HID.
        
        :returns: the number of buttons on the current HID
        """
    def getName(self) -> str:
        """
        Get the name of the HID.
        
        :returns: the name of the HID.
        """
    def getPOV(self, pov: int = 0) -> int:
        """
        Get the angle in degrees of a POV on the HID.
        
        The POV angles start at 0 in the up direction, and increase clockwise
        (e.g. right is 90, upper-left is 315).
        
        :param pov: The index of the POV to read (starting at 0)
        
        :returns: the angle of the POV in degrees, or -1 if the POV is not pressed.
        """
    def getPOVCount(self) -> int:
        """
        Get the number of POVs for the HID.
        
        :returns: the number of POVs for the current HID
        """
    def getPort(self) -> int:
        """
        Get the port number of the HID.
        
        :returns: The port number of the HID.
        """
    def getRawAxis(self, axis: int) -> float:
        """
        Get the value of the axis.
        
        :param axis: The axis to read, starting at 0.
        
        :returns: The value of the axis.
        """
    def getRawButton(self, button: int) -> bool:
        """
        Get the button value (starting at button 1).
        
        The buttons are returned in a single 16 bit value with one bit representing
        the state of each button. The appropriate button is returned as a boolean
        value.
        
        This method returns true if the button is being held down at the time
        that this method is being called.
        
        :param button: The button number to be read (starting at 1)
        
        :returns: The state of the button.
        """
    def getRawButtonPressed(self, button: int) -> bool:
        """
        Whether the button was pressed since the last check. %Button indexes begin
        at 1.
        
        This method returns true if the button went from not pressed to held down
        since the last time this method was called. This is useful if you only
        want to call a function once when you press the button.
        
        :param button: The button index, beginning at 1.
        
        :returns: Whether the button was pressed since the last check.
        """
    def getRawButtonReleased(self, button: int) -> bool:
        """
        Whether the button was released since the last check. %Button indexes begin
        at 1.
        
        This method returns true if the button went from held down to not pressed
        since the last time this method was called. This is useful if you only
        want to call a function once when you release the button.
        
        :param button: The button index, beginning at 1.
        
        :returns: Whether the button was released since the last check.
        """
    def getType(self) -> GenericHID.HIDType:
        """
        Get the type of the HID.
        
        :returns: the type of the HID.
        """
    def isConnected(self) -> bool:
        """
        Get if the HID is connected.
        
        :returns: true if the HID is connected
        """
    def setOutput(self, outputNumber: int, value: bool) -> None:
        """
        Set a single HID output value for the HID.
        
        :param outputNumber: The index of the output to set (1-32)
        :param value:        The value to set the output to
        """
    def setOutputs(self, value: int) -> None:
        """
        Set all output values for the HID.
        
        :param value: The 32 bit output value (1 bit for each output)
        """
    def setRumble(self, type: GenericHID.RumbleType, value: float) -> None:
        """
        Set the rumble output for the HID.
        
        The DS currently supports 2 rumble values, left rumble and right rumble.
        
        :param type:  Which rumble value to set
        :param value: The normalized value (0 to 1) to set the rumble to
        """
class Gyro:
    """
    Interface for yaw rate gyros.
    
    :deprecated: This interface is being removed with no replacement.
    """
    def __init__(self) -> None:
        ...
    def calibrate(self) -> None:
        """
        Calibrate the gyro. It's important to make sure that the robot is not
        moving while the calibration is in progress, this is typically
        done when the robot is first turned on while it's sitting at rest before
        the match starts.
        """
    def getAngle(self) -> float:
        """
        Return the heading of the robot in degrees.
        
        The angle is continuous, that is it will continue from 360 to 361 degrees.
        This allows algorithms that wouldn't want to see a discontinuity in the
        gyro output as it sweeps past from 360 to 0 on the second time around.
        
        The angle is expected to increase as the gyro turns clockwise when looked
        at from the top. It needs to follow the NED axis convention.
        
        :returns: the current heading of the robot in degrees. This heading is based
                  on integration of the returned rate from the gyro.
        """
    def getRate(self) -> float:
        """
        Return the rate of rotation of the gyro.
        
        The rate is based on the most recent reading of the gyro analog value.
        
        The rate is expected to be positive as the gyro turns clockwise when looked
        at from the top. It needs to follow the NED axis convention.
        
        :returns: the current rate in degrees per second
        """
    def getRotation2d(self) -> wpimath.geometry._geometry.Rotation2d:
        """
        Return the heading of the robot as a Rotation2d.
        
        The angle is continuous, that is it will continue from 360 to 361 degrees.
        This allows algorithms that wouldn't want to see a discontinuity in the
        gyro output as it sweeps past from 360 to 0 on the second time around.
        
        The angle is expected to increase as the gyro turns counterclockwise when
        looked at from the top. It needs to follow the NWU axis convention.
        
        :returns: the current heading of the robot as a Rotation2d. This heading is
                  based on integration of the returned rate from the gyro.
        """
    def reset(self) -> None:
        """
        Reset the gyro. Resets the gyro to a heading of zero. This can be used if
        there is significant drift in the gyro and it needs to be recalibrated
        after it has been running.
        """
class MotorController:
    """
    Interface for motor controlling devices.
    """
    def __init__(self) -> None:
        ...
    def disable(self) -> None:
        """
        Common interface for disabling a motor.
        """
    def get(self) -> float:
        """
        Common interface for getting the current set speed of a motor controller.
        
        :returns: The current set speed.  Value is between -1.0 and 1.0.
        """
    def getInverted(self) -> bool:
        """
        Common interface for returning the inversion state of a motor controller.
        
        :returns: isInverted The state of inversion, true is inverted.
        """
    def set(self, speed: float) -> None:
        """
        Common interface for setting the speed of a motor controller.
        
        :param speed: The speed to set.  Value should be between -1.0 and 1.0.
        """
    def setInverted(self, isInverted: bool) -> None:
        """
        Common interface for inverting direction of a motor controller.
        
        :param isInverted: The state of inversion, true is inverted.
        """
    def setVoltage(self, output: wpimath.units.volts) -> None:
        """
        Sets the voltage output of the MotorController.  Compensates for
        the current bus voltage to ensure that the desired voltage is output even
        if the battery voltage is below 12V - highly useful when the voltage
        outputs are "meaningful" (e.g. they come from a feedforward calculation).
        
        NOTE: This function *must* be called regularly in order for voltage
        compensation to work properly - unlike the ordinary set function, it is not
        "set it and forget it."
        
        :param output: The voltage to output.
        """
    def stopMotor(self) -> None:
        """
        Common interface to stop the motor until Set is called again.
        """
