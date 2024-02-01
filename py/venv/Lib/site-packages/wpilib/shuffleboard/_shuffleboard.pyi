from __future__ import annotations
import ntcore._ntcore
import typing
import wpiutil._wpiutil
__all__ = ['BuiltInLayouts', 'BuiltInWidgets', 'ComplexWidget', 'LayoutType', 'Shuffleboard', 'ShuffleboardComponentBase', 'ShuffleboardContainer', 'ShuffleboardEventImportance', 'ShuffleboardLayout', 'ShuffleboardTab', 'ShuffleboardValue', 'SimpleWidget', 'SuppliedBoolListValueWidget', 'SuppliedBoolValueWidget', 'SuppliedDoubleListValueWidget', 'SuppliedDoubleValueWidget', 'SuppliedFloatListValueWidget', 'SuppliedFloatValueWidget', 'SuppliedIntListValueWidget', 'SuppliedIntegerValueWidget', 'SuppliedRawValueWidget', 'SuppliedStringListValueWidget', 'SuppliedStringValueWidget', 'WidgetType', 'shuffleboardEventImportanceName']
class BuiltInLayouts:
    """
    The types of layouts bundled with Shuffleboard.
    
    <pre>{@code
    ShuffleboardLayout myList = Shuffleboard::GetTab("My Tab")
    .GetLayout(BuiltinLayouts::kList, "My List");
    }</pre>
    
    Members:
    
      kList : Groups components in a vertical list. New widgets added to the layout will
    be placed at the bottom of the list. 
    Custom properties: <table>
    <tr><th>Name</th><th>Type</th><th>Default Value</th><th>Notes</th></tr>
    <tr><td>Label position</td><td>String</td><td>"BOTTOM"</td>
    <td>The position of component labels inside the grid. One of
    ``["TOP", "LEFT", "BOTTOM", "RIGHT", "HIDDEN"``</td></tr>
    </table>
    
      kGrid : Groups components in an *n* x *m* grid. Grid layouts default to
    3x3. 
    Custom properties: <table>
    <tr><th>Name</th><th>Type</th><th>Default Value</th><th>Notes</th></tr>
    <tr><td>Number of columns</td><td>Number</td><td>3</td><td>Must be in the
    range [1,15]</td>
    </tr>
    <tr><td>Number of rows</td><td>Number</td><td>3</td><td>Must be in the
    range [1,15]</td></tr> <tr> <td>Label position</td> <td>String</td>
    <td>"BOTTOM"</td>
    <td>The position of component labels inside the grid.
    One of ``["TOP", "LEFT", "BOTTOM", "RIGHT", "HIDDEN"``</td>
    </tr>
    </table>
    """
    __members__: typing.ClassVar[dict[str, BuiltInLayouts]]  # value = {'kList': <BuiltInLayouts.kList: 0>, 'kGrid': <BuiltInLayouts.kGrid: 1>}
    kGrid: typing.ClassVar[BuiltInLayouts]  # value = <BuiltInLayouts.kGrid: 1>
    kList: typing.ClassVar[BuiltInLayouts]  # value = <BuiltInLayouts.kList: 0>
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
class BuiltInWidgets:
    """
    The types of the widgets bundled with Shuffleboard.
    
    For example, setting a number to be displayed with a slider:
    <pre>{@code
    NetworkTableEntry example = Shuffleboard.getTab("My Tab")
    .add("My Number", 0)
    .withWidget(BuiltInWidgets.kNumberSlider)
    .getEntry();
    }</pre>
    
    Each value in this enum goes into detail on what data types that widget
    can support, as well as the custom properties that widget uses.
    
    Members:
    
      kTextView : Displays a value with a simple text field.
    Supported types:
    
    - String
    - Number
    - Boolean
    
    This widget has no custom properties.
    
      kNumberSlider : Displays a number with a controllable slider.
    Supported types:
    
    - Number
    
    Custom properties:
    <table>
    <tr><th>Name</th><th>Type</th><th>Default Value</th><th>Notes</th></tr>
    <tr><td>Min</td><td>Number</td><td>-1.0</td><td>The minimum value of the
    slider</td></tr> <tr><td>Max</td><td>Number</td><td>1.0</td><td>The maximum
    value of the slider</td></tr> <tr><td>Block
    increment</td><td>Number</td><td>0.0625</td> <td>How much to move the
    slider by with the arrow keys</td></tr>
    </table>
    
      kNumberBar : Displays a number with a view-only bar.
    Supported types:
    
    - Number
    
    Custom properties:
    <table>
    <tr><th>Name</th><th>Type</th><th>Default Value</th><th>Notes</th></tr>
    <tr><td>Min</td><td>Number</td><td>-1.0</td><td>The minimum value of the
    bar</td></tr> <tr><td>Max</td><td>Number</td><td>1.0</td><td>The maximum
    value of the bar</td></tr>
    <tr><td>Center</td><td>Number</td><td>0</td><td>The center ("zero") value
    of the bar</td></tr>
    </table>
    
      kDial : Displays a number with a view-only dial. Displayed values are rounded to
    the nearest integer. 
    Supported types:  - Number
    
    Custom properties:
    <table>
    <tr><th>Name</th><th>Type</th><th>Default Value</th><th>Notes</th></tr>
    <tr><td>Min</td><td>Number</td><td>0</td><td>The minimum value of the
    dial</td></tr> <tr><td>Max</td><td>Number</td><td>100</td><td>The maximum
    value of the dial</td></tr> <tr><td>Show
    value</td><td>Boolean</td><td>true</td> <td>Whether or not to show the
    value as text</td></tr>
    </table>
    
      kGraph : Displays a number with a graph. **NOTE:** graphs can be taxing
    on the computer running the dashboard. Keep the number of visible data
    points to a minimum. Making the widget smaller also helps with performance,
    but may cause the graph to become difficult to read. 
    Supported types:
    
    - Number
    - Number array
    
    Custom properties:
    <table>
    <tr><th>Name</th><th>Type</th><th>Default Value</th><th>Notes</th></tr>
    <tr><td>Visible time</td><td>Number</td><td>30</td>
    <td>How long, in seconds, should past data be visible for</td></tr>
    </table>
    
      kBooleanBox : Displays a boolean value as a large colored box.
    Supported types:
    
    - Boolean
    
    Custom properties:
    <table>
    <tr><th>Name</th><th>Type</th><th>Default Value</th><th>Notes</th></tr>
    <tr><td>Color when true</td><td>Color</td><td>"green"</td>
    <td>Can be specified as a string (``"#00FF00"``) or a rgba integer
    (``0x00FF0000``)
    </td></tr>
    <tr><td>Color when false</td><td>Color</td><td>"red"</td>
    <td>Can be specified as a string or a number</td></tr>
    </table>
    
      kToggleButton : Displays a boolean with a large interactive toggle button.
    Supported types:
    
    - Boolean
    
    This widget has no custom properties.
    
      kToggleSwitch : Displays a boolean with a fixed-size toggle switch.
    Supported types:
    
    - Boolean
    
    This widget has no custom properties.
    
      kVoltageView : Displays an analog input or a raw number with a number bar.
    Supported types:
    
    - Number
    - AnalogInput
    
    Custom properties:
    <table>
    <tr><th>Name</th><th>Type</th><th>Default Value</th><th>Notes</th></tr>
    <tr><td>Min</td><td>Number</td><td>0</td><td>The minimum value of the
    bar</td></tr> <tr><td>Max</td><td>Number</td><td>5</td><td>The maximum
    value of the bar</td></tr>
    <tr><td>Center</td><td>Number</td><td>0</td><td>The center ("zero") value
    of the bar</td></tr>
    <tr><td>Orientation</td><td>String</td><td>"HORIZONTAL"</td>
    <td>The orientation of the bar. One of {@code ["HORIZONTAL",
    "VERTICAL"]}</td></tr> <tr><td>Number of tick
    marks</td><td>Number</td><td>5</td> <td>The number of discrete ticks on the
    bar</td></tr>
    </table>
    
      kPowerDistribution : Displays a PowerDistribution. 
    Supported types:  -
    PowerDistribution
    
    Custom properties:
    <table>
    <tr><th>Name</th><th>Type</th><th>Default Value</th><th>Notes</th></tr>
    <tr><td>Show voltage and current values</td><td>Boolean</td><td>true</td>
    <td>Whether or not to display the voltage and current draw</td></tr>
    </table>
    
      kComboBoxChooser : Displays a SendableChooser with a dropdown combo box with a list of
    options.
    Supported types:
    
    - SendableChooser
    
    This widget has no custom properties.
    
      kSplitButtonChooser : Displays a SendableChooserwith a toggle button for each available option.
    Supported types:
    
    - SendableChooser
    
    This widget has no custom properties.
    
      kEncoder : Displays an Encoder displaying its speed,
    total traveled distance, and its distance per tick. 
    Supported types:
    
    - Encoder
    
    This widget has no custom properties.
    
      kMotorController : Displays a MotorController.
    The motor controller will be controllable from the dashboard when test mode
    is enabled, but will otherwise be view-only. 
    Supported types:
    - PWMMotorController
    - DMC60
    - Jaguar
    - PWMTalonSRX
    - PWMVictorSPX
    - SD540
    - Spark
    - Talon
    - Victor
    - VictorSP
    - MotorControllerGroup
    - Any custom subclass of ``SpeedContorller``
    
    Custom properties:
    <table>
    <tr><th>Name</th><th>Type</th><th>Default Value</th><th>Notes</th></tr>
    <tr><td>Orientation</td><td>String</td><td>"HORIZONTAL"</td>
    <td>One of ``["HORIZONTAL", "VERTICAL"]``</td></tr>
    </table>
    
      kCommand : Displays a command with a toggle button. Pressing the button will start the
    command, and the button will automatically release when the command
    completes. 
    Supported types:  - Command - CommandGroup
    - Any custom subclass of ``Command`` or ``CommandGroup``
    
    This widget has no custom properties.
    
      kPIDCommand : Displays a PID command with a checkbox and an editor for the PIDF
    constants. Selecting the checkbox will start the command, and the checkbox
    will automatically deselect when the command completes. 
    Supported
    types:  - PIDCommand
    - Any custom subclass of ``PIDCommand``
    
    This widget has no custom properties.
    
      kPIDController : Displays a PID controller with an editor for the PIDF constants and a
    toggle switch for enabling and disabling the controller. 
    Supported
    types:  - PIDController
    
    This widget has no custom properties.
    
      kAccelerometer : Displays an accelerometer with a number bar displaying the magnitude of the
    acceleration and text displaying the exact value. 
    Supported types:
    - AnalogAccelerometer
    
    Custom properties:
    <table>
    <tr><th>Name</th><th>Type</th><th>Default Value</th><th>Notes</th></tr>
    <tr><td>Min</td><td>Number</td><td>-1</td>
    <td>The minimum acceleration value to display</td></tr>
    <tr><td>Max</td><td>Number</td><td>1</td>
    <td>The maximum acceleration value to display</td></tr>
    <tr><td>Show text</td><td>Boolean</td><td>true</td>
    <td>Show or hide the acceleration values</td></tr>
    <tr><td>Precision</td><td>Number</td><td>2</td>
    <td>How many numbers to display after the decimal point</td></tr>
    <tr><td>Show tick marks</td><td>Boolean</td><td>false</td>
    <td>Show or hide the tick marks on the number bars</td></tr>
    </table>
    
      k3AxisAccelerometer : Displays a 3-axis accelerometer with a number bar for each axis'
    acceleration. 
    Supported types:  - ADXL345_I2C -
    ADXL345_SPI - ADXL362
    
    Custom properties:
    <table>
    <tr><th>Name</th><th>Type</th><th>Default Value</th><th>Notes</th></tr>
    <tr><td>Range</td><td>Range</td><td>k16G</td><td>The accelerometer
    range</td></tr> <tr><td>Show value</td><td>Boolean</td><td>true</td>
    <td>Show or hide the acceleration values</td></tr>
    <tr><td>Precision</td><td>Number</td><td>2</td>
    <td>How many numbers to display after the decimal point</td></tr>
    <tr><td>Show tick marks</td><td>Boolean</td><td>false</td>
    <td>Show or hide the tick marks on the number bars</td></tr>
    </table>
    
      kGyro : Displays a gyro with a dial from 0 to 360 degrees.
    Supported types:
    
    - ADXRS450_Gyro
    - AnalogGyro
    - Any custom subclass of ``GyroBase`` (such as a MXP gyro)
    
    Custom properties:
    <table>
    <tr><th>Name</th><th>Type</th><th>Default Value</th><th>Notes</th></tr>
    <tr><td>Major tick
    spacing</td><td>Number</td><td>45</td><td>Degrees</td></tr>
    <tr><td>Starting angle</td><td>Number</td><td>180</td>
    <td>How far to rotate the entire dial, in degrees</td></tr>
    <tr><td>Show tick mark ring</td><td>Boolean</td><td>true</td></tr>
    </table>
    
      kRelay : Displays a relay with toggle buttons for each supported mode (off, on,
    forward, reverse). 
    Supported types:  - Relay
    
    This widget has no custom properties.
    
      kDifferentialDrive : Displays a differential drive with a widget that displays the speed of each
    side of the drivebase and a vector for the direction and rotation of the
    drivebase. The widget will be controllable if the robot is in test mode.
    Supported types:
    
    - DifferentialDrive
    
    Custom properties:
    <table>
    <tr><th>Name</th><th>Type</th><th>Default Value</th><th>Notes</th></tr>
    <tr><td>Number of wheels</td><td>Number</td><td>4</td><td>Must be a
    positive even integer
    </td></tr>
    <tr><td>Wheel diameter</td><td>Number</td><td>80</td><td>Pixels</td></tr>
    <tr><td>Show velocity vectors</td><td>Boolean</td><td>true</td></tr>
    </table>
    
      kMecanumDrive : Displays a mecanum drive with a widget that displays the speed of each
    wheel, and vectors for the direction and rotation of the drivebase. The
    widget will be controllable if the robot is in test mode. 
    Supported
    types:  - MecanumDrive
    
    Custom properties:
    <table>
    <tr><th>Name</th><th>Type</th><th>Default Value</th><th>Notes</th></tr>
    <tr><td>Show velocity vectors</td><td>Boolean</td><td>true</td></tr>
    </table>
    
      kCameraStream : Displays a camera stream.
    Supported types:
    
    - VideoSource (as long as it is streaming on an MJPEG server)
    
    Custom properties:
    <table>
    <tr><th>Name</th><th>Type</th><th>Default Value</th><th>Notes</th></tr>
    <tr><td>Show crosshair</td><td>Boolean</td><td>true</td>
    <td>Show or hide a crosshair on the image</td></tr>
    <tr><td>Crosshair color</td><td>Color</td><td>"white"</td>
    <td>Can be a string or a rgba integer</td></tr>
    <tr><td>Show controls</td><td>Boolean</td><td>true</td><td>Show or hide the
    stream controls
    </td></tr>
    <tr><td>Rotation</td><td>String</td><td>"NONE"</td>
    <td>Rotates the displayed image. One of {@code ["NONE", "QUARTER_CW",
    "QUARTER_CCW", "HALF"]}
    </td></tr>
    </table>
    
      kField : Displays a field2d object.
    Supported types:
    
    - Field2d
    """
    __members__: typing.ClassVar[dict[str, BuiltInWidgets]]  # value = {'kTextView': <BuiltInWidgets.kTextView: 0>, 'kNumberSlider': <BuiltInWidgets.kNumberSlider: 1>, 'kNumberBar': <BuiltInWidgets.kNumberBar: 2>, 'kDial': <BuiltInWidgets.kDial: 3>, 'kGraph': <BuiltInWidgets.kGraph: 4>, 'kBooleanBox': <BuiltInWidgets.kBooleanBox: 5>, 'kToggleButton': <BuiltInWidgets.kToggleButton: 6>, 'kToggleSwitch': <BuiltInWidgets.kToggleSwitch: 7>, 'kVoltageView': <BuiltInWidgets.kVoltageView: 8>, 'kPowerDistribution': <BuiltInWidgets.kPowerDistribution: 9>, 'kComboBoxChooser': <BuiltInWidgets.kComboBoxChooser: 10>, 'kSplitButtonChooser': <BuiltInWidgets.kSplitButtonChooser: 11>, 'kEncoder': <BuiltInWidgets.kEncoder: 12>, 'kMotorController': <BuiltInWidgets.kMotorController: 13>, 'kCommand': <BuiltInWidgets.kCommand: 14>, 'kPIDCommand': <BuiltInWidgets.kPIDCommand: 15>, 'kPIDController': <BuiltInWidgets.kPIDController: 16>, 'kAccelerometer': <BuiltInWidgets.kAccelerometer: 17>, 'k3AxisAccelerometer': <BuiltInWidgets.k3AxisAccelerometer: 18>, 'kGyro': <BuiltInWidgets.kGyro: 19>, 'kRelay': <BuiltInWidgets.kRelay: 20>, 'kDifferentialDrive': <BuiltInWidgets.kDifferentialDrive: 21>, 'kMecanumDrive': <BuiltInWidgets.kMecanumDrive: 22>, 'kCameraStream': <BuiltInWidgets.kCameraStream: 23>, 'kField': <BuiltInWidgets.kField: 24>}
    k3AxisAccelerometer: typing.ClassVar[BuiltInWidgets]  # value = <BuiltInWidgets.k3AxisAccelerometer: 18>
    kAccelerometer: typing.ClassVar[BuiltInWidgets]  # value = <BuiltInWidgets.kAccelerometer: 17>
    kBooleanBox: typing.ClassVar[BuiltInWidgets]  # value = <BuiltInWidgets.kBooleanBox: 5>
    kCameraStream: typing.ClassVar[BuiltInWidgets]  # value = <BuiltInWidgets.kCameraStream: 23>
    kComboBoxChooser: typing.ClassVar[BuiltInWidgets]  # value = <BuiltInWidgets.kComboBoxChooser: 10>
    kCommand: typing.ClassVar[BuiltInWidgets]  # value = <BuiltInWidgets.kCommand: 14>
    kDial: typing.ClassVar[BuiltInWidgets]  # value = <BuiltInWidgets.kDial: 3>
    kDifferentialDrive: typing.ClassVar[BuiltInWidgets]  # value = <BuiltInWidgets.kDifferentialDrive: 21>
    kEncoder: typing.ClassVar[BuiltInWidgets]  # value = <BuiltInWidgets.kEncoder: 12>
    kField: typing.ClassVar[BuiltInWidgets]  # value = <BuiltInWidgets.kField: 24>
    kGraph: typing.ClassVar[BuiltInWidgets]  # value = <BuiltInWidgets.kGraph: 4>
    kGyro: typing.ClassVar[BuiltInWidgets]  # value = <BuiltInWidgets.kGyro: 19>
    kMecanumDrive: typing.ClassVar[BuiltInWidgets]  # value = <BuiltInWidgets.kMecanumDrive: 22>
    kMotorController: typing.ClassVar[BuiltInWidgets]  # value = <BuiltInWidgets.kMotorController: 13>
    kNumberBar: typing.ClassVar[BuiltInWidgets]  # value = <BuiltInWidgets.kNumberBar: 2>
    kNumberSlider: typing.ClassVar[BuiltInWidgets]  # value = <BuiltInWidgets.kNumberSlider: 1>
    kPIDCommand: typing.ClassVar[BuiltInWidgets]  # value = <BuiltInWidgets.kPIDCommand: 15>
    kPIDController: typing.ClassVar[BuiltInWidgets]  # value = <BuiltInWidgets.kPIDController: 16>
    kPowerDistribution: typing.ClassVar[BuiltInWidgets]  # value = <BuiltInWidgets.kPowerDistribution: 9>
    kRelay: typing.ClassVar[BuiltInWidgets]  # value = <BuiltInWidgets.kRelay: 20>
    kSplitButtonChooser: typing.ClassVar[BuiltInWidgets]  # value = <BuiltInWidgets.kSplitButtonChooser: 11>
    kTextView: typing.ClassVar[BuiltInWidgets]  # value = <BuiltInWidgets.kTextView: 0>
    kToggleButton: typing.ClassVar[BuiltInWidgets]  # value = <BuiltInWidgets.kToggleButton: 6>
    kToggleSwitch: typing.ClassVar[BuiltInWidgets]  # value = <BuiltInWidgets.kToggleSwitch: 7>
    kVoltageView: typing.ClassVar[BuiltInWidgets]  # value = <BuiltInWidgets.kVoltageView: 8>
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
class ComplexWidget(_ComplexWidget):
    """
    A Shuffleboard widget that handles a Sendable object such as a motor
    controller or sensor.
    """
    def __init__(self, parent: ShuffleboardContainer, title: str, sendable: wpiutil._wpiutil.Sendable) -> None:
        ...
    def buildInto(self, parentTable: ntcore._ntcore.NetworkTable, metaTable: ntcore._ntcore.NetworkTable) -> None:
        ...
    def disableIfActuator(self) -> None:
        ...
    def enableIfActuator(self) -> None:
        ...
class LayoutType:
    """
    Represents the type of a layout in Shuffleboard. Using this is preferred over
    specifying raw strings, to avoid typos and having to know or look up the
    exact string name for a desired layout.
    
    @see BuiltInLayouts the built-in layout types
    """
    def __init__(self, layoutName: str) -> None:
        ...
    def getLayoutName(self) -> str:
        """
        Gets the string type of the layout as defined by that layout in
        Shuffleboard.
        """
class Shuffleboard:
    """
    The Shuffleboard class provides a mechanism with which data can be added and
    laid out in the Shuffleboard dashboard application from a robot program. Tabs
    and layouts can be specified, as well as choosing which widgets to display
    with and setting properties of these widgets; for example, programmers can
    specify a specific ``boolean`` value to be displayed with a toggle button
    instead of the default colored box, or set custom colors for that box.
    
    For example, displaying a boolean entry with a toggle button:
    <pre>{@code
    NetworkTableEntry myBoolean = Shuffleboard.getTab("Example Tab")
    .add("My Boolean", false)
    .withWidget("Toggle Button")
    .getEntry();
    }</pre>
    
    Changing the colors of the boolean box:
    <pre>{@code
    NetworkTableEntry myBoolean = Shuffleboard.getTab("Example Tab")
    .add("My Boolean", false)
    .withWidget("Boolean Box")
    .withProperties(Map.of("colorWhenTrue", "green", "colorWhenFalse",
    "maroon")) .getEntry();
    }</pre>
    
    Specifying a parent layout. Note that the layout type must *always* be
    specified, even if the layout has already been generated by a previously
    defined entry.
    <pre>{@code
    NetworkTableEntry myBoolean = Shuffleboard.getTab("Example Tab")
    .getLayout("List", "Example List")
    .add("My Boolean", false)
    .withWidget("Toggle Button")
    .getEntry();
    }</pre>
    </p>
    
    Teams are encouraged to set up shuffleboard layouts at the start of the robot
    program.
    """
    kBaseTableName: typing.ClassVar[str] = '/Shuffleboard'
    @staticmethod
    @typing.overload
    def addEventMarker(name: str, description: str, importance: ShuffleboardEventImportance) -> None:
        """
        Notifies Shuffleboard of an event. Events can range from as trivial as a
        change in a command state to as critical as a total power loss or component
        failure. If Shuffleboard is recording, the event will also be recorded.
        
        If ``name`` is ``null`` or empty, no event will be sent and an
        error will be printed to the driver station.
        
        :param name:        the name of the event
        :param description: a description of the event
        :param importance:  the importance of the event
        """
    @staticmethod
    @typing.overload
    def addEventMarker(name: str, importance: ShuffleboardEventImportance) -> None:
        """
        Notifies Shuffleboard of an event. Events can range from as trivial as a
        change in a command state to as critical as a total power loss or component
        failure. If Shuffleboard is recording, the event will also be recorded.
        
        If ``name`` is ``null`` or empty, no event will be sent and an
        error will be printed to the driver station.
        
        :param name:       the name of the event
        :param importance: the importance of the event
        """
    @staticmethod
    def clearRecordingFileNameFormat() -> None:
        """
        Clears the custom name format for recording files. New recordings will use
        the default format.
        
        @see SetRecordingFileNameFormat(std::string_view)
        """
    @staticmethod
    def disableActuatorWidgets() -> None:
        """
        Disables user control of widgets containing actuators. For safety reasons,
        actuators should only be controlled while in test mode. IterativeRobotBase
        and SampleRobot are both configured to call this method when exiting in
        test mode; most users should not need to use this method directly.
        """
    @staticmethod
    def enableActuatorWidgets() -> None:
        """
        Enables user control of widgets containing actuators: motor controllers,
        relays, etc. This should only be used when the robot is in test mode.
        IterativeRobotBase and SampleRobot are both configured to call this method
        when entering test mode; most users should not need to use this method
        directly.
        """
    @staticmethod
    def getTab(title: str) -> ShuffleboardTab:
        """
        Gets the Shuffleboard tab with the given title, creating it if it does not
        already exist.
        
        :param title: the title of the tab
        
        :returns: the tab with the given title
        """
    @staticmethod
    @typing.overload
    def selectTab(index: int) -> None:
        """
        Selects the tab in the dashboard with the given index in the range
        [0..n-1], where *n* is the number of tabs in the dashboard at the time
        this method is called.
        
        :param index: the index of the tab to select
        """
    @staticmethod
    @typing.overload
    def selectTab(title: str) -> None:
        """
        Selects the tab in the dashboard with the given title.
        
        :param title: the title of the tab to select
        """
    @staticmethod
    def setRecordingFileNameFormat(format: str) -> None:
        """
        Sets the file name format for new recording files to use. If recording is
        in progress when this method is called, it will continue to use the same
        file. New recordings will use the format.
        
        To avoid recording files overwriting each other, make sure to use unique
        recording file names. File name formats accept templates for inserting the
        date and time when the recording started with the ``${date``} and
        ``${time``} templates, respectively. For example, the default format is
        ``"recording-${time``"} and recording files created with it will have
        names like ``"recording-2018.01.15.sbr"``. Users are
        **strongly** recommended to use the ``${time``} template
        to ensure unique file names.
        </p>
        
        :param format: the format for the
        """
    @staticmethod
    def startRecording() -> None:
        """
        Starts data recording on the dashboard. Has no effect if recording is
        already in progress.
        """
    @staticmethod
    def stopRecording() -> None:
        """
        Stops data recording on the dashboard. Has no effect if no recording is in
        progress.
        """
    @staticmethod
    def update() -> None:
        """
        Updates all the values in Shuffleboard. Iterative and timed robots are
        pre-configured to call this method in the main robot loop; teams using
        custom robot base classes, or subclass SampleRobot, should make sure to
        call this repeatedly to keep data on the dashboard up to date.
        """
class ShuffleboardComponentBase(ShuffleboardValue):
    """
    A shim class to allow storing ShuffleboardComponents in arrays.
    """
    def buildMetadata(self, metaTable: ntcore._ntcore.NetworkTable) -> None:
        ...
    def getParent(self) -> ShuffleboardContainer:
        ...
    def getType(self) -> str:
        ...
    def setType(self, type: str) -> None:
        ...
class ShuffleboardContainer(ShuffleboardValue):
    """
    Common interface for objects that can contain shuffleboard components.
    """
    @typing.overload
    def add(self, title: str, defaultValue: wpiutil._wpiutil.Sendable) -> ComplexWidget:
        """
        Adds a widget to this container to display the given sendable.
        
        :param title:        the title of the widget
        :param defaultValue: the sendable to display
        
        :returns: a widget to display the sendable data
                  @throws IllegalArgumentException if a widget already exists in this
                  container with the given title
        """
    @typing.overload
    def add(self, defaultValue: wpiutil._wpiutil.Sendable) -> ComplexWidget:
        """
        Adds a widget to this container to display the given sendable.
        
        :param defaultValue: the sendable to display
        
        :returns: a widget to display the sendable data
                  @throws IllegalArgumentException if a widget already exists in this
                  container with the given title, or if the sendable's name has not been
                  specified
        """
    @typing.overload
    def add(self, title: str, defaultValue: ntcore._ntcore.Value) -> SimpleWidget:
        """
        Adds a widget to this container to display the given data.
        
        :param title:        the title of the widget
        :param defaultValue: the default value of the widget
        
        :returns: a widget to display the sendable data
                  @throws IllegalArgumentException if a widget already exists in this
                  container with the given title
                  @see AddPersistent(std::string_view, std::shared_ptr<nt::Value>)
                  Add(std::string_view title, std::shared_ptr<nt::Value> defaultValue)
        """
    @typing.overload
    def add(self, title: str, defaultValue: bool) -> SimpleWidget:
        """
        Adds a widget to this container to display the given data.
        
        :param title:        the title of the widget
        :param defaultValue: the default value of the widget
        
        :returns: a widget to display the sendable data
                  @throws IllegalArgumentException if a widget already exists in this
                  container with the given title
                  @see AddPersistent(std::string_view, bool)
                  Add(std::string_view title, bool defaultValue)
        """
    @typing.overload
    def add(self, title: str, defaultValue: float) -> SimpleWidget:
        """
        Adds a widget to this container to display the given data.
        
        :param title:        the title of the widget
        :param defaultValue: the default value of the widget
        
        :returns: a widget to display the sendable data
                  @throws IllegalArgumentException if a widget already exists in this
                  container with the given title
                  @see AddPersistent(std::string_view, double)
                  Add(std::string_view title, double defaultValue)
        """
    @typing.overload
    def add(self, title: str, defaultValue: float) -> SimpleWidget:
        """
        Adds a widget to this container to display the given data.
        
        :param title:        the title of the widget
        :param defaultValue: the default value of the widget
        
        :returns: a widget to display the sendable data
                  @throws IllegalArgumentException if a widget already exists in this
                  container with the given title
                  @see AddPersistent(std::string_view, double)
                  Add(std::string_view title, double defaultValue)
        """
    @typing.overload
    def add(self, title: str, defaultValue: int) -> SimpleWidget:
        """
        Adds a widget to this container to display the given data.
        
        :param title:        the title of the widget
        :param defaultValue: the default value of the widget
        
        :returns: a widget to display the sendable data
                  @throws IllegalArgumentException if a widget already exists in this
                  container with the given title
                  @see AddPersistent(std::string_view, int)
                  Add(std::string_view title, int defaultValue)
        """
    @typing.overload
    def add(self, title: str, defaultValue: str) -> SimpleWidget:
        """
        Adds a widget to this container to display the given data.
        
        :param title:        the title of the widget
        :param defaultValue: the default value of the widget
        
        :returns: a widget to display the sendable data
                  @throws IllegalArgumentException if a widget already exists in this
                  container with the given title
                  @see AddPersistent(std::string_view, std::string_view)
                  Add(std::string_view title, std::string_view defaultValue)
        """
    @typing.overload
    def add(self, title: str, defaultValue: list[bool]) -> SimpleWidget:
        """
        Adds a widget to this container to display the given data.
        
        :param title:        the title of the widget
        :param defaultValue: the default value of the widget
        
        :returns: a widget to display the sendable data
                  @throws IllegalArgumentException if a widget already exists in this
                  container with the given title
                  @see AddPersistent(std::string_view, std::span<const bool>)
                  Add(std::string_view title, std::span<const bool> defaultValue)
        """
    @typing.overload
    def add(self, title: str, defaultValue: list[float]) -> SimpleWidget:
        """
        Adds a widget to this container to display the given data.
        
        :param title:        the title of the widget
        :param defaultValue: the default value of the widget
        
        :returns: a widget to display the sendable data
                  @throws IllegalArgumentException if a widget already exists in this
                  container with the given title
                  @see AddPersistent(std::string_view, std::span<const double>)
                  Add(std::string_view title, std::span<const double> defaultValue)
        """
    @typing.overload
    def add(self, title: str, defaultValue: list[float]) -> SimpleWidget:
        """
        Adds a widget to this container to display the given data.
        
        :param title:        the title of the widget
        :param defaultValue: the default value of the widget
        
        :returns: a widget to display the sendable data
                  @throws IllegalArgumentException if a widget already exists in this
                  container with the given title
                  @see AddPersistent(std::string_view, std::span<const double>)
                  Add(std::string_view title, std::span<const double> defaultValue)
        """
    @typing.overload
    def add(self, title: str, defaultValue: list[int]) -> SimpleWidget:
        """
        Adds a widget to this container to display the given data.
        
        :param title:        the title of the widget
        :param defaultValue: the default value of the widget
        
        :returns: a widget to display the sendable data
                  @throws IllegalArgumentException if a widget already exists in this
                  container with the given title
                  @see AddPersistent(std::string_view, std::span<const double>)
                  Add(std::string_view title, std::span<const double> defaultValue)
        """
    @typing.overload
    def add(self, title: str, defaultValue: list[str]) -> SimpleWidget:
        """
        Adds a widget to this container to display the given data.
        
        :param title:        the title of the widget
        :param defaultValue: the default value of the widget
        
        :returns: a widget to display the sendable data
                  @throws IllegalArgumentException if a widget already exists in this
                  container with the given title
                  @see AddPersistent(std::string_view, std::span<const std::string>)
                  Add(std::string_view title, std::span<const std::string> defaultValue)
        """
    def addBoolean(self, title: str, supplier: typing.Callable[[], bool]) -> SuppliedBoolValueWidget:
        """
        Adds a widget to this container. The widget will display the data provided
        by the value supplier. Changes made on the dashboard will not propagate to
        the widget object, and will be overridden by values from the value
        supplier.
        
        :param title:    the title of the widget
        :param supplier: the supplier for values
        
        :returns: a widget to display data
        """
    def addBooleanArray(self, title: str, supplier: typing.Callable[[], list[int]]) -> SuppliedBoolListValueWidget:
        """
        Adds a widget to this container. The widget will display the data provided
        by the value supplier. Changes made on the dashboard will not propagate to
        the widget object, and will be overridden by values from the value
        supplier.
        
        :param title:    the title of the widget
        :param supplier: the supplier for values
        
        :returns: a widget to display data
        """
    def addDouble(self, title: str, supplier: typing.Callable[[], float]) -> SuppliedDoubleValueWidget:
        """
        Adds a widget to this container. The widget will display the data provided
        by the value supplier. Changes made on the dashboard will not propagate to
        the widget object, and will be overridden by values from the value
        supplier.
        
        :param title:    the title of the widget
        :param supplier: the supplier for values
        
        :returns: a widget to display data
        """
    def addDoubleArray(self, title: str, supplier: typing.Callable[[], list[float]]) -> SuppliedDoubleListValueWidget:
        """
        Adds a widget to this container. The widget will display the data provided
        by the value supplier. Changes made on the dashboard will not propagate to
        the widget object, and will be overridden by values from the value
        supplier.
        
        :param title:    the title of the widget
        :param supplier: the supplier for values
        
        :returns: a widget to display data
        """
    def addFloat(self, title: str, supplier: typing.Callable[[], float]) -> SuppliedFloatValueWidget:
        """
        Adds a widget to this container. The widget will display the data provided
        by the value supplier. Changes made on the dashboard will not propagate to
        the widget object, and will be overridden by values from the value
        supplier.
        
        :param title:    the title of the widget
        :param supplier: the supplier for values
        
        :returns: a widget to display data
        """
    def addFloatArray(self, title: str, supplier: typing.Callable[[], list[float]]) -> SuppliedFloatListValueWidget:
        """
        Adds a widget to this container. The widget will display the data provided
        by the value supplier. Changes made on the dashboard will not propagate to
        the widget object, and will be overridden by values from the value
        supplier.
        
        :param title:    the title of the widget
        :param supplier: the supplier for values
        
        :returns: a widget to display data
        """
    def addInteger(self, title: str, supplier: typing.Callable[[], int]) -> SuppliedIntegerValueWidget:
        """
        Adds a widget to this container. The widget will display the data provided
        by the value supplier. Changes made on the dashboard will not propagate to
        the widget object, and will be overridden by values from the value
        supplier.
        
        :param title:    the title of the widget
        :param supplier: the supplier for values
        
        :returns: a widget to display data
        """
    def addIntegerArray(self, title: str, supplier: typing.Callable[[], list[int]]) -> SuppliedIntListValueWidget:
        """
        Adds a widget to this container. The widget will display the data provided
        by the value supplier. Changes made on the dashboard will not propagate to
        the widget object, and will be overridden by values from the value
        supplier.
        
        :param title:    the title of the widget
        :param supplier: the supplier for values
        
        :returns: a widget to display data
        """
    def addNumber(self, title: str, supplier: typing.Callable[[], float]) -> SuppliedDoubleValueWidget:
        """
        Adds a widget to this container. The widget will display the data provided
        by the value supplier. Changes made on the dashboard will not propagate to
        the widget object, and will be overridden by values from the value
        supplier.
        
        :param title:    the title of the widget
        :param supplier: the supplier for values
        
        :returns: a widget to display data
        """
    def addNumberArray(self, title: str, supplier: typing.Callable[[], list[float]]) -> SuppliedDoubleListValueWidget:
        """
        Adds a widget to this container. The widget will display the data provided
        by the value supplier. Changes made on the dashboard will not propagate to
        the widget object, and will be overridden by values from the value
        supplier.
        
        :param title:    the title of the widget
        :param supplier: the supplier for values
        
        :returns: a widget to display data
        """
    @typing.overload
    def addPersistent(self, title: str, defaultValue: ntcore._ntcore.Value) -> SimpleWidget:
        """
        Adds a widget to this container to display a simple piece of data.
        
        Unlike Add(std::string_view, std::shared_ptr<nt::Value>), the value in the
        widget will be saved on the robot and will be used when the robot program
        next starts rather than ``defaultValue``.
        
        :param title:        the title of the widget
        :param defaultValue: the default value of the widget
        
        :returns: a widget to display the sendable data
                  @see Add(stdd::string_view, std::shared_ptr<nt::Value>)
                  Add(std::string_view title, std::shared_ptr<nt::Value> defaultValue)
        """
    @typing.overload
    def addPersistent(self, title: str, defaultValue: bool) -> SimpleWidget:
        """
        Adds a widget to this container to display a simple piece of data.
        
        Unlike Add(std::string_view, bool), the value in the widget will be saved
        on the robot and will be used when the robot program next starts rather
        than ``defaultValue``.
        
        :param title:        the title of the widget
        :param defaultValue: the default value of the widget
        
        :returns: a widget to display the sendable data
                  @see Add(std::string_view, bool)
                  Add(std::string_view title, bool defaultValue)
        """
    @typing.overload
    def addPersistent(self, title: str, defaultValue: float) -> SimpleWidget:
        """
        Adds a widget to this container to display a simple piece of data.
        
        Unlike Add(std::string_view, double), the value in the widget will be saved
        on the robot and will be used when the robot program next starts rather
        than ``defaultValue``.
        
        :param title:        the title of the widget
        :param defaultValue: the default value of the widget
        
        :returns: a widget to display the sendable data
                  @see Add(std::string_view, double)
                  Add(std::string_view title, double defaultValue)
        """
    @typing.overload
    def addPersistent(self, title: str, defaultValue: float) -> SimpleWidget:
        """
        Adds a widget to this container to display a simple piece of data.
        
        Unlike Add(std::string_view, float), the value in the widget will be saved
        on the robot and will be used when the robot program next starts rather
        than ``defaultValue``.
        
        :param title:        the title of the widget
        :param defaultValue: the default value of the widget
        
        :returns: a widget to display the sendable data
                  @see Add(std::string_view, float)
                  Add(std::string_view title, float defaultValue)
        """
    @typing.overload
    def addPersistent(self, title: str, defaultValue: int) -> SimpleWidget:
        """
        Adds a widget to this container to display a simple piece of data.
        
        Unlike Add(std::string_view, int64_t), the value in the widget will be
        saved on the robot and will be used when the robot program next starts
        rather than ``defaultValue``.
        
        :param title:        the title of the widget
        :param defaultValue: the default value of the widget
        
        :returns: a widget to display the sendable data
                  @see Add(std:string_view, int64_t)
                  Add(std::string_view title, int64_t defaultValue)
        """
    @typing.overload
    def addPersistent(self, title: str, defaultValue: str) -> SimpleWidget:
        """
        Adds a widget to this container to display a simple piece of data.
        
        Unlike Add(std::string_view, std::string_view), the value in the widget
        will be saved on the robot and will be used when the robot program next
        starts rather than ``defaultValue``.
        
        :param title:        the title of the widget
        :param defaultValue: the default value of the widget
        
        :returns: a widget to display the sendable data
                  @see Add(std::string_view, std::string_view)
                  Add(std::string_view title, std::string_view defaultValue)
        """
    @typing.overload
    def addPersistent(self, title: str, defaultValue: list[bool]) -> SimpleWidget:
        """
        Adds a widget to this container to display a simple piece of data.
        
        Unlike Add(std::string_view, std::span<const bool>), the value in the
        widget will be saved on the robot and will be used when the robot program
        next starts rather than ``defaultValue``.
        
        :param title:        the title of the widget
        :param defaultValue: the default value of the widget
        
        :returns: a widget to display the sendable data
                  @see Add(std::string_view, std::span<const bool>)
                  Add(std::string_view title, std::span<const bool> defaultValue)
        """
    @typing.overload
    def addPersistent(self, title: str, defaultValue: list[float]) -> SimpleWidget:
        """
        Adds a widget to this container to display a simple piece of data.
        
        Unlike Add(std::string_view, std::span<const double>), the value in the
        widget will be saved on the robot and will be used when the robot program
        next starts rather than ``defaultValue``.
        
        :param title:        the title of the widget
        :param defaultValue: the default value of the widget
        
        :returns: a widget to display the sendable data
                  @see Add(std::string_view, std::span<const double>)
                  Add(std::string_view title, std::span<const double> defaultValue)
        """
    @typing.overload
    def addPersistent(self, title: str, defaultValue: list[float]) -> SimpleWidget:
        """
        Adds a widget to this container to display a simple piece of data.
        
        Unlike Add(std::string_view, std::span<const float>), the value in the
        widget will be saved on the robot and will be used when the robot program
        next starts rather than ``defaultValue``.
        
        :param title:        the title of the widget
        :param defaultValue: the default value of the widget
        
        :returns: a widget to display the sendable data
                  @see Add(std::string_view, std::span<const float>)
                  Add(std::string_view title, std::span<const float> defaultValue)
        """
    @typing.overload
    def addPersistent(self, title: str, defaultValue: list[int]) -> SimpleWidget:
        """
        Adds a widget to this container to display a simple piece of data.
        
        Unlike Add(std::string_view, std::span<const int64_t>), the value in the
        widget will be saved on the robot and will be used when the robot program
        next starts rather than ``defaultValue``.
        
        :param title:        the title of the widget
        :param defaultValue: the default value of the widget
        
        :returns: a widget to display the sendable data
                  @see Add(std::string_view, std::span<const int64_t>)
                  Add(std::string_view title, std::span<const int64_t> defaultValue)
        """
    @typing.overload
    def addPersistent(self, title: str, defaultValue: list[str]) -> SimpleWidget:
        """
        Adds a widget to this container to display a simple piece of data.
        
        Unlike Add(std::string_view, std::span<const std::string>), the value in
        the widget will be saved on the robot and will be used when the robot
        program next starts rather than ``defaultValue``.
        
        :param title:        the title of the widget
        :param defaultValue: the default value of the widget
        
        :returns: a widget to display the sendable data
                  @see Add(std::string_view, std::span<const std::string>)
                  Add(std::string_view title, std::span<const std::string> defaultValue)
        """
    @typing.overload
    def addRaw(self, title: str, supplier: typing.Callable[[], list[int]]) -> SuppliedRawValueWidget:
        """
        Adds a widget to this container. The widget will display the data provided
        by the value supplier. Changes made on the dashboard will not propagate to
        the widget object, and will be overridden by values from the value
        supplier.
        
        :param title:    the title of the widget
        :param supplier: the supplier for values
        
        :returns: a widget to display data
        """
    @typing.overload
    def addRaw(self, title: str, typeString: str, supplier: typing.Callable[[], list[int]]) -> SuppliedRawValueWidget:
        """
        Adds a widget to this container. The widget will display the data provided
        by the value supplier. Changes made on the dashboard will not propagate to
        the widget object, and will be overridden by values from the value
        supplier.
        
        :param title:      the title of the widget
        :param typeString: the NT type string
        :param supplier:   the supplier for values
        
        :returns: a widget to display data
        """
    def addString(self, title: str, supplier: typing.Callable[[], str]) -> SuppliedStringValueWidget:
        """
        Adds a widget to this container. The widget will display the data provided
        by the value supplier. Changes made on the dashboard will not propagate to
        the widget object, and will be overridden by values from the value
        supplier.
        
        :param title:    the title of the widget
        :param supplier: the supplier for values
        
        :returns: a widget to display data
        """
    def addStringArray(self, title: str, supplier: typing.Callable[[], list[str]]) -> SuppliedStringListValueWidget:
        """
        Adds a widget to this container. The widget will display the data provided
        by the value supplier. Changes made on the dashboard will not propagate to
        the widget object, and will be overridden by values from the value
        supplier.
        
        :param title:    the title of the widget
        :param supplier: the supplier for values
        
        :returns: a widget to display data
        """
    def disableIfActuator(self) -> None:
        ...
    def enableIfActuator(self) -> None:
        ...
    def getComponents(self) -> list[ShuffleboardComponentBase]:
        """
        Gets the components that are direct children of this container.
        """
    @typing.overload
    def getLayout(self, title: str, type: BuiltInLayouts) -> ShuffleboardLayout:
        """
        Gets the layout with the given type and title, creating it if it does not
        already exist at the time this method is called.
        
        :param title: the title of the layout
        :param type:  the type of the layout, eg "List" or "Grid"
        
        :returns: the layout
        """
    @typing.overload
    def getLayout(self, title: str, type: LayoutType) -> ShuffleboardLayout:
        """
        Gets the layout with the given type and title, creating it if it does not
        already exist at the time this method is called.
        
        :param title: the title of the layout
        :param type:  the type of the layout, eg "List" or "Grid"
        
        :returns: the layout
        """
    @typing.overload
    def getLayout(self, title: str, type: str) -> ShuffleboardLayout:
        """
        Gets the layout with the given type and title, creating it if it does not
        already exist at the time this method is called. Note: this method should
        only be used to use a layout type that is not already built into
        Shuffleboard. To use a layout built into Shuffleboard, use
        GetLayout(std::string_view, const LayoutType&) and the layouts in
        BuiltInLayouts.
        
        :param title: the title of the layout
        :param type:  the type of the layout, eg "List Layout" or "Grid Layout"
        
        :returns: the layout
                  @see GetLayout(std::string_view, const LayoutType&)
        """
    @typing.overload
    def getLayout(self, title: str) -> ShuffleboardLayout:
        """
        Gets the already-defined layout in this container with the given title.
        
        <pre>{@code
        Shuffleboard::GetTab("Example Tab")->getLayout("My Layout",
        &BuiltInLayouts.kList);
        
        // Later...
        Shuffleboard::GetTab("Example Tab")->GetLayout("My Layout");
        }</pre>
        
        :param title: the title of the layout to get
        
        :returns: the layout with the given title
                  @throws if no layout has yet been defined with the given title
        """
class ShuffleboardEventImportance:
    """
    Members:
    
      kTrivial
    
      kLow
    
      kNormal
    
      kHigh
    
      kCritical
    """
    __members__: typing.ClassVar[dict[str, ShuffleboardEventImportance]]  # value = {'kTrivial': <ShuffleboardEventImportance.kTrivial: 0>, 'kLow': <ShuffleboardEventImportance.kLow: 1>, 'kNormal': <ShuffleboardEventImportance.kNormal: 2>, 'kHigh': <ShuffleboardEventImportance.kHigh: 3>, 'kCritical': <ShuffleboardEventImportance.kCritical: 4>}
    kCritical: typing.ClassVar[ShuffleboardEventImportance]  # value = <ShuffleboardEventImportance.kCritical: 4>
    kHigh: typing.ClassVar[ShuffleboardEventImportance]  # value = <ShuffleboardEventImportance.kHigh: 3>
    kLow: typing.ClassVar[ShuffleboardEventImportance]  # value = <ShuffleboardEventImportance.kLow: 1>
    kNormal: typing.ClassVar[ShuffleboardEventImportance]  # value = <ShuffleboardEventImportance.kNormal: 2>
    kTrivial: typing.ClassVar[ShuffleboardEventImportance]  # value = <ShuffleboardEventImportance.kTrivial: 0>
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
class ShuffleboardLayout(_LayoutComponent, ShuffleboardContainer):
    """
    A layout in a Shuffleboard tab. Layouts can contain widgets and other
    layouts.
    """
    def __init__(self, parent: ShuffleboardContainer, name: str, type: str) -> None:
        ...
    def buildInto(self, parentTable: ntcore._ntcore.NetworkTable, metaTable: ntcore._ntcore.NetworkTable) -> None:
        ...
class ShuffleboardTab(ShuffleboardContainer):
    """
    Represents a tab in the Shuffleboard dashboard. Widgets can be added to the
    tab with Add(Sendable), Add(std::string_view, Object), and
    Add(String, Sendable). Widgets can also be added to layouts with
    GetLayout(std::string_view, std::string_view); layouts can be nested
    arbitrarily deep (note that too many levels may make deeper components
    unusable).
    """
    def __init__(self, root: _ShuffleboardRoot, title: str) -> None:
        ...
    def buildInto(self, parentTable: ntcore._ntcore.NetworkTable, metaTable: ntcore._ntcore.NetworkTable) -> None:
        ...
    def getRoot(self) -> _ShuffleboardRoot:
        ...
class ShuffleboardValue:
    def buildInto(self, parentTable: ntcore._ntcore.NetworkTable, metaTable: ntcore._ntcore.NetworkTable) -> None:
        """
        Builds the entries for this value.
        
        :param parentTable: The table containing all the data for the parent. Values
                            that require a complex entry or table structure should
                            call ``parentTable.getSubtable(getTitle())`` to get
                            the table to put data into. Values that only use a
                            single entry should call
                            ``parentTable.getEntry(getTitle())`` to get that
                            entry.
        :param metaTable:   The table containing all the metadata for this value and
                            its sub-values
        """
    def disableIfActuator(self) -> None:
        """
        Disables user control of this widget in the Shuffleboard application.
        
        This method is package-private to prevent users from enabling control
        themselves. Has no effect if the sendable is not marked as an actuator with
        SendableBuilder::SetActuator().
        """
    def enableIfActuator(self) -> None:
        """
        Enables user control of this widget in the Shuffleboard application.
        
        This method is package-private to prevent users from enabling control
        themselves. Has no effect if the sendable is not marked as an actuator with
        SendableBuilder::SetActuator().
        """
    def getTitle(self) -> str:
        """
        Gets the title of this Shuffleboard value.
        """
class SimpleWidget(_SimpleWidget):
    """
    A Shuffleboard widget that handles a single data point such as a number or
    string.
    """
    def __init__(self, parent: ShuffleboardContainer, title: str) -> None:
        ...
    def buildInto(self, parentTable: ntcore._ntcore.NetworkTable, metaTable: ntcore._ntcore.NetworkTable) -> None:
        ...
    @typing.overload
    def getEntry(self) -> ntcore._ntcore.GenericEntry:
        """
        Gets the NetworkTable entry that contains the data for this widget.
        The widget owns the entry; the returned pointer's lifetime is the same as
        that of the widget.
        """
    @typing.overload
    def getEntry(self, typeString: str) -> ntcore._ntcore.GenericEntry:
        """
        Gets the NetworkTable entry that contains the data for this widget.
        The widget owns the entry; the returned pointer's lifetime is the same as
        that of the widget.
        
        :param typeString: NT type string
        """
class SuppliedBoolListValueWidget(_SuppliedValueWidget_vector_bool):
    def buildInto(self, parentTable: ntcore._ntcore.NetworkTable, metaTable: ntcore._ntcore.NetworkTable) -> None:
        ...
class SuppliedBoolValueWidget(_SuppliedValueWidget_bool):
    def buildInto(self, parentTable: ntcore._ntcore.NetworkTable, metaTable: ntcore._ntcore.NetworkTable) -> None:
        ...
class SuppliedDoubleListValueWidget(_SuppliedValueWidget_vector_double):
    def buildInto(self, parentTable: ntcore._ntcore.NetworkTable, metaTable: ntcore._ntcore.NetworkTable) -> None:
        ...
class SuppliedDoubleValueWidget(_SuppliedValueWidget_double):
    def buildInto(self, parentTable: ntcore._ntcore.NetworkTable, metaTable: ntcore._ntcore.NetworkTable) -> None:
        ...
class SuppliedFloatListValueWidget(_SuppliedValueWidget_vector_float):
    def buildInto(self, parentTable: ntcore._ntcore.NetworkTable, metaTable: ntcore._ntcore.NetworkTable) -> None:
        ...
class SuppliedFloatValueWidget(_SuppliedValueWidget_float):
    def buildInto(self, parentTable: ntcore._ntcore.NetworkTable, metaTable: ntcore._ntcore.NetworkTable) -> None:
        ...
class SuppliedIntListValueWidget(_SuppliedValueWidget_vector_int):
    def buildInto(self, parentTable: ntcore._ntcore.NetworkTable, metaTable: ntcore._ntcore.NetworkTable) -> None:
        ...
class SuppliedIntegerValueWidget(_SuppliedValueWidget_integer):
    def buildInto(self, parentTable: ntcore._ntcore.NetworkTable, metaTable: ntcore._ntcore.NetworkTable) -> None:
        ...
class SuppliedRawValueWidget(_SuppliedValueWidget_vector_raw):
    def buildInto(self, parentTable: ntcore._ntcore.NetworkTable, metaTable: ntcore._ntcore.NetworkTable) -> None:
        ...
class SuppliedStringListValueWidget(_SuppliedValueWidget_vector_string):
    def buildInto(self, parentTable: ntcore._ntcore.NetworkTable, metaTable: ntcore._ntcore.NetworkTable) -> None:
        ...
class SuppliedStringValueWidget(_SuppliedValueWidget_string):
    def buildInto(self, parentTable: ntcore._ntcore.NetworkTable, metaTable: ntcore._ntcore.NetworkTable) -> None:
        ...
class WidgetType:
    """
    Represents the type of a widget in Shuffleboard. Using this is preferred over
    specifying raw strings, to avoid typos and having to know or look up the
    exact string name for a desired widget.
    
    @see BuiltInWidgets the built-in widget types
    """
    def __init__(self, widgetName: str) -> None:
        ...
    def getWidgetName(self) -> str:
        """
        Gets the string type of the widget as defined by that widget in
        Shuffleboard.
        """
class _ComplexComponent(ShuffleboardComponentBase):
    """
    A generic component in Shuffleboard.
    
    @tparam Derived the self type
    """
    def withPosition(self, columnIndex: int, rowIndex: int) -> ComplexWidget:
        """
        Sets the position of this component in the tab. This has no effect if this
        component is inside a layout.
        
        If the position of a single component is set, it is recommended to set the
        positions of *all* components inside a tab to prevent Shuffleboard
        from automatically placing another component there before the one with the
        specific position is sent.
        
        :param columnIndex: the column in the tab to place this component
        :param rowIndex:    the row in the tab to place this component
        
        :returns: this component
        """
    def withProperties(self, properties: dict[str, ntcore._ntcore.Value]) -> ComplexWidget:
        """
        Sets custom properties for this component. Property names are
        case-sensitive and whitespace-insensitive (capitalization and spaces do not
        matter).
        
        :param properties: the properties for this component
        
        :returns: this component
        """
    def withSize(self, width: int, height: int) -> ComplexWidget:
        """
        Sets the size of this component in the tab. This has no effect if this
        component is inside a layout.
        
        :param width:  how many columns wide the component should be
        :param height: how many rows high the component should be
        
        :returns: this component
        """
class _ComplexWidget(_ComplexComponent):
    """
    Abstract superclass for widgets.
    
    This class is package-private to minimize API surface area.
    
    @tparam Derived the self type
    """
    @typing.overload
    def withWidget(self, widgetType: BuiltInWidgets) -> ComplexWidget:
        """
        Sets the type of widget used to display the data. If not set, the default
        widget type will be used.
        
        :param widgetType: the type of the widget used to display the data
        
        :returns: this widget object
                  @see BuiltInWidgets
        """
    @typing.overload
    def withWidget(self, widgetType: WidgetType) -> ComplexWidget:
        """
        Sets the type of widget used to display the data. If not set, the default
        widget type will be used.
        
        :param widgetType: the type of the widget used to display the data
        
        :returns: this widget object
        """
    @typing.overload
    def withWidget(self, widgetType: str) -> ComplexWidget:
        """
        Sets the type of widget used to display the data. If not set, the default
        widget type will be used. This method should only be used to use a widget
        that does not come built into Shuffleboard (i.e. one that comes with a
        custom or third-party plugin). To use a widget that is built into
        Shuffleboard, use WithWidget(WidgetType) and BuiltInWidgets.
        
        :param widgetType: the type of the widget used to display the data
        
        :returns: this widget object
        """
class _LayoutComponent(ShuffleboardComponentBase):
    """
    A generic component in Shuffleboard.
    
    @tparam Derived the self type
    """
    def withPosition(self, columnIndex: int, rowIndex: int) -> ShuffleboardLayout:
        """
        Sets the position of this component in the tab. This has no effect if this
        component is inside a layout.
        
        If the position of a single component is set, it is recommended to set the
        positions of *all* components inside a tab to prevent Shuffleboard
        from automatically placing another component there before the one with the
        specific position is sent.
        
        :param columnIndex: the column in the tab to place this component
        :param rowIndex:    the row in the tab to place this component
        
        :returns: this component
        """
    def withProperties(self, properties: dict[str, ntcore._ntcore.Value]) -> ShuffleboardLayout:
        """
        Sets custom properties for this component. Property names are
        case-sensitive and whitespace-insensitive (capitalization and spaces do not
        matter).
        
        :param properties: the properties for this component
        
        :returns: this component
        """
    def withSize(self, width: int, height: int) -> ShuffleboardLayout:
        """
        Sets the size of this component in the tab. This has no effect if this
        component is inside a layout.
        
        :param width:  how many columns wide the component should be
        :param height: how many rows high the component should be
        
        :returns: this component
        """
class _ShuffleboardInstance(_ShuffleboardRoot):
    def __init__(self, ntInstance: ntcore._ntcore.NetworkTableInstance) -> None:
        ...
    def disableActuatorWidgets(self) -> None:
        ...
    def enableActuatorWidgets(self) -> None:
        ...
    def getTab(self, title: str) -> ShuffleboardTab:
        ...
    @typing.overload
    def selectTab(self, index: int) -> None:
        ...
    @typing.overload
    def selectTab(self, param0: str) -> None:
        ...
    def update(self) -> None:
        ...
class _ShuffleboardRoot:
    """
    The root of the data placed in Shuffleboard. It contains the tabs, but no
    data is placed directly in the root.
    
    This class is package-private to minimize API surface area.
    """
    def __init__(self) -> None:
        ...
    def disableActuatorWidgets(self) -> None:
        """
        Disables all widgets in Shuffleboard that offer user control over
        actuators.
        """
    def enableActuatorWidgets(self) -> None:
        """
        Enables all widgets in Shuffleboard that offer user control over actuators.
        """
    def getTab(self, title: str) -> ShuffleboardTab:
        """
        Gets the tab with the given title, creating it if it does not already
        exist.
        
        :param title: the title of the tab
        
        :returns: the tab with the given title
        """
    @typing.overload
    def selectTab(self, index: int) -> None:
        """
        Selects the tab in the dashboard with the given index in the range
        [0..n-1], where *n* is the number of tabs in the dashboard at the time
        this method is called.
        
        :param index: the index of the tab to select
        """
    @typing.overload
    def selectTab(self, title: str) -> None:
        """
        Selects the tab in the dashboard with the given title.
        
        :param title: the title of the tab to select
        """
    def update(self) -> None:
        """
        Updates all tabs.
        """
class _SimpleComponent(ShuffleboardComponentBase):
    """
    A generic component in Shuffleboard.
    
    @tparam Derived the self type
    """
    def withPosition(self, columnIndex: int, rowIndex: int) -> SimpleWidget:
        """
        Sets the position of this component in the tab. This has no effect if this
        component is inside a layout.
        
        If the position of a single component is set, it is recommended to set the
        positions of *all* components inside a tab to prevent Shuffleboard
        from automatically placing another component there before the one with the
        specific position is sent.
        
        :param columnIndex: the column in the tab to place this component
        :param rowIndex:    the row in the tab to place this component
        
        :returns: this component
        """
    def withProperties(self, properties: dict[str, ntcore._ntcore.Value]) -> SimpleWidget:
        """
        Sets custom properties for this component. Property names are
        case-sensitive and whitespace-insensitive (capitalization and spaces do not
        matter).
        
        :param properties: the properties for this component
        
        :returns: this component
        """
    def withSize(self, width: int, height: int) -> SimpleWidget:
        """
        Sets the size of this component in the tab. This has no effect if this
        component is inside a layout.
        
        :param width:  how many columns wide the component should be
        :param height: how many rows high the component should be
        
        :returns: this component
        """
class _SimpleWidget(_SimpleComponent):
    """
    Abstract superclass for widgets.
    
    This class is package-private to minimize API surface area.
    
    @tparam Derived the self type
    """
    @typing.overload
    def withWidget(self, widgetType: BuiltInWidgets) -> SimpleWidget:
        """
        Sets the type of widget used to display the data. If not set, the default
        widget type will be used.
        
        :param widgetType: the type of the widget used to display the data
        
        :returns: this widget object
                  @see BuiltInWidgets
        """
    @typing.overload
    def withWidget(self, widgetType: WidgetType) -> SimpleWidget:
        """
        Sets the type of widget used to display the data. If not set, the default
        widget type will be used.
        
        :param widgetType: the type of the widget used to display the data
        
        :returns: this widget object
        """
    @typing.overload
    def withWidget(self, widgetType: str) -> SimpleWidget:
        """
        Sets the type of widget used to display the data. If not set, the default
        widget type will be used. This method should only be used to use a widget
        that does not come built into Shuffleboard (i.e. one that comes with a
        custom or third-party plugin). To use a widget that is built into
        Shuffleboard, use WithWidget(WidgetType) and BuiltInWidgets.
        
        :param widgetType: the type of the widget used to display the data
        
        :returns: this widget object
        """
class _SuppliedValueComponent_bool(ShuffleboardComponentBase):
    """
    A generic component in Shuffleboard.
    
    @tparam Derived the self type
    """
    def withPosition(self, columnIndex: int, rowIndex: int) -> SuppliedBoolValueWidget:
        """
        Sets the position of this component in the tab. This has no effect if this
        component is inside a layout.
        
        If the position of a single component is set, it is recommended to set the
        positions of *all* components inside a tab to prevent Shuffleboard
        from automatically placing another component there before the one with the
        specific position is sent.
        
        :param columnIndex: the column in the tab to place this component
        :param rowIndex:    the row in the tab to place this component
        
        :returns: this component
        """
    def withProperties(self, properties: dict[str, ntcore._ntcore.Value]) -> SuppliedBoolValueWidget:
        """
        Sets custom properties for this component. Property names are
        case-sensitive and whitespace-insensitive (capitalization and spaces do not
        matter).
        
        :param properties: the properties for this component
        
        :returns: this component
        """
    def withSize(self, width: int, height: int) -> SuppliedBoolValueWidget:
        """
        Sets the size of this component in the tab. This has no effect if this
        component is inside a layout.
        
        :param width:  how many columns wide the component should be
        :param height: how many rows high the component should be
        
        :returns: this component
        """
class _SuppliedValueComponent_double(ShuffleboardComponentBase):
    """
    A generic component in Shuffleboard.
    
    @tparam Derived the self type
    """
    def withPosition(self, columnIndex: int, rowIndex: int) -> SuppliedDoubleValueWidget:
        """
        Sets the position of this component in the tab. This has no effect if this
        component is inside a layout.
        
        If the position of a single component is set, it is recommended to set the
        positions of *all* components inside a tab to prevent Shuffleboard
        from automatically placing another component there before the one with the
        specific position is sent.
        
        :param columnIndex: the column in the tab to place this component
        :param rowIndex:    the row in the tab to place this component
        
        :returns: this component
        """
    def withProperties(self, properties: dict[str, ntcore._ntcore.Value]) -> SuppliedDoubleValueWidget:
        """
        Sets custom properties for this component. Property names are
        case-sensitive and whitespace-insensitive (capitalization and spaces do not
        matter).
        
        :param properties: the properties for this component
        
        :returns: this component
        """
    def withSize(self, width: int, height: int) -> SuppliedDoubleValueWidget:
        """
        Sets the size of this component in the tab. This has no effect if this
        component is inside a layout.
        
        :param width:  how many columns wide the component should be
        :param height: how many rows high the component should be
        
        :returns: this component
        """
class _SuppliedValueComponent_float(ShuffleboardComponentBase):
    """
    A generic component in Shuffleboard.
    
    @tparam Derived the self type
    """
    def withPosition(self, columnIndex: int, rowIndex: int) -> SuppliedFloatValueWidget:
        """
        Sets the position of this component in the tab. This has no effect if this
        component is inside a layout.
        
        If the position of a single component is set, it is recommended to set the
        positions of *all* components inside a tab to prevent Shuffleboard
        from automatically placing another component there before the one with the
        specific position is sent.
        
        :param columnIndex: the column in the tab to place this component
        :param rowIndex:    the row in the tab to place this component
        
        :returns: this component
        """
    def withProperties(self, properties: dict[str, ntcore._ntcore.Value]) -> SuppliedFloatValueWidget:
        """
        Sets custom properties for this component. Property names are
        case-sensitive and whitespace-insensitive (capitalization and spaces do not
        matter).
        
        :param properties: the properties for this component
        
        :returns: this component
        """
    def withSize(self, width: int, height: int) -> SuppliedFloatValueWidget:
        """
        Sets the size of this component in the tab. This has no effect if this
        component is inside a layout.
        
        :param width:  how many columns wide the component should be
        :param height: how many rows high the component should be
        
        :returns: this component
        """
class _SuppliedValueComponent_integer(ShuffleboardComponentBase):
    """
    A generic component in Shuffleboard.
    
    @tparam Derived the self type
    """
    def withPosition(self, columnIndex: int, rowIndex: int) -> SuppliedIntegerValueWidget:
        """
        Sets the position of this component in the tab. This has no effect if this
        component is inside a layout.
        
        If the position of a single component is set, it is recommended to set the
        positions of *all* components inside a tab to prevent Shuffleboard
        from automatically placing another component there before the one with the
        specific position is sent.
        
        :param columnIndex: the column in the tab to place this component
        :param rowIndex:    the row in the tab to place this component
        
        :returns: this component
        """
    def withProperties(self, properties: dict[str, ntcore._ntcore.Value]) -> SuppliedIntegerValueWidget:
        """
        Sets custom properties for this component. Property names are
        case-sensitive and whitespace-insensitive (capitalization and spaces do not
        matter).
        
        :param properties: the properties for this component
        
        :returns: this component
        """
    def withSize(self, width: int, height: int) -> SuppliedIntegerValueWidget:
        """
        Sets the size of this component in the tab. This has no effect if this
        component is inside a layout.
        
        :param width:  how many columns wide the component should be
        :param height: how many rows high the component should be
        
        :returns: this component
        """
class _SuppliedValueComponent_string(ShuffleboardComponentBase):
    """
    A generic component in Shuffleboard.
    
    @tparam Derived the self type
    """
    def withPosition(self, columnIndex: int, rowIndex: int) -> SuppliedStringValueWidget:
        """
        Sets the position of this component in the tab. This has no effect if this
        component is inside a layout.
        
        If the position of a single component is set, it is recommended to set the
        positions of *all* components inside a tab to prevent Shuffleboard
        from automatically placing another component there before the one with the
        specific position is sent.
        
        :param columnIndex: the column in the tab to place this component
        :param rowIndex:    the row in the tab to place this component
        
        :returns: this component
        """
    def withProperties(self, properties: dict[str, ntcore._ntcore.Value]) -> SuppliedStringValueWidget:
        """
        Sets custom properties for this component. Property names are
        case-sensitive and whitespace-insensitive (capitalization and spaces do not
        matter).
        
        :param properties: the properties for this component
        
        :returns: this component
        """
    def withSize(self, width: int, height: int) -> SuppliedStringValueWidget:
        """
        Sets the size of this component in the tab. This has no effect if this
        component is inside a layout.
        
        :param width:  how many columns wide the component should be
        :param height: how many rows high the component should be
        
        :returns: this component
        """
class _SuppliedValueComponent_vector_bool(ShuffleboardComponentBase):
    """
    A generic component in Shuffleboard.
    
    @tparam Derived the self type
    """
    def withPosition(self, columnIndex: int, rowIndex: int) -> SuppliedBoolListValueWidget:
        """
        Sets the position of this component in the tab. This has no effect if this
        component is inside a layout.
        
        If the position of a single component is set, it is recommended to set the
        positions of *all* components inside a tab to prevent Shuffleboard
        from automatically placing another component there before the one with the
        specific position is sent.
        
        :param columnIndex: the column in the tab to place this component
        :param rowIndex:    the row in the tab to place this component
        
        :returns: this component
        """
    def withProperties(self, properties: dict[str, ntcore._ntcore.Value]) -> SuppliedBoolListValueWidget:
        """
        Sets custom properties for this component. Property names are
        case-sensitive and whitespace-insensitive (capitalization and spaces do not
        matter).
        
        :param properties: the properties for this component
        
        :returns: this component
        """
    def withSize(self, width: int, height: int) -> SuppliedBoolListValueWidget:
        """
        Sets the size of this component in the tab. This has no effect if this
        component is inside a layout.
        
        :param width:  how many columns wide the component should be
        :param height: how many rows high the component should be
        
        :returns: this component
        """
class _SuppliedValueComponent_vector_double(ShuffleboardComponentBase):
    """
    A generic component in Shuffleboard.
    
    @tparam Derived the self type
    """
    def withPosition(self, columnIndex: int, rowIndex: int) -> SuppliedDoubleListValueWidget:
        """
        Sets the position of this component in the tab. This has no effect if this
        component is inside a layout.
        
        If the position of a single component is set, it is recommended to set the
        positions of *all* components inside a tab to prevent Shuffleboard
        from automatically placing another component there before the one with the
        specific position is sent.
        
        :param columnIndex: the column in the tab to place this component
        :param rowIndex:    the row in the tab to place this component
        
        :returns: this component
        """
    def withProperties(self, properties: dict[str, ntcore._ntcore.Value]) -> SuppliedDoubleListValueWidget:
        """
        Sets custom properties for this component. Property names are
        case-sensitive and whitespace-insensitive (capitalization and spaces do not
        matter).
        
        :param properties: the properties for this component
        
        :returns: this component
        """
    def withSize(self, width: int, height: int) -> SuppliedDoubleListValueWidget:
        """
        Sets the size of this component in the tab. This has no effect if this
        component is inside a layout.
        
        :param width:  how many columns wide the component should be
        :param height: how many rows high the component should be
        
        :returns: this component
        """
class _SuppliedValueComponent_vector_float(ShuffleboardComponentBase):
    """
    A generic component in Shuffleboard.
    
    @tparam Derived the self type
    """
    def withPosition(self, columnIndex: int, rowIndex: int) -> SuppliedFloatListValueWidget:
        """
        Sets the position of this component in the tab. This has no effect if this
        component is inside a layout.
        
        If the position of a single component is set, it is recommended to set the
        positions of *all* components inside a tab to prevent Shuffleboard
        from automatically placing another component there before the one with the
        specific position is sent.
        
        :param columnIndex: the column in the tab to place this component
        :param rowIndex:    the row in the tab to place this component
        
        :returns: this component
        """
    def withProperties(self, properties: dict[str, ntcore._ntcore.Value]) -> SuppliedFloatListValueWidget:
        """
        Sets custom properties for this component. Property names are
        case-sensitive and whitespace-insensitive (capitalization and spaces do not
        matter).
        
        :param properties: the properties for this component
        
        :returns: this component
        """
    def withSize(self, width: int, height: int) -> SuppliedFloatListValueWidget:
        """
        Sets the size of this component in the tab. This has no effect if this
        component is inside a layout.
        
        :param width:  how many columns wide the component should be
        :param height: how many rows high the component should be
        
        :returns: this component
        """
class _SuppliedValueComponent_vector_int(ShuffleboardComponentBase):
    """
    A generic component in Shuffleboard.
    
    @tparam Derived the self type
    """
    def withPosition(self, columnIndex: int, rowIndex: int) -> SuppliedIntListValueWidget:
        """
        Sets the position of this component in the tab. This has no effect if this
        component is inside a layout.
        
        If the position of a single component is set, it is recommended to set the
        positions of *all* components inside a tab to prevent Shuffleboard
        from automatically placing another component there before the one with the
        specific position is sent.
        
        :param columnIndex: the column in the tab to place this component
        :param rowIndex:    the row in the tab to place this component
        
        :returns: this component
        """
    def withProperties(self, properties: dict[str, ntcore._ntcore.Value]) -> SuppliedIntListValueWidget:
        """
        Sets custom properties for this component. Property names are
        case-sensitive and whitespace-insensitive (capitalization and spaces do not
        matter).
        
        :param properties: the properties for this component
        
        :returns: this component
        """
    def withSize(self, width: int, height: int) -> SuppliedIntListValueWidget:
        """
        Sets the size of this component in the tab. This has no effect if this
        component is inside a layout.
        
        :param width:  how many columns wide the component should be
        :param height: how many rows high the component should be
        
        :returns: this component
        """
class _SuppliedValueComponent_vector_raw(ShuffleboardComponentBase):
    """
    A generic component in Shuffleboard.
    
    @tparam Derived the self type
    """
    def withPosition(self, columnIndex: int, rowIndex: int) -> SuppliedRawValueWidget:
        """
        Sets the position of this component in the tab. This has no effect if this
        component is inside a layout.
        
        If the position of a single component is set, it is recommended to set the
        positions of *all* components inside a tab to prevent Shuffleboard
        from automatically placing another component there before the one with the
        specific position is sent.
        
        :param columnIndex: the column in the tab to place this component
        :param rowIndex:    the row in the tab to place this component
        
        :returns: this component
        """
    def withProperties(self, properties: dict[str, ntcore._ntcore.Value]) -> SuppliedRawValueWidget:
        """
        Sets custom properties for this component. Property names are
        case-sensitive and whitespace-insensitive (capitalization and spaces do not
        matter).
        
        :param properties: the properties for this component
        
        :returns: this component
        """
    def withSize(self, width: int, height: int) -> SuppliedRawValueWidget:
        """
        Sets the size of this component in the tab. This has no effect if this
        component is inside a layout.
        
        :param width:  how many columns wide the component should be
        :param height: how many rows high the component should be
        
        :returns: this component
        """
class _SuppliedValueComponent_vector_string(ShuffleboardComponentBase):
    """
    A generic component in Shuffleboard.
    
    @tparam Derived the self type
    """
    def withPosition(self, columnIndex: int, rowIndex: int) -> SuppliedStringListValueWidget:
        """
        Sets the position of this component in the tab. This has no effect if this
        component is inside a layout.
        
        If the position of a single component is set, it is recommended to set the
        positions of *all* components inside a tab to prevent Shuffleboard
        from automatically placing another component there before the one with the
        specific position is sent.
        
        :param columnIndex: the column in the tab to place this component
        :param rowIndex:    the row in the tab to place this component
        
        :returns: this component
        """
    def withProperties(self, properties: dict[str, ntcore._ntcore.Value]) -> SuppliedStringListValueWidget:
        """
        Sets custom properties for this component. Property names are
        case-sensitive and whitespace-insensitive (capitalization and spaces do not
        matter).
        
        :param properties: the properties for this component
        
        :returns: this component
        """
    def withSize(self, width: int, height: int) -> SuppliedStringListValueWidget:
        """
        Sets the size of this component in the tab. This has no effect if this
        component is inside a layout.
        
        :param width:  how many columns wide the component should be
        :param height: how many rows high the component should be
        
        :returns: this component
        """
class _SuppliedValueWidget_bool(_SuppliedValueComponent_bool):
    """
    Abstract superclass for widgets.
    
    This class is package-private to minimize API surface area.
    
    @tparam Derived the self type
    """
    @typing.overload
    def withWidget(self, widgetType: BuiltInWidgets) -> SuppliedBoolValueWidget:
        """
        Sets the type of widget used to display the data. If not set, the default
        widget type will be used.
        
        :param widgetType: the type of the widget used to display the data
        
        :returns: this widget object
                  @see BuiltInWidgets
        """
    @typing.overload
    def withWidget(self, widgetType: WidgetType) -> SuppliedBoolValueWidget:
        """
        Sets the type of widget used to display the data. If not set, the default
        widget type will be used.
        
        :param widgetType: the type of the widget used to display the data
        
        :returns: this widget object
        """
    @typing.overload
    def withWidget(self, widgetType: str) -> SuppliedBoolValueWidget:
        """
        Sets the type of widget used to display the data. If not set, the default
        widget type will be used. This method should only be used to use a widget
        that does not come built into Shuffleboard (i.e. one that comes with a
        custom or third-party plugin). To use a widget that is built into
        Shuffleboard, use WithWidget(WidgetType) and BuiltInWidgets.
        
        :param widgetType: the type of the widget used to display the data
        
        :returns: this widget object
        """
class _SuppliedValueWidget_double(_SuppliedValueComponent_double):
    """
    Abstract superclass for widgets.
    
    This class is package-private to minimize API surface area.
    
    @tparam Derived the self type
    """
    @typing.overload
    def withWidget(self, widgetType: BuiltInWidgets) -> SuppliedDoubleValueWidget:
        """
        Sets the type of widget used to display the data. If not set, the default
        widget type will be used.
        
        :param widgetType: the type of the widget used to display the data
        
        :returns: this widget object
                  @see BuiltInWidgets
        """
    @typing.overload
    def withWidget(self, widgetType: WidgetType) -> SuppliedDoubleValueWidget:
        """
        Sets the type of widget used to display the data. If not set, the default
        widget type will be used.
        
        :param widgetType: the type of the widget used to display the data
        
        :returns: this widget object
        """
    @typing.overload
    def withWidget(self, widgetType: str) -> SuppliedDoubleValueWidget:
        """
        Sets the type of widget used to display the data. If not set, the default
        widget type will be used. This method should only be used to use a widget
        that does not come built into Shuffleboard (i.e. one that comes with a
        custom or third-party plugin). To use a widget that is built into
        Shuffleboard, use WithWidget(WidgetType) and BuiltInWidgets.
        
        :param widgetType: the type of the widget used to display the data
        
        :returns: this widget object
        """
class _SuppliedValueWidget_float(_SuppliedValueComponent_float):
    """
    Abstract superclass for widgets.
    
    This class is package-private to minimize API surface area.
    
    @tparam Derived the self type
    """
    @typing.overload
    def withWidget(self, widgetType: BuiltInWidgets) -> SuppliedFloatValueWidget:
        """
        Sets the type of widget used to display the data. If not set, the default
        widget type will be used.
        
        :param widgetType: the type of the widget used to display the data
        
        :returns: this widget object
                  @see BuiltInWidgets
        """
    @typing.overload
    def withWidget(self, widgetType: WidgetType) -> SuppliedFloatValueWidget:
        """
        Sets the type of widget used to display the data. If not set, the default
        widget type will be used.
        
        :param widgetType: the type of the widget used to display the data
        
        :returns: this widget object
        """
    @typing.overload
    def withWidget(self, widgetType: str) -> SuppliedFloatValueWidget:
        """
        Sets the type of widget used to display the data. If not set, the default
        widget type will be used. This method should only be used to use a widget
        that does not come built into Shuffleboard (i.e. one that comes with a
        custom or third-party plugin). To use a widget that is built into
        Shuffleboard, use WithWidget(WidgetType) and BuiltInWidgets.
        
        :param widgetType: the type of the widget used to display the data
        
        :returns: this widget object
        """
class _SuppliedValueWidget_integer(_SuppliedValueComponent_integer):
    """
    Abstract superclass for widgets.
    
    This class is package-private to minimize API surface area.
    
    @tparam Derived the self type
    """
    @typing.overload
    def withWidget(self, widgetType: BuiltInWidgets) -> SuppliedIntegerValueWidget:
        """
        Sets the type of widget used to display the data. If not set, the default
        widget type will be used.
        
        :param widgetType: the type of the widget used to display the data
        
        :returns: this widget object
                  @see BuiltInWidgets
        """
    @typing.overload
    def withWidget(self, widgetType: WidgetType) -> SuppliedIntegerValueWidget:
        """
        Sets the type of widget used to display the data. If not set, the default
        widget type will be used.
        
        :param widgetType: the type of the widget used to display the data
        
        :returns: this widget object
        """
    @typing.overload
    def withWidget(self, widgetType: str) -> SuppliedIntegerValueWidget:
        """
        Sets the type of widget used to display the data. If not set, the default
        widget type will be used. This method should only be used to use a widget
        that does not come built into Shuffleboard (i.e. one that comes with a
        custom or third-party plugin). To use a widget that is built into
        Shuffleboard, use WithWidget(WidgetType) and BuiltInWidgets.
        
        :param widgetType: the type of the widget used to display the data
        
        :returns: this widget object
        """
class _SuppliedValueWidget_string(_SuppliedValueComponent_string):
    """
    Abstract superclass for widgets.
    
    This class is package-private to minimize API surface area.
    
    @tparam Derived the self type
    """
    @typing.overload
    def withWidget(self, widgetType: BuiltInWidgets) -> SuppliedStringValueWidget:
        """
        Sets the type of widget used to display the data. If not set, the default
        widget type will be used.
        
        :param widgetType: the type of the widget used to display the data
        
        :returns: this widget object
                  @see BuiltInWidgets
        """
    @typing.overload
    def withWidget(self, widgetType: WidgetType) -> SuppliedStringValueWidget:
        """
        Sets the type of widget used to display the data. If not set, the default
        widget type will be used.
        
        :param widgetType: the type of the widget used to display the data
        
        :returns: this widget object
        """
    @typing.overload
    def withWidget(self, widgetType: str) -> SuppliedStringValueWidget:
        """
        Sets the type of widget used to display the data. If not set, the default
        widget type will be used. This method should only be used to use a widget
        that does not come built into Shuffleboard (i.e. one that comes with a
        custom or third-party plugin). To use a widget that is built into
        Shuffleboard, use WithWidget(WidgetType) and BuiltInWidgets.
        
        :param widgetType: the type of the widget used to display the data
        
        :returns: this widget object
        """
class _SuppliedValueWidget_vector_bool(_SuppliedValueComponent_vector_bool):
    """
    Abstract superclass for widgets.
    
    This class is package-private to minimize API surface area.
    
    @tparam Derived the self type
    """
    @typing.overload
    def withWidget(self, widgetType: BuiltInWidgets) -> SuppliedBoolListValueWidget:
        """
        Sets the type of widget used to display the data. If not set, the default
        widget type will be used.
        
        :param widgetType: the type of the widget used to display the data
        
        :returns: this widget object
                  @see BuiltInWidgets
        """
    @typing.overload
    def withWidget(self, widgetType: WidgetType) -> SuppliedBoolListValueWidget:
        """
        Sets the type of widget used to display the data. If not set, the default
        widget type will be used.
        
        :param widgetType: the type of the widget used to display the data
        
        :returns: this widget object
        """
    @typing.overload
    def withWidget(self, widgetType: str) -> SuppliedBoolListValueWidget:
        """
        Sets the type of widget used to display the data. If not set, the default
        widget type will be used. This method should only be used to use a widget
        that does not come built into Shuffleboard (i.e. one that comes with a
        custom or third-party plugin). To use a widget that is built into
        Shuffleboard, use WithWidget(WidgetType) and BuiltInWidgets.
        
        :param widgetType: the type of the widget used to display the data
        
        :returns: this widget object
        """
class _SuppliedValueWidget_vector_double(_SuppliedValueComponent_vector_double):
    """
    Abstract superclass for widgets.
    
    This class is package-private to minimize API surface area.
    
    @tparam Derived the self type
    """
    @typing.overload
    def withWidget(self, widgetType: BuiltInWidgets) -> SuppliedDoubleListValueWidget:
        """
        Sets the type of widget used to display the data. If not set, the default
        widget type will be used.
        
        :param widgetType: the type of the widget used to display the data
        
        :returns: this widget object
                  @see BuiltInWidgets
        """
    @typing.overload
    def withWidget(self, widgetType: WidgetType) -> SuppliedDoubleListValueWidget:
        """
        Sets the type of widget used to display the data. If not set, the default
        widget type will be used.
        
        :param widgetType: the type of the widget used to display the data
        
        :returns: this widget object
        """
    @typing.overload
    def withWidget(self, widgetType: str) -> SuppliedDoubleListValueWidget:
        """
        Sets the type of widget used to display the data. If not set, the default
        widget type will be used. This method should only be used to use a widget
        that does not come built into Shuffleboard (i.e. one that comes with a
        custom or third-party plugin). To use a widget that is built into
        Shuffleboard, use WithWidget(WidgetType) and BuiltInWidgets.
        
        :param widgetType: the type of the widget used to display the data
        
        :returns: this widget object
        """
class _SuppliedValueWidget_vector_float(_SuppliedValueComponent_vector_float):
    """
    Abstract superclass for widgets.
    
    This class is package-private to minimize API surface area.
    
    @tparam Derived the self type
    """
    @typing.overload
    def withWidget(self, widgetType: BuiltInWidgets) -> SuppliedFloatListValueWidget:
        """
        Sets the type of widget used to display the data. If not set, the default
        widget type will be used.
        
        :param widgetType: the type of the widget used to display the data
        
        :returns: this widget object
                  @see BuiltInWidgets
        """
    @typing.overload
    def withWidget(self, widgetType: WidgetType) -> SuppliedFloatListValueWidget:
        """
        Sets the type of widget used to display the data. If not set, the default
        widget type will be used.
        
        :param widgetType: the type of the widget used to display the data
        
        :returns: this widget object
        """
    @typing.overload
    def withWidget(self, widgetType: str) -> SuppliedFloatListValueWidget:
        """
        Sets the type of widget used to display the data. If not set, the default
        widget type will be used. This method should only be used to use a widget
        that does not come built into Shuffleboard (i.e. one that comes with a
        custom or third-party plugin). To use a widget that is built into
        Shuffleboard, use WithWidget(WidgetType) and BuiltInWidgets.
        
        :param widgetType: the type of the widget used to display the data
        
        :returns: this widget object
        """
class _SuppliedValueWidget_vector_int(_SuppliedValueComponent_vector_int):
    """
    Abstract superclass for widgets.
    
    This class is package-private to minimize API surface area.
    
    @tparam Derived the self type
    """
    @typing.overload
    def withWidget(self, widgetType: BuiltInWidgets) -> SuppliedIntListValueWidget:
        """
        Sets the type of widget used to display the data. If not set, the default
        widget type will be used.
        
        :param widgetType: the type of the widget used to display the data
        
        :returns: this widget object
                  @see BuiltInWidgets
        """
    @typing.overload
    def withWidget(self, widgetType: WidgetType) -> SuppliedIntListValueWidget:
        """
        Sets the type of widget used to display the data. If not set, the default
        widget type will be used.
        
        :param widgetType: the type of the widget used to display the data
        
        :returns: this widget object
        """
    @typing.overload
    def withWidget(self, widgetType: str) -> SuppliedIntListValueWidget:
        """
        Sets the type of widget used to display the data. If not set, the default
        widget type will be used. This method should only be used to use a widget
        that does not come built into Shuffleboard (i.e. one that comes with a
        custom or third-party plugin). To use a widget that is built into
        Shuffleboard, use WithWidget(WidgetType) and BuiltInWidgets.
        
        :param widgetType: the type of the widget used to display the data
        
        :returns: this widget object
        """
class _SuppliedValueWidget_vector_raw(_SuppliedValueComponent_vector_raw):
    """
    Abstract superclass for widgets.
    
    This class is package-private to minimize API surface area.
    
    @tparam Derived the self type
    """
    @typing.overload
    def withWidget(self, widgetType: BuiltInWidgets) -> SuppliedRawValueWidget:
        """
        Sets the type of widget used to display the data. If not set, the default
        widget type will be used.
        
        :param widgetType: the type of the widget used to display the data
        
        :returns: this widget object
                  @see BuiltInWidgets
        """
    @typing.overload
    def withWidget(self, widgetType: WidgetType) -> SuppliedRawValueWidget:
        """
        Sets the type of widget used to display the data. If not set, the default
        widget type will be used.
        
        :param widgetType: the type of the widget used to display the data
        
        :returns: this widget object
        """
    @typing.overload
    def withWidget(self, widgetType: str) -> SuppliedRawValueWidget:
        """
        Sets the type of widget used to display the data. If not set, the default
        widget type will be used. This method should only be used to use a widget
        that does not come built into Shuffleboard (i.e. one that comes with a
        custom or third-party plugin). To use a widget that is built into
        Shuffleboard, use WithWidget(WidgetType) and BuiltInWidgets.
        
        :param widgetType: the type of the widget used to display the data
        
        :returns: this widget object
        """
class _SuppliedValueWidget_vector_string(_SuppliedValueComponent_vector_string):
    """
    Abstract superclass for widgets.
    
    This class is package-private to minimize API surface area.
    
    @tparam Derived the self type
    """
    @typing.overload
    def withWidget(self, widgetType: BuiltInWidgets) -> SuppliedStringListValueWidget:
        """
        Sets the type of widget used to display the data. If not set, the default
        widget type will be used.
        
        :param widgetType: the type of the widget used to display the data
        
        :returns: this widget object
                  @see BuiltInWidgets
        """
    @typing.overload
    def withWidget(self, widgetType: WidgetType) -> SuppliedStringListValueWidget:
        """
        Sets the type of widget used to display the data. If not set, the default
        widget type will be used.
        
        :param widgetType: the type of the widget used to display the data
        
        :returns: this widget object
        """
    @typing.overload
    def withWidget(self, widgetType: str) -> SuppliedStringListValueWidget:
        """
        Sets the type of widget used to display the data. If not set, the default
        widget type will be used. This method should only be used to use a widget
        that does not come built into Shuffleboard (i.e. one that comes with a
        custom or third-party plugin). To use a widget that is built into
        Shuffleboard, use WithWidget(WidgetType) and BuiltInWidgets.
        
        :param widgetType: the type of the widget used to display the data
        
        :returns: this widget object
        """
def _clearShuffleboardData() -> None:
    ...
def _getStringForWidgetType(type: BuiltInWidgets) -> str:
    ...
def shuffleboardEventImportanceName(importance: ShuffleboardEventImportance) -> str:
    """
    Returns name of the given enum.
    
    :returns: Name of the given enum.
    """
_sbd_cleanup: typing.Any  # value = <capsule object>
