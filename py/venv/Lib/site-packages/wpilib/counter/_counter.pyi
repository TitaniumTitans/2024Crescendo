from __future__ import annotations
import typing
import wpilib._wpilib
import wpimath.units
import wpiutil._wpiutil
__all__ = ['EdgeConfiguration', 'ExternalDirectionCounter', 'Tachometer', 'UpDownCounter']
class EdgeConfiguration:
    """
    Edge configuration.
    
    Members:
    
      kNone : No edge configuration (neither rising nor falling).
    
      kRisingEdge : Rising edge configuration.
    
      kFallingEdge : Falling edge configuration.
    
      kBoth : Both rising and falling edges configuration.
    """
    __members__: typing.ClassVar[dict[str, EdgeConfiguration]]  # value = {'kNone': <EdgeConfiguration.kNone: 0>, 'kRisingEdge': <EdgeConfiguration.kRisingEdge: 1>, 'kFallingEdge': <EdgeConfiguration.kFallingEdge: 2>, 'kBoth': <EdgeConfiguration.kBoth: 3>}
    kBoth: typing.ClassVar[EdgeConfiguration]  # value = <EdgeConfiguration.kBoth: 3>
    kFallingEdge: typing.ClassVar[EdgeConfiguration]  # value = <EdgeConfiguration.kFallingEdge: 2>
    kNone: typing.ClassVar[EdgeConfiguration]  # value = <EdgeConfiguration.kNone: 0>
    kRisingEdge: typing.ClassVar[EdgeConfiguration]  # value = <EdgeConfiguration.kRisingEdge: 1>
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
class ExternalDirectionCounter(wpiutil._wpiutil.Sendable):
    """
    Counter using external direction.
    
    This counts on an edge from one digital input and the whether it counts
    up or down based on the state of a second digital input.
    """
    @typing.overload
    def __init__(self, countSource: wpilib._wpilib.DigitalSource, directionSource: wpilib._wpilib.DigitalSource) -> None:
        """
        Constructs a new ExternalDirectionCounter.
        
        :param countSource:     The source for counting.
        :param directionSource: The source for selecting count direction.
        """
    @typing.overload
    def __init__(self, countSource: wpilib._wpilib.DigitalSource, directionSource: wpilib._wpilib.DigitalSource) -> None:
        """
        Constructs a new ExternalDirectionCounter.
        
        :param countSource:     The source for counting.
        :param directionSource: The source for selecting count direction.
        """
    def _initSendable(self, builder: wpiutil._wpiutil.SendableBuilder) -> None:
        ...
    def getCount(self) -> int:
        """
        Gets the current count.
        
        :returns: The current count.
        """
    def reset(self) -> None:
        """
        Resets the current count.
        """
    def setEdgeConfiguration(self, configuration: EdgeConfiguration) -> None:
        """
        Sets the edge configuration for counting.
        
        :param configuration: The counting edge configuration.
        """
    def setReverseDirection(self, reverseDirection: bool) -> None:
        """
        Sets to reverse the counter direction.
        
        :param reverseDirection: True to reverse counting direction.
        """
class Tachometer(wpiutil._wpiutil.Sendable):
    """
    Tachometer for getting rotational speed from a device.
    
    The Tachometer class measures the time between digital pulses to
    determine the rotation speed of a mechanism. Examples of devices that could
    be used with the tachometer class are a hall effect sensor, break beam
    sensor, or optical sensor detecting tape on a shooter wheel. Unlike
    encoders, this class only needs a single digital input.
    """
    @typing.overload
    def __init__(self, source: wpilib._wpilib.DigitalSource) -> None:
        """
        Constructs a new tachometer.
        
        :param source: The source.
        """
    @typing.overload
    def __init__(self, source: wpilib._wpilib.DigitalSource) -> None:
        """
        Constructs a new tachometer.
        
        :param source: The source.
        """
    def _initSendable(self, builder: wpiutil._wpiutil.SendableBuilder) -> None:
        ...
    def getEdgesPerRevolution(self) -> int:
        """
        Gets the number of edges per revolution.
        
        :returns: Edges per revolution.
        """
    def getFrequency(self) -> wpimath.units.hertz:
        """
        Gets the tachometer frequency.
        
        :returns: Current frequency.
        """
    def getPeriod(self) -> wpimath.units.seconds:
        """
        Gets the tachometer period.
        
        :returns: Current period.
        """
    def getRevolutionsPerMinute(self) -> wpimath.units.revolutions_per_minute:
        """
        Gets the current tachometer revolutions per minute.
        
        SetEdgesPerRevolution must be set with a non 0 value for this to work.
        
        :returns: Current RPM.
        """
    def getRevolutionsPerSecond(self) -> wpimath.units.turns_per_second:
        """
        Gets the current tachometer revolutions per second.
        
        SetEdgesPerRevolution must be set with a non 0 value for this to work.
        
        :returns: Current RPS.
        """
    def getSamplesToAverage(self) -> int:
        """
        Gets the number of sample to average.
        
        :returns: Samples to average.
        """
    def getStopped(self) -> bool:
        """
        Gets if the tachometer is stopped.
        
        :returns: True if the tachometer is stopped.
        """
    def setEdgesPerRevolution(self, edges: int) -> None:
        """
        Sets the number of edges per revolution.
        
        :param edges: Edges per revolution.
        """
    def setMaxPeriod(self, maxPeriod: wpimath.units.seconds) -> None:
        """
        Sets the maximum period before the tachometer is considered stopped.
        
        :param maxPeriod: The max period.
        """
    def setSamplesToAverage(self, samples: int) -> None:
        """
        Sets the number of samples to average.
        
        :param samples: Samples to average.
        """
    def setUpdateWhenEmpty(self, updateWhenEmpty: bool) -> None:
        """
        Sets if to update when empty.
        
        :param updateWhenEmpty: True to update when empty.
        """
class UpDownCounter(wpiutil._wpiutil.Sendable):
    """
    Up Down Counter.
    
    This class can count edges on a single digital input or count up based on an
    edge from one digital input and down on an edge from another digital input.
    """
    @typing.overload
    def __init__(self, upSource: wpilib._wpilib.DigitalSource, downSource: wpilib._wpilib.DigitalSource) -> None:
        """
        Constructs a new UpDown Counter.
        
        :param upSource:   The up count source (can be null).
        :param downSource: The down count source (can be null).
        """
    @typing.overload
    def __init__(self, upSource: wpilib._wpilib.DigitalSource, downSource: wpilib._wpilib.DigitalSource) -> None:
        """
        Constructs a new UpDown Counter.
        
        :param upSource:   The up count source (can be null).
        :param downSource: The down count source (can be null).
        """
    def _initSendable(self, builder: wpiutil._wpiutil.SendableBuilder) -> None:
        ...
    def getCount(self) -> int:
        """
        Gets the current count.
        
        :returns: The current count.
        """
    def reset(self) -> None:
        """
        Resets the current count.
        """
    def setDownEdgeConfiguration(self, configuration: EdgeConfiguration) -> None:
        """
        Sets the configuration for the down source.
        
        :param configuration: The down source configuration.
        """
    def setReverseDirection(self, reverseDirection: bool) -> None:
        """
        Sets to revert the counter direction.
        
        :param reverseDirection: True to reverse counting direction.
        """
    def setUpEdgeConfiguration(self, configuration: EdgeConfiguration) -> None:
        """
        Sets the configuration for the up source.
        
        :param configuration: The up source configuration.
        """
