from __future__ import annotations
import typing
import wpimath.units
__all__ = ['Debouncer', 'LinearFilter', 'MedianFilter', 'SlewRateLimiter']
class Debouncer:
    """
    A simple debounce filter for boolean streams.  Requires that the boolean
    change value from baseline for a specified period of time before the filtered
    value changes.
    """
    class DebounceType:
        """
        Type of debouncing to perform.
        
        Members:
        
          kRising : Rising edge.
        
          kFalling : Falling edge.
        
          kBoth : Both rising and falling edges.
        """
        __members__: typing.ClassVar[dict[str, Debouncer.DebounceType]]  # value = {'kRising': <DebounceType.kRising: 0>, 'kFalling': <DebounceType.kFalling: 1>, 'kBoth': <DebounceType.kBoth: 2>}
        kBoth: typing.ClassVar[Debouncer.DebounceType]  # value = <DebounceType.kBoth: 2>
        kFalling: typing.ClassVar[Debouncer.DebounceType]  # value = <DebounceType.kFalling: 1>
        kRising: typing.ClassVar[Debouncer.DebounceType]  # value = <DebounceType.kRising: 0>
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
    def __init__(self, debounceTime: wpimath.units.seconds, type: Debouncer.DebounceType = ...) -> None:
        """
        Creates a new Debouncer.
        
        :param debounceTime: The number of seconds the value must change from
                             baseline for the filtered value to change.
        :param type:         Which type of state change the debouncing will be
                             performed on.
        """
    def calculate(self, input: bool) -> bool:
        """
        Applies the debouncer to the input stream.
        
        :param input: The current value of the input stream.
        
        :returns: The debounced value of the input stream.
        """
class LinearFilter:
    """
    This class implements a linear, digital filter. All types of FIR and IIR
    filters are supported. Static factory methods are provided to create commonly
    used types of filters.
    
    Filters are of the form:
    y[n] = (b0 x[n] + b1 x[n-1] + … + bP x[n-P]) -
    (a0 y[n-1] + a2 y[n-2] + … + aQ y[n-Q])
    
    Where:
    y[n] is the output at time "n"
    x[n] is the input at time "n"
    y[n-1] is the output from the LAST time step ("n-1")
    x[n-1] is the input from the LAST time step ("n-1")
    b0 … bP are the "feedforward" (FIR) gains
    a0 … aQ are the "feedback" (IIR) gains
    IMPORTANT! Note the "-" sign in front of the feedback term! This is a common
    convention in signal processing.
    
    What can linear filters do? Basically, they can filter, or diminish, the
    effects of undesirable input frequencies. High frequencies, or rapid changes,
    can be indicative of sensor noise or be otherwise undesirable. A "low pass"
    filter smooths out the signal, reducing the impact of these high frequency
    components.  Likewise, a "high pass" filter gets rid of slow-moving signal
    components, letting you detect large changes more easily.
    
    Example FRC applications of filters:
    - Getting rid of noise from an analog sensor input (note: the roboRIO's FPGA
    can do this faster in hardware)
    - Smoothing out joystick input to prevent the wheels from slipping or the
    robot from tipping
    - Smoothing motor commands so that unnecessary strain isn't put on
    electrical or mechanical components
    - If you use clever gains, you can make a PID controller out of this class!
    
    For more on filters, we highly recommend the following articles:
    https://en.wikipedia.org/wiki/Linear_filter
    https://en.wikipedia.org/wiki/Iir_filter
    https://en.wikipedia.org/wiki/Fir_filter
    
    Note 1: Calculate() should be called by the user on a known, regular period.
    You can use a Notifier for this or do it "inline" with code in a
    periodic function.
    
    Note 2: For ALL filters, gains are necessarily a function of frequency. If
    you make a filter that works well for you at, say, 100Hz, you will most
    definitely need to adjust the gains if you then want to run it at 200Hz!
    Combining this with Note 1 - the impetus is on YOU as a developer to make
    sure Calculate() gets called at the desired, constant frequency!
    """
    @staticmethod
    def highPass(timeConstant: float, period: wpimath.units.seconds) -> LinearFilter:
        """
        Creates a first-order high-pass filter of the form:
        y[n] = gain x[n] + (-gain) x[n-1] + gain y[n-1]
        where gain = e:sup:`-dt / T`, T is the time constant in seconds
        
        Note: T = 1 / (2 pi f) where f is the cutoff frequency in Hz, the frequency
        below which the input starts to attenuate.
        
        This filter is stable for time constants greater than zero.
        
        :param timeConstant: The discrete-time time constant in seconds.
        :param period:       The period in seconds between samples taken by the
                             user.
        """
    @staticmethod
    def movingAverage(taps: int) -> LinearFilter:
        """
        Creates a K-tap FIR moving average filter of the form:
        y[n] = 1/k (x[k] + x[k-1] + … + x[0])
        
        This filter is always stable.
        
        :param taps: The number of samples to average over. Higher = smoother but
                     slower
                     @throws std::runtime_error if number of taps is less than 1.
        """
    @staticmethod
    def singlePoleIIR(timeConstant: float, period: wpimath.units.seconds) -> LinearFilter:
        """
        Creates a one-pole IIR low-pass filter of the form:
        y[n] = (1 - gain) x[n] + gain y[n-1]
        where gain = e:sup:`-dt / T`, T is the time constant in seconds
        
        Note: T = 1 / (2 pi f) where f is the cutoff frequency in Hz, the frequency
        above which the input starts to attenuate.
        
        This filter is stable for time constants greater than zero.
        
        :param timeConstant: The discrete-time time constant in seconds.
        :param period:       The period in seconds between samples taken by the
                             user.
        """
    def __init__(self, ffGains: list[float], fbGains: list[float]) -> None:
        """
        Create a linear FIR or IIR filter.
        
        :param ffGains: The "feedforward" or FIR gains.
        :param fbGains: The "feedback" or IIR gains.
        """
    def calculate(self, input: float) -> float:
        """
        Calculates the next value of the filter.
        
        :param input: Current input value.
        
        :returns: The filtered value at this step
        """
    @typing.overload
    def reset(self) -> None:
        """
        Reset the filter state.
        """
    @typing.overload
    def reset(self, inputBuffer: list[float], outputBuffer: list[float]) -> None:
        """
        Resets the filter state, initializing internal buffers to the provided
        values.
        
        These are the expected lengths of the buffers, depending on what type of
        linear filter used:
        
        <table>
        <tr>
        <th>Type</th>
        <th>Input Buffer Size</th>
        <th>Output Buffer Size</th>
        </tr>
        <tr>
        <td>Unspecified</td>
        <td>size of ``ffGains``</td>
        <td>size of ``fbGains``</td>
        </tr>
        <tr>
        <td>Single Pole IIR</td>
        <td>1</td>
        <td>1</td>
        </tr>
        <tr>
        <td>High-Pass</td>
        <td>2</td>
        <td>1</td>
        </tr>
        <tr>
        <td>Moving Average</td>
        <td>``taps``</td>
        <td>0</td>
        </tr>
        <tr>
        <td>Finite Difference</td>
        <td>size of ``stencil``</td>
        <td>0</td>
        </tr>
        <tr>
        <td>Backward Finite Difference</td>
        <td>``Samples``</td>
        <td>0</td>
        </tr>
        </table>
        
        :param inputBuffer:  Values to initialize input buffer.
        :param outputBuffer: Values to initialize output buffer.
                             @throws std::runtime_error if size of inputBuffer or outputBuffer does not
                             match the size of ffGains and fbGains provided in the constructor.
        """
class MedianFilter:
    """
    A class that implements a moving-window median filter.  Useful for reducing
    measurement noise, especially with processes that generate occasional,
    extreme outliers (such as values from vision processing, LIDAR, or ultrasonic
    sensors).
    """
    def __init__(self, size: int) -> None:
        """
        Creates a new MedianFilter.
        
        :param size: The number of samples in the moving window.
        """
    def calculate(self, next: float) -> float:
        """
        Calculates the moving-window median for the next value of the input stream.
        
        :param next: The next input value.
        
        :returns: The median of the moving window, updated to include the next value.
        """
    def reset(self) -> None:
        """
        Resets the filter, clearing the window of all elements.
        """
class SlewRateLimiter:
    """
    A class that limits the rate of change of an input value.  Useful for
    implementing voltage, setpoint, and/or output ramps.  A slew-rate limit
    is most appropriate when the quantity being controlled is a velocity or
    a voltage; when controlling a position, consider using a TrapezoidProfile
    instead.
    
    @see TrapezoidProfile
    """
    @typing.overload
    def __init__(self, positiveRateLimit: wpimath.units.units_per_second, negativeRateLimit: wpimath.units.units_per_second, initialValue: float = 0.0) -> None:
        """
        Creates a new SlewRateLimiter with the given positive and negative rate
        limits and initial value.
        
        :param positiveRateLimit: The rate-of-change limit in the positive
                                  direction, in units per second. This is expected
                                  to be positive.
        :param negativeRateLimit: The rate-of-change limit in the negative
                                  direction, in units per second. This is expected
                                  to be negative.
        :param initialValue:      The initial value of the input.
        """
    @typing.overload
    def __init__(self, rateLimit: wpimath.units.units_per_second) -> None:
        """
        Creates a new SlewRateLimiter with the given positive rate limit and
        negative rate limit of -rateLimit.
        
        :param rateLimit: The rate-of-change limit.
        """
    def calculate(self, input: float) -> float:
        """
        Filters the input to limit its slew rate.
        
        :param input: The input value whose slew rate is to be limited.
        
        :returns: The filtered value, which will not change faster than the slew
                  rate.
        """
    def reset(self, value: float) -> None:
        """
        Resets the slew rate limiter to the specified value; ignores the rate limit
        when doing so.
        
        :param value: The value to reset to.
        """
