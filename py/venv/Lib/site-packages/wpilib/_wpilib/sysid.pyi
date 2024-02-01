from __future__ import annotations
import typing
import wpimath.units
__all__ = ['State', 'SysIdRoutineLog']
class State:
    """
    Possible state of a SysId routine.
    
    Members:
    
      kQuasistaticForward : Quasistatic forward test.
    
      kQuasistaticReverse : Quasistatic reverse test.
    
      kDynamicForward : Dynamic forward test.
    
      kDynamicReverse : Dynamic reverse test.
    
      kNone : No test.
    """
    __members__: typing.ClassVar[dict[str, State]]  # value = {'kQuasistaticForward': <State.kQuasistaticForward: 0>, 'kQuasistaticReverse': <State.kQuasistaticReverse: 1>, 'kDynamicForward': <State.kDynamicForward: 2>, 'kDynamicReverse': <State.kDynamicReverse: 3>, 'kNone': <State.kNone: 4>}
    kDynamicForward: typing.ClassVar[State]  # value = <State.kDynamicForward: 2>
    kDynamicReverse: typing.ClassVar[State]  # value = <State.kDynamicReverse: 3>
    kNone: typing.ClassVar[State]  # value = <State.kNone: 4>
    kQuasistaticForward: typing.ClassVar[State]  # value = <State.kQuasistaticForward: 0>
    kQuasistaticReverse: typing.ClassVar[State]  # value = <State.kQuasistaticReverse: 1>
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
class SysIdRoutineLog:
    """
    Utility for logging data from a SysId test routine. Each complete routine
    (quasistatic and dynamic, forward and reverse) should have its own
    SysIdRoutineLog instance, with a unique log name.
    """
    class MotorLog:
        """
        Logs data from a single motor during a SysIdRoutine.
        """
        @typing.overload
        def acceleration(self, acceleration: wpimath.units.meters_per_second_squared) -> SysIdRoutineLog.MotorLog:
            """
            Log the linear acceleration of the motor.
            
            This is optional; SysId can perform an accurate fit without it.
            
            :param acceleration: The linear acceleration to record.
            
            :returns: The motor log (for call chaining).
            """
        @typing.overload
        def acceleration(self, acceleration: wpimath.units.turns_per_second_squared) -> SysIdRoutineLog.MotorLog:
            """
            Log the angular acceleration of the motor.
            
            This is optional; SysId can perform an accurate fit without it.
            
            :param acceleration: The angular acceleration to record.
            
            :returns: The motor log (for call chaining).
            """
        def current(self, current: wpimath.units.amperes) -> SysIdRoutineLog.MotorLog:
            """
            Log the current applied to the motor.
            
            This is optional; SysId can perform an accurate fit without it.
            
            :param current: The current to record.
            
            :returns: The motor log (for call chaining).
            """
        @typing.overload
        def position(self, position: wpimath.units.meters) -> SysIdRoutineLog.MotorLog:
            """
            Log the linear position of the motor.
            
            :param position: The linear position to record.
            
            :returns: The motor log (for call chaining).
            """
        @typing.overload
        def position(self, position: wpimath.units.turns) -> SysIdRoutineLog.MotorLog:
            """
            Log the angular position of the motor.
            
            :param position: The angular position to record.
            
            :returns: The motor log (for call chaining).
            """
        def value(self, name: str, value: float, unit: str) -> SysIdRoutineLog.MotorLog:
            """
            Log a generic data value from the motor.
            
            :param name:  The name of the data field being recorded.
            :param value: The numeric value of the data field.
            :param unit:  The unit string of the data field.
            
            :returns: The motor log (for call chaining).
            """
        @typing.overload
        def velocity(self, velocity: wpimath.units.meters_per_second) -> SysIdRoutineLog.MotorLog:
            """
            Log the linear velocity of the motor.
            
            :param velocity: The linear velocity to record.
            
            :returns: The motor log (for call chaining).
            """
        @typing.overload
        def velocity(self, velocity: wpimath.units.turns_per_second) -> SysIdRoutineLog.MotorLog:
            """
            Log the angular velocity of the motor.
            
            :param velocity: The angular velocity to record.
            
            :returns: The motor log (for call chaining).
            """
        def voltage(self, voltage: wpimath.units.volts) -> SysIdRoutineLog.MotorLog:
            """
            Log the voltage applied to the motor.
            
            :param voltage: The voltage to record.
            
            :returns: The motor log (for call chaining).
            """
    @staticmethod
    def stateEnumToString(state: State) -> str:
        ...
    def __init__(self, logName: str) -> None:
        """
        Create a new logging utility for a SysId test routine.
        
        :param logName: The name for the test routine in the log. Should be unique
                        between complete test routines (quasistatic and dynamic, forward and
                        reverse). The current state of this test (e.g. "quasistatic-forward")
                        will appear in WPILog under the "sysid-test-state-logName" entry.
        """
    def motor(self, motorName: str) -> SysIdRoutineLog.MotorLog:
        """
        Log data from a motor during a SysId routine.
        
        :param motorName: The name of the motor.
        
        :returns: Handle with chainable callbacks to log individual data fields.
        """
    def recordState(self, state: State) -> None:
        """
        Records the current state of the SysId test routine. Should be called once
        per iteration during tests with the type of the current test, and once upon
        test end with state `none`.
        
        :param state: The current state of the SysId test routine.
        """
