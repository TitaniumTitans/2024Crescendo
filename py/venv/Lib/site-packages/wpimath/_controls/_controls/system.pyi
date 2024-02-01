from __future__ import annotations
import numpy
import typing
import wpimath._controls._controls.controller
import wpimath._controls._controls.estimator
import wpimath.units
__all__ = ['LinearSystemLoop_1_1_1', 'LinearSystemLoop_2_1_1', 'LinearSystemLoop_2_2_2', 'LinearSystemLoop_3_2_3', 'LinearSystem_1_1_1', 'LinearSystem_1_1_2', 'LinearSystem_1_1_3', 'LinearSystem_2_1_1', 'LinearSystem_2_1_2', 'LinearSystem_2_1_3', 'LinearSystem_2_2_1', 'LinearSystem_2_2_2', 'LinearSystem_2_2_3', 'LinearSystem_3_2_1', 'LinearSystem_3_2_2', 'LinearSystem_3_2_3']
class LinearSystemLoop_1_1_1:
    """
    Combines a controller, feedforward, and observer for controlling a mechanism
    with full state feedback.
    
    For everything in this file, "inputs" and "outputs" are defined from the
    perspective of the plant. This means U is an input and Y is an output
    (because you give the plant U (powers) and it gives you back a Y (sensor
    values). This is the opposite of what they mean from the perspective of the
    controller (U is an output because that's what goes to the motors and Y is an
    input because that's what comes back from the sensors).
    
    For more on the underlying math, read
    https://file.tavsys.net/control/controls-engineering-in-frc.pdf.
    
    @tparam States Number of states.
    @tparam Inputs Number of inputs.
    @tparam Outputs Number of outputs.
    """
    @typing.overload
    def U(self) -> numpy.ndarray[numpy.float64[1, 1]]:
        """
        Returns the controller's calculated control input u.
        """
    @typing.overload
    def U(self, i: int) -> float:
        """
        Returns an element of the controller's calculated control input u.
        
        :param i: Row of u.
        """
    @typing.overload
    def __init__(self, plant: LinearSystem_1_1_1, controller: wpimath._controls._controls.controller.LinearQuadraticRegulator_1_1, observer: wpimath._controls._controls.estimator.KalmanFilter_1_1_1, maxVoltage: wpimath.units.volts, dt: wpimath.units.seconds) -> None:
        """
        Constructs a state-space loop with the given plant, controller, and
        observer. By default, the initial reference is all zeros. Users should
        call reset with the initial system state before enabling the loop. This
        constructor assumes that the input(s) to this system are voltage.
        
        :param plant:      State-space plant.
        :param controller: State-space controller.
        :param observer:   State-space observer.
        :param maxVoltage: The maximum voltage that can be applied. Commonly 12.
        :param dt:         The nominal timestep.
        """
    @typing.overload
    def __init__(self, plant: LinearSystem_1_1_1, controller: wpimath._controls._controls.controller.LinearQuadraticRegulator_1_1, observer: wpimath._controls._controls.estimator.KalmanFilter_1_1_1, clampFunction: typing.Callable[[numpy.ndarray[numpy.float64[1, 1]]], numpy.ndarray[numpy.float64[1, 1]]], dt: wpimath.units.seconds) -> None:
        """
        Constructs a state-space loop with the given plant, controller, and
        observer. By default, the initial reference is all zeros. Users should
        call reset with the initial system state before enabling the loop. This
        constructor assumes that the input(s) to this system are voltage.
        
        :param plant:         State-space plant.
        :param controller:    State-space controller.
        :param observer:      State-space observer.
        :param clampFunction: The function used to clamp the input vector.
        :param dt:            The nominal timestep.
        """
    @typing.overload
    def __init__(self, controller: wpimath._controls._controls.controller.LinearQuadraticRegulator_1_1, feedforward: wpimath._controls._controls.controller.LinearPlantInversionFeedforward_1_1, observer: wpimath._controls._controls.estimator.KalmanFilter_1_1_1, maxVoltage: wpimath.units.volts) -> None:
        """
        Constructs a state-space loop with the given controller, feedforward and
        observer. By default, the initial reference is all zeros. Users should
        call reset with the initial system state.
        
        :param controller:  State-space controller.
        :param feedforward: Plant inversion feedforward.
        :param observer:    State-space observer.
        :param maxVoltage:  The maximum voltage that can be applied. Assumes
                            that the inputs are voltages.
        """
    @typing.overload
    def __init__(self, controller: wpimath._controls._controls.controller.LinearQuadraticRegulator_1_1, feedforward: wpimath._controls._controls.controller.LinearPlantInversionFeedforward_1_1, observer: wpimath._controls._controls.estimator.KalmanFilter_1_1_1, clampFunction: typing.Callable[[numpy.ndarray[numpy.float64[1, 1]]], numpy.ndarray[numpy.float64[1, 1]]]) -> None:
        """
        Constructs a state-space loop with the given controller, feedforward,
        observer and clamp function. By default, the initial reference is all
        zeros. Users should call reset with the initial system state.
        
        :param controller:    State-space controller.
        :param feedforward:   Plant-inversion feedforward.
        :param observer:      State-space observer.
        :param clampFunction: The function used to clamp the input vector.
        """
    def clampInput(self, u: numpy.ndarray[numpy.float64[1, 1]]) -> numpy.ndarray[numpy.float64[1, 1]]:
        """
        Clamps input vector between system's minimum and maximum allowable input.
        
        :param u: Input vector to clamp.
        
        :returns: Clamped input vector.
        """
    def correct(self, y: numpy.ndarray[numpy.float64[1, 1]]) -> None:
        """
        Correct the state estimate x-hat using the measurements in y.
        
        :param y: Measurement vector.
        """
    def error(self) -> numpy.ndarray[numpy.float64[1, 1]]:
        """
        Returns difference between reference r and current state x-hat.
        """
    @typing.overload
    def nextR(self) -> numpy.ndarray[numpy.float64[1, 1]]:
        """
        Returns the controller's next reference r.
        """
    @typing.overload
    def nextR(self, i: int) -> float:
        """
        Returns an element of the controller's next reference r.
        
        :param i: Row of r.
        """
    def predict(self, dt: wpimath.units.seconds) -> None:
        """
        Sets new controller output, projects model forward, and runs observer
        prediction.
        
        After calling this, the user should send the elements of u to the
        actuators.
        
        :param dt: Timestep for model update.
        """
    def reset(self, initialState: numpy.ndarray[numpy.float64[1, 1]]) -> None:
        """
        Zeroes reference r and controller output u. The previous reference
        of the PlantInversionFeedforward and the initial state estimate of
        the KalmanFilter are set to the initial state provided.
        
        :param initialState: The initial state.
        """
    def setNextR(self, nextR: numpy.ndarray[numpy.float64[1, 1]]) -> None:
        """
        Set the next reference r.
        
        :param nextR: Next reference.
        """
    @typing.overload
    def setXhat(self, xHat: numpy.ndarray[numpy.float64[1, 1]]) -> None:
        """
        Set the initial state estimate x-hat.
        
        :param xHat: The initial state estimate x-hat.
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
        Returns the observer's state estimate x-hat.
        """
    @typing.overload
    def xhat(self, i: int) -> float:
        """
        Returns an element of the observer's state estimate x-hat.
        
        :param i: Row of x-hat.
        """
class LinearSystemLoop_2_1_1:
    """
    Combines a controller, feedforward, and observer for controlling a mechanism
    with full state feedback.
    
    For everything in this file, "inputs" and "outputs" are defined from the
    perspective of the plant. This means U is an input and Y is an output
    (because you give the plant U (powers) and it gives you back a Y (sensor
    values). This is the opposite of what they mean from the perspective of the
    controller (U is an output because that's what goes to the motors and Y is an
    input because that's what comes back from the sensors).
    
    For more on the underlying math, read
    https://file.tavsys.net/control/controls-engineering-in-frc.pdf.
    
    @tparam States Number of states.
    @tparam Inputs Number of inputs.
    @tparam Outputs Number of outputs.
    """
    @typing.overload
    def U(self) -> numpy.ndarray[numpy.float64[1, 1]]:
        """
        Returns the controller's calculated control input u.
        """
    @typing.overload
    def U(self, i: int) -> float:
        """
        Returns an element of the controller's calculated control input u.
        
        :param i: Row of u.
        """
    @typing.overload
    def __init__(self, plant: LinearSystem_2_1_1, controller: wpimath._controls._controls.controller.LinearQuadraticRegulator_2_1, observer: wpimath._controls._controls.estimator.KalmanFilter_2_1_1, maxVoltage: wpimath.units.volts, dt: wpimath.units.seconds) -> None:
        """
        Constructs a state-space loop with the given plant, controller, and
        observer. By default, the initial reference is all zeros. Users should
        call reset with the initial system state before enabling the loop. This
        constructor assumes that the input(s) to this system are voltage.
        
        :param plant:      State-space plant.
        :param controller: State-space controller.
        :param observer:   State-space observer.
        :param maxVoltage: The maximum voltage that can be applied. Commonly 12.
        :param dt:         The nominal timestep.
        """
    @typing.overload
    def __init__(self, plant: LinearSystem_2_1_1, controller: wpimath._controls._controls.controller.LinearQuadraticRegulator_2_1, observer: wpimath._controls._controls.estimator.KalmanFilter_2_1_1, clampFunction: typing.Callable[[numpy.ndarray[numpy.float64[1, 1]]], numpy.ndarray[numpy.float64[1, 1]]], dt: wpimath.units.seconds) -> None:
        """
        Constructs a state-space loop with the given plant, controller, and
        observer. By default, the initial reference is all zeros. Users should
        call reset with the initial system state before enabling the loop. This
        constructor assumes that the input(s) to this system are voltage.
        
        :param plant:         State-space plant.
        :param controller:    State-space controller.
        :param observer:      State-space observer.
        :param clampFunction: The function used to clamp the input vector.
        :param dt:            The nominal timestep.
        """
    @typing.overload
    def __init__(self, controller: wpimath._controls._controls.controller.LinearQuadraticRegulator_2_1, feedforward: wpimath._controls._controls.controller.LinearPlantInversionFeedforward_2_1, observer: wpimath._controls._controls.estimator.KalmanFilter_2_1_1, maxVoltage: wpimath.units.volts) -> None:
        """
        Constructs a state-space loop with the given controller, feedforward and
        observer. By default, the initial reference is all zeros. Users should
        call reset with the initial system state.
        
        :param controller:  State-space controller.
        :param feedforward: Plant inversion feedforward.
        :param observer:    State-space observer.
        :param maxVoltage:  The maximum voltage that can be applied. Assumes
                            that the inputs are voltages.
        """
    @typing.overload
    def __init__(self, controller: wpimath._controls._controls.controller.LinearQuadraticRegulator_2_1, feedforward: wpimath._controls._controls.controller.LinearPlantInversionFeedforward_2_1, observer: wpimath._controls._controls.estimator.KalmanFilter_2_1_1, clampFunction: typing.Callable[[numpy.ndarray[numpy.float64[1, 1]]], numpy.ndarray[numpy.float64[1, 1]]]) -> None:
        """
        Constructs a state-space loop with the given controller, feedforward,
        observer and clamp function. By default, the initial reference is all
        zeros. Users should call reset with the initial system state.
        
        :param controller:    State-space controller.
        :param feedforward:   Plant-inversion feedforward.
        :param observer:      State-space observer.
        :param clampFunction: The function used to clamp the input vector.
        """
    def clampInput(self, u: numpy.ndarray[numpy.float64[1, 1]]) -> numpy.ndarray[numpy.float64[1, 1]]:
        """
        Clamps input vector between system's minimum and maximum allowable input.
        
        :param u: Input vector to clamp.
        
        :returns: Clamped input vector.
        """
    def correct(self, y: numpy.ndarray[numpy.float64[1, 1]]) -> None:
        """
        Correct the state estimate x-hat using the measurements in y.
        
        :param y: Measurement vector.
        """
    def error(self) -> numpy.ndarray[numpy.float64[2, 1]]:
        """
        Returns difference between reference r and current state x-hat.
        """
    @typing.overload
    def nextR(self) -> numpy.ndarray[numpy.float64[2, 1]]:
        """
        Returns the controller's next reference r.
        """
    @typing.overload
    def nextR(self, i: int) -> float:
        """
        Returns an element of the controller's next reference r.
        
        :param i: Row of r.
        """
    def predict(self, dt: wpimath.units.seconds) -> None:
        """
        Sets new controller output, projects model forward, and runs observer
        prediction.
        
        After calling this, the user should send the elements of u to the
        actuators.
        
        :param dt: Timestep for model update.
        """
    def reset(self, initialState: numpy.ndarray[numpy.float64[2, 1]]) -> None:
        """
        Zeroes reference r and controller output u. The previous reference
        of the PlantInversionFeedforward and the initial state estimate of
        the KalmanFilter are set to the initial state provided.
        
        :param initialState: The initial state.
        """
    def setNextR(self, nextR: numpy.ndarray[numpy.float64[2, 1]]) -> None:
        """
        Set the next reference r.
        
        :param nextR: Next reference.
        """
    @typing.overload
    def setXhat(self, xHat: numpy.ndarray[numpy.float64[2, 1]]) -> None:
        """
        Set the initial state estimate x-hat.
        
        :param xHat: The initial state estimate x-hat.
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
        Returns the observer's state estimate x-hat.
        """
    @typing.overload
    def xhat(self, i: int) -> float:
        """
        Returns an element of the observer's state estimate x-hat.
        
        :param i: Row of x-hat.
        """
class LinearSystemLoop_2_2_2:
    """
    Combines a controller, feedforward, and observer for controlling a mechanism
    with full state feedback.
    
    For everything in this file, "inputs" and "outputs" are defined from the
    perspective of the plant. This means U is an input and Y is an output
    (because you give the plant U (powers) and it gives you back a Y (sensor
    values). This is the opposite of what they mean from the perspective of the
    controller (U is an output because that's what goes to the motors and Y is an
    input because that's what comes back from the sensors).
    
    For more on the underlying math, read
    https://file.tavsys.net/control/controls-engineering-in-frc.pdf.
    
    @tparam States Number of states.
    @tparam Inputs Number of inputs.
    @tparam Outputs Number of outputs.
    """
    @typing.overload
    def U(self) -> numpy.ndarray[numpy.float64[2, 1]]:
        """
        Returns the controller's calculated control input u.
        """
    @typing.overload
    def U(self, i: int) -> float:
        """
        Returns an element of the controller's calculated control input u.
        
        :param i: Row of u.
        """
    @typing.overload
    def __init__(self, plant: LinearSystem_2_2_2, controller: wpimath._controls._controls.controller.LinearQuadraticRegulator_2_2, observer: wpimath._controls._controls.estimator.KalmanFilter_2_2_2, maxVoltage: wpimath.units.volts, dt: wpimath.units.seconds) -> None:
        """
        Constructs a state-space loop with the given plant, controller, and
        observer. By default, the initial reference is all zeros. Users should
        call reset with the initial system state before enabling the loop. This
        constructor assumes that the input(s) to this system are voltage.
        
        :param plant:      State-space plant.
        :param controller: State-space controller.
        :param observer:   State-space observer.
        :param maxVoltage: The maximum voltage that can be applied. Commonly 12.
        :param dt:         The nominal timestep.
        """
    @typing.overload
    def __init__(self, plant: LinearSystem_2_2_2, controller: wpimath._controls._controls.controller.LinearQuadraticRegulator_2_2, observer: wpimath._controls._controls.estimator.KalmanFilter_2_2_2, clampFunction: typing.Callable[[numpy.ndarray[numpy.float64[2, 1]]], numpy.ndarray[numpy.float64[2, 1]]], dt: wpimath.units.seconds) -> None:
        """
        Constructs a state-space loop with the given plant, controller, and
        observer. By default, the initial reference is all zeros. Users should
        call reset with the initial system state before enabling the loop. This
        constructor assumes that the input(s) to this system are voltage.
        
        :param plant:         State-space plant.
        :param controller:    State-space controller.
        :param observer:      State-space observer.
        :param clampFunction: The function used to clamp the input vector.
        :param dt:            The nominal timestep.
        """
    @typing.overload
    def __init__(self, controller: wpimath._controls._controls.controller.LinearQuadraticRegulator_2_2, feedforward: wpimath._controls._controls.controller.LinearPlantInversionFeedforward_2_2, observer: wpimath._controls._controls.estimator.KalmanFilter_2_2_2, maxVoltage: wpimath.units.volts) -> None:
        """
        Constructs a state-space loop with the given controller, feedforward and
        observer. By default, the initial reference is all zeros. Users should
        call reset with the initial system state.
        
        :param controller:  State-space controller.
        :param feedforward: Plant inversion feedforward.
        :param observer:    State-space observer.
        :param maxVoltage:  The maximum voltage that can be applied. Assumes
                            that the inputs are voltages.
        """
    @typing.overload
    def __init__(self, controller: wpimath._controls._controls.controller.LinearQuadraticRegulator_2_2, feedforward: wpimath._controls._controls.controller.LinearPlantInversionFeedforward_2_2, observer: wpimath._controls._controls.estimator.KalmanFilter_2_2_2, clampFunction: typing.Callable[[numpy.ndarray[numpy.float64[2, 1]]], numpy.ndarray[numpy.float64[2, 1]]]) -> None:
        """
        Constructs a state-space loop with the given controller, feedforward,
        observer and clamp function. By default, the initial reference is all
        zeros. Users should call reset with the initial system state.
        
        :param controller:    State-space controller.
        :param feedforward:   Plant-inversion feedforward.
        :param observer:      State-space observer.
        :param clampFunction: The function used to clamp the input vector.
        """
    def clampInput(self, u: numpy.ndarray[numpy.float64[2, 1]]) -> numpy.ndarray[numpy.float64[2, 1]]:
        """
        Clamps input vector between system's minimum and maximum allowable input.
        
        :param u: Input vector to clamp.
        
        :returns: Clamped input vector.
        """
    def correct(self, y: numpy.ndarray[numpy.float64[2, 1]]) -> None:
        """
        Correct the state estimate x-hat using the measurements in y.
        
        :param y: Measurement vector.
        """
    def error(self) -> numpy.ndarray[numpy.float64[2, 1]]:
        """
        Returns difference between reference r and current state x-hat.
        """
    @typing.overload
    def nextR(self) -> numpy.ndarray[numpy.float64[2, 1]]:
        """
        Returns the controller's next reference r.
        """
    @typing.overload
    def nextR(self, i: int) -> float:
        """
        Returns an element of the controller's next reference r.
        
        :param i: Row of r.
        """
    def predict(self, dt: wpimath.units.seconds) -> None:
        """
        Sets new controller output, projects model forward, and runs observer
        prediction.
        
        After calling this, the user should send the elements of u to the
        actuators.
        
        :param dt: Timestep for model update.
        """
    def reset(self, initialState: numpy.ndarray[numpy.float64[2, 1]]) -> None:
        """
        Zeroes reference r and controller output u. The previous reference
        of the PlantInversionFeedforward and the initial state estimate of
        the KalmanFilter are set to the initial state provided.
        
        :param initialState: The initial state.
        """
    def setNextR(self, nextR: numpy.ndarray[numpy.float64[2, 1]]) -> None:
        """
        Set the next reference r.
        
        :param nextR: Next reference.
        """
    @typing.overload
    def setXhat(self, xHat: numpy.ndarray[numpy.float64[2, 1]]) -> None:
        """
        Set the initial state estimate x-hat.
        
        :param xHat: The initial state estimate x-hat.
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
        Returns the observer's state estimate x-hat.
        """
    @typing.overload
    def xhat(self, i: int) -> float:
        """
        Returns an element of the observer's state estimate x-hat.
        
        :param i: Row of x-hat.
        """
class LinearSystemLoop_3_2_3:
    """
    Combines a controller, feedforward, and observer for controlling a mechanism
    with full state feedback.
    
    For everything in this file, "inputs" and "outputs" are defined from the
    perspective of the plant. This means U is an input and Y is an output
    (because you give the plant U (powers) and it gives you back a Y (sensor
    values). This is the opposite of what they mean from the perspective of the
    controller (U is an output because that's what goes to the motors and Y is an
    input because that's what comes back from the sensors).
    
    For more on the underlying math, read
    https://file.tavsys.net/control/controls-engineering-in-frc.pdf.
    
    @tparam States Number of states.
    @tparam Inputs Number of inputs.
    @tparam Outputs Number of outputs.
    """
    @typing.overload
    def U(self) -> numpy.ndarray[numpy.float64[2, 1]]:
        """
        Returns the controller's calculated control input u.
        """
    @typing.overload
    def U(self, i: int) -> float:
        """
        Returns an element of the controller's calculated control input u.
        
        :param i: Row of u.
        """
    @typing.overload
    def __init__(self, plant: LinearSystem_3_2_3, controller: wpimath._controls._controls.controller.LinearQuadraticRegulator_3_2, observer: wpimath._controls._controls.estimator.KalmanFilter_3_2_3, maxVoltage: wpimath.units.volts, dt: wpimath.units.seconds) -> None:
        """
        Constructs a state-space loop with the given plant, controller, and
        observer. By default, the initial reference is all zeros. Users should
        call reset with the initial system state before enabling the loop. This
        constructor assumes that the input(s) to this system are voltage.
        
        :param plant:      State-space plant.
        :param controller: State-space controller.
        :param observer:   State-space observer.
        :param maxVoltage: The maximum voltage that can be applied. Commonly 12.
        :param dt:         The nominal timestep.
        """
    @typing.overload
    def __init__(self, plant: LinearSystem_3_2_3, controller: wpimath._controls._controls.controller.LinearQuadraticRegulator_3_2, observer: wpimath._controls._controls.estimator.KalmanFilter_3_2_3, clampFunction: typing.Callable[[numpy.ndarray[numpy.float64[2, 1]]], numpy.ndarray[numpy.float64[2, 1]]], dt: wpimath.units.seconds) -> None:
        """
        Constructs a state-space loop with the given plant, controller, and
        observer. By default, the initial reference is all zeros. Users should
        call reset with the initial system state before enabling the loop. This
        constructor assumes that the input(s) to this system are voltage.
        
        :param plant:         State-space plant.
        :param controller:    State-space controller.
        :param observer:      State-space observer.
        :param clampFunction: The function used to clamp the input vector.
        :param dt:            The nominal timestep.
        """
    @typing.overload
    def __init__(self, controller: wpimath._controls._controls.controller.LinearQuadraticRegulator_3_2, feedforward: wpimath._controls._controls.controller.LinearPlantInversionFeedforward_3_2, observer: wpimath._controls._controls.estimator.KalmanFilter_3_2_3, maxVoltage: wpimath.units.volts) -> None:
        """
        Constructs a state-space loop with the given controller, feedforward and
        observer. By default, the initial reference is all zeros. Users should
        call reset with the initial system state.
        
        :param controller:  State-space controller.
        :param feedforward: Plant inversion feedforward.
        :param observer:    State-space observer.
        :param maxVoltage:  The maximum voltage that can be applied. Assumes
                            that the inputs are voltages.
        """
    @typing.overload
    def __init__(self, controller: wpimath._controls._controls.controller.LinearQuadraticRegulator_3_2, feedforward: wpimath._controls._controls.controller.LinearPlantInversionFeedforward_3_2, observer: wpimath._controls._controls.estimator.KalmanFilter_3_2_3, clampFunction: typing.Callable[[numpy.ndarray[numpy.float64[2, 1]]], numpy.ndarray[numpy.float64[2, 1]]]) -> None:
        """
        Constructs a state-space loop with the given controller, feedforward,
        observer and clamp function. By default, the initial reference is all
        zeros. Users should call reset with the initial system state.
        
        :param controller:    State-space controller.
        :param feedforward:   Plant-inversion feedforward.
        :param observer:      State-space observer.
        :param clampFunction: The function used to clamp the input vector.
        """
    def clampInput(self, u: numpy.ndarray[numpy.float64[2, 1]]) -> numpy.ndarray[numpy.float64[2, 1]]:
        """
        Clamps input vector between system's minimum and maximum allowable input.
        
        :param u: Input vector to clamp.
        
        :returns: Clamped input vector.
        """
    def correct(self, y: numpy.ndarray[numpy.float64[3, 1]]) -> None:
        """
        Correct the state estimate x-hat using the measurements in y.
        
        :param y: Measurement vector.
        """
    def error(self) -> numpy.ndarray[numpy.float64[3, 1]]:
        """
        Returns difference between reference r and current state x-hat.
        """
    @typing.overload
    def nextR(self) -> numpy.ndarray[numpy.float64[3, 1]]:
        """
        Returns the controller's next reference r.
        """
    @typing.overload
    def nextR(self, i: int) -> float:
        """
        Returns an element of the controller's next reference r.
        
        :param i: Row of r.
        """
    def predict(self, dt: wpimath.units.seconds) -> None:
        """
        Sets new controller output, projects model forward, and runs observer
        prediction.
        
        After calling this, the user should send the elements of u to the
        actuators.
        
        :param dt: Timestep for model update.
        """
    def reset(self, initialState: numpy.ndarray[numpy.float64[3, 1]]) -> None:
        """
        Zeroes reference r and controller output u. The previous reference
        of the PlantInversionFeedforward and the initial state estimate of
        the KalmanFilter are set to the initial state provided.
        
        :param initialState: The initial state.
        """
    def setNextR(self, nextR: numpy.ndarray[numpy.float64[3, 1]]) -> None:
        """
        Set the next reference r.
        
        :param nextR: Next reference.
        """
    @typing.overload
    def setXhat(self, xHat: numpy.ndarray[numpy.float64[3, 1]]) -> None:
        """
        Set the initial state estimate x-hat.
        
        :param xHat: The initial state estimate x-hat.
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
        Returns the observer's state estimate x-hat.
        """
    @typing.overload
    def xhat(self, i: int) -> float:
        """
        Returns an element of the observer's state estimate x-hat.
        
        :param i: Row of x-hat.
        """
class LinearSystem_1_1_1:
    """
    A plant defined using state-space notation.
    
    A plant is a mathematical model of a system's dynamics.
    
    For more on the underlying math, read
    https://file.tavsys.net/control/controls-engineering-in-frc.pdf.
    
    @tparam States Number of states.
    @tparam Inputs Number of inputs.
    @tparam Outputs Number of outputs.
    """
    @typing.overload
    def A(self) -> numpy.ndarray[numpy.float64[1, 1]]:
        """
        Returns the system matrix A.
        """
    @typing.overload
    def A(self, i: int, j: int) -> float:
        """
        Returns an element of the system matrix A.
        
        :param i: Row of A.
        :param j: Column of A.
        """
    @typing.overload
    def B(self) -> numpy.ndarray[numpy.float64[1, 1]]:
        """
        Returns the input matrix B.
        """
    @typing.overload
    def B(self, i: int, j: int) -> float:
        """
        Returns an element of the input matrix B.
        
        :param i: Row of B.
        :param j: Column of B.
        """
    @typing.overload
    def C(self) -> numpy.ndarray[numpy.float64[1, 1]]:
        """
        Returns the output matrix C.
        """
    @typing.overload
    def C(self, i: int, j: int) -> float:
        """
        Returns an element of the output matrix C.
        
        :param i: Row of C.
        :param j: Column of C.
        """
    @typing.overload
    def D(self) -> numpy.ndarray[numpy.float64[1, 1]]:
        """
        Returns the feedthrough matrix D.
        """
    @typing.overload
    def D(self, i: int, j: int) -> float:
        """
        Returns an element of the feedthrough matrix D.
        
        :param i: Row of D.
        :param j: Column of D.
        """
    def __init__(self, A: numpy.ndarray[numpy.float64[1, 1]], B: numpy.ndarray[numpy.float64[1, 1]], C: numpy.ndarray[numpy.float64[1, 1]], D: numpy.ndarray[numpy.float64[1, 1]]) -> None:
        """
        Constructs a discrete plant with the given continuous system coefficients.
        
        :param A: System matrix.
        :param B: Input matrix.
        :param C: Output matrix.
        :param D: Feedthrough matrix.
                  @throws std::domain_error if any matrix element isn't finite.
        """
    def calculateX(self, x: numpy.ndarray[numpy.float64[1, 1]], clampedU: numpy.ndarray[numpy.float64[1, 1]], dt: wpimath.units.seconds) -> numpy.ndarray[numpy.float64[1, 1]]:
        """
        Computes the new x given the old x and the control input.
        
        This is used by state observers directly to run updates based on state
        estimate.
        
        :param x:        The current state.
        :param clampedU: The control input.
        :param dt:       Timestep for model update.
        """
    def calculateY(self, x: numpy.ndarray[numpy.float64[1, 1]], clampedU: numpy.ndarray[numpy.float64[1, 1]]) -> numpy.ndarray[numpy.float64[1, 1]]:
        """
        Computes the new y given the control input.
        
        This is used by state observers directly to run updates based on state
        estimate.
        
        :param x:        The current state.
        :param clampedU: The control input.
        """
class LinearSystem_1_1_2:
    """
    A plant defined using state-space notation.
    
    A plant is a mathematical model of a system's dynamics.
    
    For more on the underlying math, read
    https://file.tavsys.net/control/controls-engineering-in-frc.pdf.
    
    @tparam States Number of states.
    @tparam Inputs Number of inputs.
    @tparam Outputs Number of outputs.
    """
    @typing.overload
    def A(self) -> numpy.ndarray[numpy.float64[1, 1]]:
        """
        Returns the system matrix A.
        """
    @typing.overload
    def A(self, i: int, j: int) -> float:
        """
        Returns an element of the system matrix A.
        
        :param i: Row of A.
        :param j: Column of A.
        """
    @typing.overload
    def B(self) -> numpy.ndarray[numpy.float64[1, 1]]:
        """
        Returns the input matrix B.
        """
    @typing.overload
    def B(self, i: int, j: int) -> float:
        """
        Returns an element of the input matrix B.
        
        :param i: Row of B.
        :param j: Column of B.
        """
    @typing.overload
    def C(self) -> numpy.ndarray[numpy.float64[2, 1]]:
        """
        Returns the output matrix C.
        """
    @typing.overload
    def C(self, i: int, j: int) -> float:
        """
        Returns an element of the output matrix C.
        
        :param i: Row of C.
        :param j: Column of C.
        """
    @typing.overload
    def D(self) -> numpy.ndarray[numpy.float64[2, 1]]:
        """
        Returns the feedthrough matrix D.
        """
    @typing.overload
    def D(self, i: int, j: int) -> float:
        """
        Returns an element of the feedthrough matrix D.
        
        :param i: Row of D.
        :param j: Column of D.
        """
    def __init__(self, A: numpy.ndarray[numpy.float64[1, 1]], B: numpy.ndarray[numpy.float64[1, 1]], C: numpy.ndarray[numpy.float64[2, 1]], D: numpy.ndarray[numpy.float64[2, 1]]) -> None:
        """
        Constructs a discrete plant with the given continuous system coefficients.
        
        :param A: System matrix.
        :param B: Input matrix.
        :param C: Output matrix.
        :param D: Feedthrough matrix.
                  @throws std::domain_error if any matrix element isn't finite.
        """
    def calculateX(self, x: numpy.ndarray[numpy.float64[1, 1]], clampedU: numpy.ndarray[numpy.float64[1, 1]], dt: wpimath.units.seconds) -> numpy.ndarray[numpy.float64[1, 1]]:
        """
        Computes the new x given the old x and the control input.
        
        This is used by state observers directly to run updates based on state
        estimate.
        
        :param x:        The current state.
        :param clampedU: The control input.
        :param dt:       Timestep for model update.
        """
    def calculateY(self, x: numpy.ndarray[numpy.float64[1, 1]], clampedU: numpy.ndarray[numpy.float64[1, 1]]) -> numpy.ndarray[numpy.float64[2, 1]]:
        """
        Computes the new y given the control input.
        
        This is used by state observers directly to run updates based on state
        estimate.
        
        :param x:        The current state.
        :param clampedU: The control input.
        """
class LinearSystem_1_1_3:
    """
    A plant defined using state-space notation.
    
    A plant is a mathematical model of a system's dynamics.
    
    For more on the underlying math, read
    https://file.tavsys.net/control/controls-engineering-in-frc.pdf.
    
    @tparam States Number of states.
    @tparam Inputs Number of inputs.
    @tparam Outputs Number of outputs.
    """
    @typing.overload
    def A(self) -> numpy.ndarray[numpy.float64[1, 1]]:
        """
        Returns the system matrix A.
        """
    @typing.overload
    def A(self, i: int, j: int) -> float:
        """
        Returns an element of the system matrix A.
        
        :param i: Row of A.
        :param j: Column of A.
        """
    @typing.overload
    def B(self) -> numpy.ndarray[numpy.float64[1, 1]]:
        """
        Returns the input matrix B.
        """
    @typing.overload
    def B(self, i: int, j: int) -> float:
        """
        Returns an element of the input matrix B.
        
        :param i: Row of B.
        :param j: Column of B.
        """
    @typing.overload
    def C(self) -> numpy.ndarray[numpy.float64[3, 1]]:
        """
        Returns the output matrix C.
        """
    @typing.overload
    def C(self, i: int, j: int) -> float:
        """
        Returns an element of the output matrix C.
        
        :param i: Row of C.
        :param j: Column of C.
        """
    @typing.overload
    def D(self) -> numpy.ndarray[numpy.float64[3, 1]]:
        """
        Returns the feedthrough matrix D.
        """
    @typing.overload
    def D(self, i: int, j: int) -> float:
        """
        Returns an element of the feedthrough matrix D.
        
        :param i: Row of D.
        :param j: Column of D.
        """
    def __init__(self, A: numpy.ndarray[numpy.float64[1, 1]], B: numpy.ndarray[numpy.float64[1, 1]], C: numpy.ndarray[numpy.float64[3, 1]], D: numpy.ndarray[numpy.float64[3, 1]]) -> None:
        """
        Constructs a discrete plant with the given continuous system coefficients.
        
        :param A: System matrix.
        :param B: Input matrix.
        :param C: Output matrix.
        :param D: Feedthrough matrix.
                  @throws std::domain_error if any matrix element isn't finite.
        """
    def calculateX(self, x: numpy.ndarray[numpy.float64[1, 1]], clampedU: numpy.ndarray[numpy.float64[1, 1]], dt: wpimath.units.seconds) -> numpy.ndarray[numpy.float64[1, 1]]:
        """
        Computes the new x given the old x and the control input.
        
        This is used by state observers directly to run updates based on state
        estimate.
        
        :param x:        The current state.
        :param clampedU: The control input.
        :param dt:       Timestep for model update.
        """
    def calculateY(self, x: numpy.ndarray[numpy.float64[1, 1]], clampedU: numpy.ndarray[numpy.float64[1, 1]]) -> numpy.ndarray[numpy.float64[3, 1]]:
        """
        Computes the new y given the control input.
        
        This is used by state observers directly to run updates based on state
        estimate.
        
        :param x:        The current state.
        :param clampedU: The control input.
        """
class LinearSystem_2_1_1:
    """
    A plant defined using state-space notation.
    
    A plant is a mathematical model of a system's dynamics.
    
    For more on the underlying math, read
    https://file.tavsys.net/control/controls-engineering-in-frc.pdf.
    
    @tparam States Number of states.
    @tparam Inputs Number of inputs.
    @tparam Outputs Number of outputs.
    """
    @typing.overload
    def A(self) -> numpy.ndarray[numpy.float64[2, 2]]:
        """
        Returns the system matrix A.
        """
    @typing.overload
    def A(self, i: int, j: int) -> float:
        """
        Returns an element of the system matrix A.
        
        :param i: Row of A.
        :param j: Column of A.
        """
    @typing.overload
    def B(self) -> numpy.ndarray[numpy.float64[2, 1]]:
        """
        Returns the input matrix B.
        """
    @typing.overload
    def B(self, i: int, j: int) -> float:
        """
        Returns an element of the input matrix B.
        
        :param i: Row of B.
        :param j: Column of B.
        """
    @typing.overload
    def C(self) -> numpy.ndarray[numpy.float64[1, 2]]:
        """
        Returns the output matrix C.
        """
    @typing.overload
    def C(self, i: int, j: int) -> float:
        """
        Returns an element of the output matrix C.
        
        :param i: Row of C.
        :param j: Column of C.
        """
    @typing.overload
    def D(self) -> numpy.ndarray[numpy.float64[1, 1]]:
        """
        Returns the feedthrough matrix D.
        """
    @typing.overload
    def D(self, i: int, j: int) -> float:
        """
        Returns an element of the feedthrough matrix D.
        
        :param i: Row of D.
        :param j: Column of D.
        """
    def __init__(self, A: numpy.ndarray[numpy.float64[2, 2]], B: numpy.ndarray[numpy.float64[2, 1]], C: numpy.ndarray[numpy.float64[1, 2]], D: numpy.ndarray[numpy.float64[1, 1]]) -> None:
        """
        Constructs a discrete plant with the given continuous system coefficients.
        
        :param A: System matrix.
        :param B: Input matrix.
        :param C: Output matrix.
        :param D: Feedthrough matrix.
                  @throws std::domain_error if any matrix element isn't finite.
        """
    def calculateX(self, x: numpy.ndarray[numpy.float64[2, 1]], clampedU: numpy.ndarray[numpy.float64[1, 1]], dt: wpimath.units.seconds) -> numpy.ndarray[numpy.float64[2, 1]]:
        """
        Computes the new x given the old x and the control input.
        
        This is used by state observers directly to run updates based on state
        estimate.
        
        :param x:        The current state.
        :param clampedU: The control input.
        :param dt:       Timestep for model update.
        """
    def calculateY(self, x: numpy.ndarray[numpy.float64[2, 1]], clampedU: numpy.ndarray[numpy.float64[1, 1]]) -> numpy.ndarray[numpy.float64[1, 1]]:
        """
        Computes the new y given the control input.
        
        This is used by state observers directly to run updates based on state
        estimate.
        
        :param x:        The current state.
        :param clampedU: The control input.
        """
class LinearSystem_2_1_2:
    """
    A plant defined using state-space notation.
    
    A plant is a mathematical model of a system's dynamics.
    
    For more on the underlying math, read
    https://file.tavsys.net/control/controls-engineering-in-frc.pdf.
    
    @tparam States Number of states.
    @tparam Inputs Number of inputs.
    @tparam Outputs Number of outputs.
    """
    @typing.overload
    def A(self) -> numpy.ndarray[numpy.float64[2, 2]]:
        """
        Returns the system matrix A.
        """
    @typing.overload
    def A(self, i: int, j: int) -> float:
        """
        Returns an element of the system matrix A.
        
        :param i: Row of A.
        :param j: Column of A.
        """
    @typing.overload
    def B(self) -> numpy.ndarray[numpy.float64[2, 1]]:
        """
        Returns the input matrix B.
        """
    @typing.overload
    def B(self, i: int, j: int) -> float:
        """
        Returns an element of the input matrix B.
        
        :param i: Row of B.
        :param j: Column of B.
        """
    @typing.overload
    def C(self) -> numpy.ndarray[numpy.float64[2, 2]]:
        """
        Returns the output matrix C.
        """
    @typing.overload
    def C(self, i: int, j: int) -> float:
        """
        Returns an element of the output matrix C.
        
        :param i: Row of C.
        :param j: Column of C.
        """
    @typing.overload
    def D(self) -> numpy.ndarray[numpy.float64[2, 1]]:
        """
        Returns the feedthrough matrix D.
        """
    @typing.overload
    def D(self, i: int, j: int) -> float:
        """
        Returns an element of the feedthrough matrix D.
        
        :param i: Row of D.
        :param j: Column of D.
        """
    def __init__(self, A: numpy.ndarray[numpy.float64[2, 2]], B: numpy.ndarray[numpy.float64[2, 1]], C: numpy.ndarray[numpy.float64[2, 2]], D: numpy.ndarray[numpy.float64[2, 1]]) -> None:
        """
        Constructs a discrete plant with the given continuous system coefficients.
        
        :param A: System matrix.
        :param B: Input matrix.
        :param C: Output matrix.
        :param D: Feedthrough matrix.
                  @throws std::domain_error if any matrix element isn't finite.
        """
    def calculateX(self, x: numpy.ndarray[numpy.float64[2, 1]], clampedU: numpy.ndarray[numpy.float64[1, 1]], dt: wpimath.units.seconds) -> numpy.ndarray[numpy.float64[2, 1]]:
        """
        Computes the new x given the old x and the control input.
        
        This is used by state observers directly to run updates based on state
        estimate.
        
        :param x:        The current state.
        :param clampedU: The control input.
        :param dt:       Timestep for model update.
        """
    def calculateY(self, x: numpy.ndarray[numpy.float64[2, 1]], clampedU: numpy.ndarray[numpy.float64[1, 1]]) -> numpy.ndarray[numpy.float64[2, 1]]:
        """
        Computes the new y given the control input.
        
        This is used by state observers directly to run updates based on state
        estimate.
        
        :param x:        The current state.
        :param clampedU: The control input.
        """
class LinearSystem_2_1_3:
    """
    A plant defined using state-space notation.
    
    A plant is a mathematical model of a system's dynamics.
    
    For more on the underlying math, read
    https://file.tavsys.net/control/controls-engineering-in-frc.pdf.
    
    @tparam States Number of states.
    @tparam Inputs Number of inputs.
    @tparam Outputs Number of outputs.
    """
    @typing.overload
    def A(self) -> numpy.ndarray[numpy.float64[2, 2]]:
        """
        Returns the system matrix A.
        """
    @typing.overload
    def A(self, i: int, j: int) -> float:
        """
        Returns an element of the system matrix A.
        
        :param i: Row of A.
        :param j: Column of A.
        """
    @typing.overload
    def B(self) -> numpy.ndarray[numpy.float64[2, 1]]:
        """
        Returns the input matrix B.
        """
    @typing.overload
    def B(self, i: int, j: int) -> float:
        """
        Returns an element of the input matrix B.
        
        :param i: Row of B.
        :param j: Column of B.
        """
    @typing.overload
    def C(self) -> numpy.ndarray[numpy.float64[3, 2]]:
        """
        Returns the output matrix C.
        """
    @typing.overload
    def C(self, i: int, j: int) -> float:
        """
        Returns an element of the output matrix C.
        
        :param i: Row of C.
        :param j: Column of C.
        """
    @typing.overload
    def D(self) -> numpy.ndarray[numpy.float64[3, 1]]:
        """
        Returns the feedthrough matrix D.
        """
    @typing.overload
    def D(self, i: int, j: int) -> float:
        """
        Returns an element of the feedthrough matrix D.
        
        :param i: Row of D.
        :param j: Column of D.
        """
    def __init__(self, A: numpy.ndarray[numpy.float64[2, 2]], B: numpy.ndarray[numpy.float64[2, 1]], C: numpy.ndarray[numpy.float64[3, 2]], D: numpy.ndarray[numpy.float64[3, 1]]) -> None:
        """
        Constructs a discrete plant with the given continuous system coefficients.
        
        :param A: System matrix.
        :param B: Input matrix.
        :param C: Output matrix.
        :param D: Feedthrough matrix.
                  @throws std::domain_error if any matrix element isn't finite.
        """
    def calculateX(self, x: numpy.ndarray[numpy.float64[2, 1]], clampedU: numpy.ndarray[numpy.float64[1, 1]], dt: wpimath.units.seconds) -> numpy.ndarray[numpy.float64[2, 1]]:
        """
        Computes the new x given the old x and the control input.
        
        This is used by state observers directly to run updates based on state
        estimate.
        
        :param x:        The current state.
        :param clampedU: The control input.
        :param dt:       Timestep for model update.
        """
    def calculateY(self, x: numpy.ndarray[numpy.float64[2, 1]], clampedU: numpy.ndarray[numpy.float64[1, 1]]) -> numpy.ndarray[numpy.float64[3, 1]]:
        """
        Computes the new y given the control input.
        
        This is used by state observers directly to run updates based on state
        estimate.
        
        :param x:        The current state.
        :param clampedU: The control input.
        """
class LinearSystem_2_2_1:
    """
    A plant defined using state-space notation.
    
    A plant is a mathematical model of a system's dynamics.
    
    For more on the underlying math, read
    https://file.tavsys.net/control/controls-engineering-in-frc.pdf.
    
    @tparam States Number of states.
    @tparam Inputs Number of inputs.
    @tparam Outputs Number of outputs.
    """
    @typing.overload
    def A(self) -> numpy.ndarray[numpy.float64[2, 2]]:
        """
        Returns the system matrix A.
        """
    @typing.overload
    def A(self, i: int, j: int) -> float:
        """
        Returns an element of the system matrix A.
        
        :param i: Row of A.
        :param j: Column of A.
        """
    @typing.overload
    def B(self) -> numpy.ndarray[numpy.float64[2, 2]]:
        """
        Returns the input matrix B.
        """
    @typing.overload
    def B(self, i: int, j: int) -> float:
        """
        Returns an element of the input matrix B.
        
        :param i: Row of B.
        :param j: Column of B.
        """
    @typing.overload
    def C(self) -> numpy.ndarray[numpy.float64[1, 2]]:
        """
        Returns the output matrix C.
        """
    @typing.overload
    def C(self, i: int, j: int) -> float:
        """
        Returns an element of the output matrix C.
        
        :param i: Row of C.
        :param j: Column of C.
        """
    @typing.overload
    def D(self) -> numpy.ndarray[numpy.float64[1, 2]]:
        """
        Returns the feedthrough matrix D.
        """
    @typing.overload
    def D(self, i: int, j: int) -> float:
        """
        Returns an element of the feedthrough matrix D.
        
        :param i: Row of D.
        :param j: Column of D.
        """
    def __init__(self, A: numpy.ndarray[numpy.float64[2, 2]], B: numpy.ndarray[numpy.float64[2, 2]], C: numpy.ndarray[numpy.float64[1, 2]], D: numpy.ndarray[numpy.float64[1, 2]]) -> None:
        """
        Constructs a discrete plant with the given continuous system coefficients.
        
        :param A: System matrix.
        :param B: Input matrix.
        :param C: Output matrix.
        :param D: Feedthrough matrix.
                  @throws std::domain_error if any matrix element isn't finite.
        """
    def calculateX(self, x: numpy.ndarray[numpy.float64[2, 1]], clampedU: numpy.ndarray[numpy.float64[2, 1]], dt: wpimath.units.seconds) -> numpy.ndarray[numpy.float64[2, 1]]:
        """
        Computes the new x given the old x and the control input.
        
        This is used by state observers directly to run updates based on state
        estimate.
        
        :param x:        The current state.
        :param clampedU: The control input.
        :param dt:       Timestep for model update.
        """
    def calculateY(self, x: numpy.ndarray[numpy.float64[2, 1]], clampedU: numpy.ndarray[numpy.float64[2, 1]]) -> numpy.ndarray[numpy.float64[1, 1]]:
        """
        Computes the new y given the control input.
        
        This is used by state observers directly to run updates based on state
        estimate.
        
        :param x:        The current state.
        :param clampedU: The control input.
        """
class LinearSystem_2_2_2:
    """
    A plant defined using state-space notation.
    
    A plant is a mathematical model of a system's dynamics.
    
    For more on the underlying math, read
    https://file.tavsys.net/control/controls-engineering-in-frc.pdf.
    
    @tparam States Number of states.
    @tparam Inputs Number of inputs.
    @tparam Outputs Number of outputs.
    """
    @typing.overload
    def A(self) -> numpy.ndarray[numpy.float64[2, 2]]:
        """
        Returns the system matrix A.
        """
    @typing.overload
    def A(self, i: int, j: int) -> float:
        """
        Returns an element of the system matrix A.
        
        :param i: Row of A.
        :param j: Column of A.
        """
    @typing.overload
    def B(self) -> numpy.ndarray[numpy.float64[2, 2]]:
        """
        Returns the input matrix B.
        """
    @typing.overload
    def B(self, i: int, j: int) -> float:
        """
        Returns an element of the input matrix B.
        
        :param i: Row of B.
        :param j: Column of B.
        """
    @typing.overload
    def C(self) -> numpy.ndarray[numpy.float64[2, 2]]:
        """
        Returns the output matrix C.
        """
    @typing.overload
    def C(self, i: int, j: int) -> float:
        """
        Returns an element of the output matrix C.
        
        :param i: Row of C.
        :param j: Column of C.
        """
    @typing.overload
    def D(self) -> numpy.ndarray[numpy.float64[2, 2]]:
        """
        Returns the feedthrough matrix D.
        """
    @typing.overload
    def D(self, i: int, j: int) -> float:
        """
        Returns an element of the feedthrough matrix D.
        
        :param i: Row of D.
        :param j: Column of D.
        """
    def __init__(self, A: numpy.ndarray[numpy.float64[2, 2]], B: numpy.ndarray[numpy.float64[2, 2]], C: numpy.ndarray[numpy.float64[2, 2]], D: numpy.ndarray[numpy.float64[2, 2]]) -> None:
        """
        Constructs a discrete plant with the given continuous system coefficients.
        
        :param A: System matrix.
        :param B: Input matrix.
        :param C: Output matrix.
        :param D: Feedthrough matrix.
                  @throws std::domain_error if any matrix element isn't finite.
        """
    def calculateX(self, x: numpy.ndarray[numpy.float64[2, 1]], clampedU: numpy.ndarray[numpy.float64[2, 1]], dt: wpimath.units.seconds) -> numpy.ndarray[numpy.float64[2, 1]]:
        """
        Computes the new x given the old x and the control input.
        
        This is used by state observers directly to run updates based on state
        estimate.
        
        :param x:        The current state.
        :param clampedU: The control input.
        :param dt:       Timestep for model update.
        """
    def calculateY(self, x: numpy.ndarray[numpy.float64[2, 1]], clampedU: numpy.ndarray[numpy.float64[2, 1]]) -> numpy.ndarray[numpy.float64[2, 1]]:
        """
        Computes the new y given the control input.
        
        This is used by state observers directly to run updates based on state
        estimate.
        
        :param x:        The current state.
        :param clampedU: The control input.
        """
class LinearSystem_2_2_3:
    """
    A plant defined using state-space notation.
    
    A plant is a mathematical model of a system's dynamics.
    
    For more on the underlying math, read
    https://file.tavsys.net/control/controls-engineering-in-frc.pdf.
    
    @tparam States Number of states.
    @tparam Inputs Number of inputs.
    @tparam Outputs Number of outputs.
    """
    @typing.overload
    def A(self) -> numpy.ndarray[numpy.float64[2, 2]]:
        """
        Returns the system matrix A.
        """
    @typing.overload
    def A(self, i: int, j: int) -> float:
        """
        Returns an element of the system matrix A.
        
        :param i: Row of A.
        :param j: Column of A.
        """
    @typing.overload
    def B(self) -> numpy.ndarray[numpy.float64[2, 2]]:
        """
        Returns the input matrix B.
        """
    @typing.overload
    def B(self, i: int, j: int) -> float:
        """
        Returns an element of the input matrix B.
        
        :param i: Row of B.
        :param j: Column of B.
        """
    @typing.overload
    def C(self) -> numpy.ndarray[numpy.float64[3, 2]]:
        """
        Returns the output matrix C.
        """
    @typing.overload
    def C(self, i: int, j: int) -> float:
        """
        Returns an element of the output matrix C.
        
        :param i: Row of C.
        :param j: Column of C.
        """
    @typing.overload
    def D(self) -> numpy.ndarray[numpy.float64[3, 2]]:
        """
        Returns the feedthrough matrix D.
        """
    @typing.overload
    def D(self, i: int, j: int) -> float:
        """
        Returns an element of the feedthrough matrix D.
        
        :param i: Row of D.
        :param j: Column of D.
        """
    def __init__(self, A: numpy.ndarray[numpy.float64[2, 2]], B: numpy.ndarray[numpy.float64[2, 2]], C: numpy.ndarray[numpy.float64[3, 2]], D: numpy.ndarray[numpy.float64[3, 2]]) -> None:
        """
        Constructs a discrete plant with the given continuous system coefficients.
        
        :param A: System matrix.
        :param B: Input matrix.
        :param C: Output matrix.
        :param D: Feedthrough matrix.
                  @throws std::domain_error if any matrix element isn't finite.
        """
    def calculateX(self, x: numpy.ndarray[numpy.float64[2, 1]], clampedU: numpy.ndarray[numpy.float64[2, 1]], dt: wpimath.units.seconds) -> numpy.ndarray[numpy.float64[2, 1]]:
        """
        Computes the new x given the old x and the control input.
        
        This is used by state observers directly to run updates based on state
        estimate.
        
        :param x:        The current state.
        :param clampedU: The control input.
        :param dt:       Timestep for model update.
        """
    def calculateY(self, x: numpy.ndarray[numpy.float64[2, 1]], clampedU: numpy.ndarray[numpy.float64[2, 1]]) -> numpy.ndarray[numpy.float64[3, 1]]:
        """
        Computes the new y given the control input.
        
        This is used by state observers directly to run updates based on state
        estimate.
        
        :param x:        The current state.
        :param clampedU: The control input.
        """
class LinearSystem_3_2_1:
    """
    A plant defined using state-space notation.
    
    A plant is a mathematical model of a system's dynamics.
    
    For more on the underlying math, read
    https://file.tavsys.net/control/controls-engineering-in-frc.pdf.
    
    @tparam States Number of states.
    @tparam Inputs Number of inputs.
    @tparam Outputs Number of outputs.
    """
    @typing.overload
    def A(self) -> numpy.ndarray[numpy.float64[3, 3]]:
        """
        Returns the system matrix A.
        """
    @typing.overload
    def A(self, i: int, j: int) -> float:
        """
        Returns an element of the system matrix A.
        
        :param i: Row of A.
        :param j: Column of A.
        """
    @typing.overload
    def B(self) -> numpy.ndarray[numpy.float64[3, 2]]:
        """
        Returns the input matrix B.
        """
    @typing.overload
    def B(self, i: int, j: int) -> float:
        """
        Returns an element of the input matrix B.
        
        :param i: Row of B.
        :param j: Column of B.
        """
    @typing.overload
    def C(self) -> numpy.ndarray[numpy.float64[1, 3]]:
        """
        Returns the output matrix C.
        """
    @typing.overload
    def C(self, i: int, j: int) -> float:
        """
        Returns an element of the output matrix C.
        
        :param i: Row of C.
        :param j: Column of C.
        """
    @typing.overload
    def D(self) -> numpy.ndarray[numpy.float64[1, 2]]:
        """
        Returns the feedthrough matrix D.
        """
    @typing.overload
    def D(self, i: int, j: int) -> float:
        """
        Returns an element of the feedthrough matrix D.
        
        :param i: Row of D.
        :param j: Column of D.
        """
    def __init__(self, A: numpy.ndarray[numpy.float64[3, 3]], B: numpy.ndarray[numpy.float64[3, 2]], C: numpy.ndarray[numpy.float64[1, 3]], D: numpy.ndarray[numpy.float64[1, 2]]) -> None:
        """
        Constructs a discrete plant with the given continuous system coefficients.
        
        :param A: System matrix.
        :param B: Input matrix.
        :param C: Output matrix.
        :param D: Feedthrough matrix.
                  @throws std::domain_error if any matrix element isn't finite.
        """
    def calculateX(self, x: numpy.ndarray[numpy.float64[3, 1]], clampedU: numpy.ndarray[numpy.float64[2, 1]], dt: wpimath.units.seconds) -> numpy.ndarray[numpy.float64[3, 1]]:
        """
        Computes the new x given the old x and the control input.
        
        This is used by state observers directly to run updates based on state
        estimate.
        
        :param x:        The current state.
        :param clampedU: The control input.
        :param dt:       Timestep for model update.
        """
    def calculateY(self, x: numpy.ndarray[numpy.float64[3, 1]], clampedU: numpy.ndarray[numpy.float64[2, 1]]) -> numpy.ndarray[numpy.float64[1, 1]]:
        """
        Computes the new y given the control input.
        
        This is used by state observers directly to run updates based on state
        estimate.
        
        :param x:        The current state.
        :param clampedU: The control input.
        """
class LinearSystem_3_2_2:
    """
    A plant defined using state-space notation.
    
    A plant is a mathematical model of a system's dynamics.
    
    For more on the underlying math, read
    https://file.tavsys.net/control/controls-engineering-in-frc.pdf.
    
    @tparam States Number of states.
    @tparam Inputs Number of inputs.
    @tparam Outputs Number of outputs.
    """
    @typing.overload
    def A(self) -> numpy.ndarray[numpy.float64[3, 3]]:
        """
        Returns the system matrix A.
        """
    @typing.overload
    def A(self, i: int, j: int) -> float:
        """
        Returns an element of the system matrix A.
        
        :param i: Row of A.
        :param j: Column of A.
        """
    @typing.overload
    def B(self) -> numpy.ndarray[numpy.float64[3, 2]]:
        """
        Returns the input matrix B.
        """
    @typing.overload
    def B(self, i: int, j: int) -> float:
        """
        Returns an element of the input matrix B.
        
        :param i: Row of B.
        :param j: Column of B.
        """
    @typing.overload
    def C(self) -> numpy.ndarray[numpy.float64[2, 3]]:
        """
        Returns the output matrix C.
        """
    @typing.overload
    def C(self, i: int, j: int) -> float:
        """
        Returns an element of the output matrix C.
        
        :param i: Row of C.
        :param j: Column of C.
        """
    @typing.overload
    def D(self) -> numpy.ndarray[numpy.float64[2, 2]]:
        """
        Returns the feedthrough matrix D.
        """
    @typing.overload
    def D(self, i: int, j: int) -> float:
        """
        Returns an element of the feedthrough matrix D.
        
        :param i: Row of D.
        :param j: Column of D.
        """
    def __init__(self, A: numpy.ndarray[numpy.float64[3, 3]], B: numpy.ndarray[numpy.float64[3, 2]], C: numpy.ndarray[numpy.float64[2, 3]], D: numpy.ndarray[numpy.float64[2, 2]]) -> None:
        """
        Constructs a discrete plant with the given continuous system coefficients.
        
        :param A: System matrix.
        :param B: Input matrix.
        :param C: Output matrix.
        :param D: Feedthrough matrix.
                  @throws std::domain_error if any matrix element isn't finite.
        """
    def calculateX(self, x: numpy.ndarray[numpy.float64[3, 1]], clampedU: numpy.ndarray[numpy.float64[2, 1]], dt: wpimath.units.seconds) -> numpy.ndarray[numpy.float64[3, 1]]:
        """
        Computes the new x given the old x and the control input.
        
        This is used by state observers directly to run updates based on state
        estimate.
        
        :param x:        The current state.
        :param clampedU: The control input.
        :param dt:       Timestep for model update.
        """
    def calculateY(self, x: numpy.ndarray[numpy.float64[3, 1]], clampedU: numpy.ndarray[numpy.float64[2, 1]]) -> numpy.ndarray[numpy.float64[2, 1]]:
        """
        Computes the new y given the control input.
        
        This is used by state observers directly to run updates based on state
        estimate.
        
        :param x:        The current state.
        :param clampedU: The control input.
        """
class LinearSystem_3_2_3:
    """
    A plant defined using state-space notation.
    
    A plant is a mathematical model of a system's dynamics.
    
    For more on the underlying math, read
    https://file.tavsys.net/control/controls-engineering-in-frc.pdf.
    
    @tparam States Number of states.
    @tparam Inputs Number of inputs.
    @tparam Outputs Number of outputs.
    """
    @typing.overload
    def A(self) -> numpy.ndarray[numpy.float64[3, 3]]:
        """
        Returns the system matrix A.
        """
    @typing.overload
    def A(self, i: int, j: int) -> float:
        """
        Returns an element of the system matrix A.
        
        :param i: Row of A.
        :param j: Column of A.
        """
    @typing.overload
    def B(self) -> numpy.ndarray[numpy.float64[3, 2]]:
        """
        Returns the input matrix B.
        """
    @typing.overload
    def B(self, i: int, j: int) -> float:
        """
        Returns an element of the input matrix B.
        
        :param i: Row of B.
        :param j: Column of B.
        """
    @typing.overload
    def C(self) -> numpy.ndarray[numpy.float64[3, 3]]:
        """
        Returns the output matrix C.
        """
    @typing.overload
    def C(self, i: int, j: int) -> float:
        """
        Returns an element of the output matrix C.
        
        :param i: Row of C.
        :param j: Column of C.
        """
    @typing.overload
    def D(self) -> numpy.ndarray[numpy.float64[3, 2]]:
        """
        Returns the feedthrough matrix D.
        """
    @typing.overload
    def D(self, i: int, j: int) -> float:
        """
        Returns an element of the feedthrough matrix D.
        
        :param i: Row of D.
        :param j: Column of D.
        """
    def __init__(self, A: numpy.ndarray[numpy.float64[3, 3]], B: numpy.ndarray[numpy.float64[3, 2]], C: numpy.ndarray[numpy.float64[3, 3]], D: numpy.ndarray[numpy.float64[3, 2]]) -> None:
        """
        Constructs a discrete plant with the given continuous system coefficients.
        
        :param A: System matrix.
        :param B: Input matrix.
        :param C: Output matrix.
        :param D: Feedthrough matrix.
                  @throws std::domain_error if any matrix element isn't finite.
        """
    def calculateX(self, x: numpy.ndarray[numpy.float64[3, 1]], clampedU: numpy.ndarray[numpy.float64[2, 1]], dt: wpimath.units.seconds) -> numpy.ndarray[numpy.float64[3, 1]]:
        """
        Computes the new x given the old x and the control input.
        
        This is used by state observers directly to run updates based on state
        estimate.
        
        :param x:        The current state.
        :param clampedU: The control input.
        :param dt:       Timestep for model update.
        """
    def calculateY(self, x: numpy.ndarray[numpy.float64[3, 1]], clampedU: numpy.ndarray[numpy.float64[2, 1]]) -> numpy.ndarray[numpy.float64[3, 1]]:
        """
        Computes the new y given the control input.
        
        This is used by state observers directly to run updates based on state
        estimate.
        
        :param x:        The current state.
        :param clampedU: The control input.
        """
