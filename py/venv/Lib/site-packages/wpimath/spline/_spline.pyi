from __future__ import annotations
import numpy
import typing
import wpimath.geometry._geometry
import wpimath.units
__all__ = ['CubicHermiteSpline', 'QuinticHermiteSpline', 'Spline3', 'Spline5', 'SplineHelper', 'SplineParameterizer']
class CubicHermiteSpline(Spline3):
    """
    Represents a hermite spline of degree 3.
    """
    def __init__(self, xInitialControlVector: tuple[float, float], xFinalControlVector: tuple[float, float], yInitialControlVector: tuple[float, float], yFinalControlVector: tuple[float, float]) -> None:
        """
        Constructs a cubic hermite spline with the specified control vectors. Each
        control vector contains info about the location of the point and its first
        derivative.
        
        :param xInitialControlVector: The control vector for the initial point in
                                      the x dimension.
        :param xFinalControlVector:   The control vector for the final point in
                                      the x dimension.
        :param yInitialControlVector: The control vector for the initial point in
                                      the y dimension.
        :param yFinalControlVector:   The control vector for the final point in
                                      the y dimension.
        """
    def getFinalControlVector(self) -> Spline3.ControlVector:
        """
        Returns the final control vector that created this spline.
        
        :returns: The final control vector that created this spline.
        """
    def getInitialControlVector(self) -> Spline3.ControlVector:
        """
        Returns the initial control vector that created this spline.
        
        :returns: The initial control vector that created this spline.
        """
class QuinticHermiteSpline(Spline5):
    """
    Represents a hermite spline of degree 5.
    """
    def __init__(self, xInitialControlVector: tuple[float, float, float], xFinalControlVector: tuple[float, float, float], yInitialControlVector: tuple[float, float, float], yFinalControlVector: tuple[float, float, float]) -> None:
        """
        Constructs a quintic hermite spline with the specified control vectors.
        Each control vector contains into about the location of the point, its
        first derivative, and its second derivative.
        
        :param xInitialControlVector: The control vector for the initial point in
                                      the x dimension.
        :param xFinalControlVector:   The control vector for the final point in
                                      the x dimension.
        :param yInitialControlVector: The control vector for the initial point in
                                      the y dimension.
        :param yFinalControlVector:   The control vector for the final point in
                                      the y dimension.
        """
    def getFinalControlVector(self) -> Spline5.ControlVector:
        """
        Returns the final control vector that created this spline.
        
        :returns: The final control vector that created this spline.
        """
    def getInitialControlVector(self) -> Spline5.ControlVector:
        """
        Returns the initial control vector that created this spline.
        
        :returns: The initial control vector that created this spline.
        """
class Spline3:
    """
    Represents a two-dimensional parametric spline that interpolates between two
    points.
    
    @tparam Degree The degree of the spline.
    """
    class ControlVector:
        """
        Represents a control vector for a spline.
        
        Each element in each array represents the value of the derivative at the
        index. For example, the value of x[2] is the second derivative in the x
        dimension.
        """
        def __init__(self, x: tuple[float, float], y: tuple[float, float]) -> None:
            ...
        @property
        def x(self) -> tuple[float, float]:
            """
            The x components of the control vector.
            """
        @x.setter
        def x(self, arg0: tuple[float, float]) -> None:
            ...
        @property
        def y(self) -> tuple[float, float]:
            """
            The y components of the control vector.
            """
        @y.setter
        def y(self, arg0: tuple[float, float]) -> None:
            ...
    def __init__(self) -> None:
        ...
    def coefficients(self) -> numpy.ndarray[numpy.float64[6, 4]]:
        """
        Returns the coefficients of the spline.
        
        :returns: The coefficients of the spline.
        """
    def getFinalControlVector(self) -> Spline3.ControlVector:
        """
        Returns the final control vector that created this spline.
        
        :returns: The final control vector that created this spline.
        """
    def getInitialControlVector(self) -> Spline3.ControlVector:
        """
        Returns the initial control vector that created this spline.
        
        :returns: The initial control vector that created this spline.
        """
    def getPoint(self, t: float) -> tuple[wpimath.geometry._geometry.Pose2d, wpimath.units.radians_per_meter]:
        """
        Gets the pose and curvature at some point t on the spline.
        
        :param t: The point t
        
        :returns: The pose and curvature at that point.
        """
class Spline5:
    """
    Represents a two-dimensional parametric spline that interpolates between two
    points.
    
    @tparam Degree The degree of the spline.
    """
    class ControlVector:
        """
        Represents a control vector for a spline.
        
        Each element in each array represents the value of the derivative at the
        index. For example, the value of x[2] is the second derivative in the x
        dimension.
        """
        def __init__(self, x: tuple[float, float, float], y: tuple[float, float, float]) -> None:
            ...
        @property
        def x(self) -> tuple[float, float, float]:
            """
            The x components of the control vector.
            """
        @x.setter
        def x(self, arg0: tuple[float, float, float]) -> None:
            ...
        @property
        def y(self) -> tuple[float, float, float]:
            """
            The y components of the control vector.
            """
        @y.setter
        def y(self, arg0: tuple[float, float, float]) -> None:
            ...
    def __init__(self) -> None:
        ...
    def coefficients(self) -> numpy.ndarray[numpy.float64[6, 6]]:
        """
        Returns the coefficients of the spline.
        
        :returns: The coefficients of the spline.
        """
    def getFinalControlVector(self) -> Spline5.ControlVector:
        """
        Returns the final control vector that created this spline.
        
        :returns: The final control vector that created this spline.
        """
    def getInitialControlVector(self) -> Spline5.ControlVector:
        """
        Returns the initial control vector that created this spline.
        
        :returns: The initial control vector that created this spline.
        """
    def getPoint(self, t: float) -> tuple[wpimath.geometry._geometry.Pose2d, wpimath.units.radians_per_meter]:
        """
        Gets the pose and curvature at some point t on the spline.
        
        :param t: The point t
        
        :returns: The pose and curvature at that point.
        """
class SplineHelper:
    """
    Helper class that is used to generate cubic and quintic splines from user
    provided waypoints.
    """
    @staticmethod
    def cubicControlVectorsFromWaypoints(start: wpimath.geometry._geometry.Pose2d, interiorWaypoints: list[wpimath.geometry._geometry.Translation2d], end: wpimath.geometry._geometry.Pose2d) -> tuple[Spline3.ControlVector, Spline3.ControlVector]:
        """
        Returns 2 cubic control vectors from a set of exterior waypoints and
        interior translations.
        
        :param start:             The starting pose.
        :param interiorWaypoints: The interior waypoints.
        :param end:               The ending pose.
        
        :returns: 2 cubic control vectors.
        """
    @staticmethod
    def cubicSplinesFromControlVectors(start: Spline3.ControlVector, waypoints: list[wpimath.geometry._geometry.Translation2d], end: Spline3.ControlVector) -> list[CubicHermiteSpline]:
        """
        Returns a set of cubic splines corresponding to the provided control
        vectors. The user is free to set the direction of the start and end
        point. The directions for the middle waypoints are determined
        automatically to ensure continuous curvature throughout the path.
        
        The derivation for the algorithm used can be found here:
        <https://www.uio.no/studier/emner/matnat/ifi/nedlagte-emner/INF-MAT4350/h08/undervisningsmateriale/chap7alecture.pdf>
        
        :param start:     The starting control vector.
        :param waypoints: The middle waypoints. This can be left blank if you
                          only wish to create a path with two waypoints.
        :param end:       The ending control vector.
        
        :returns: A vector of cubic hermite splines that interpolate through the
                  provided waypoints.
        """
    @staticmethod
    def optimizeCurvature(splines: list[QuinticHermiteSpline]) -> list[QuinticHermiteSpline]:
        """
        Optimizes the curvature of 2 or more quintic splines at knot points.
        Overall, this reduces the integral of the absolute value of the second
        derivative across the set of splines.
        
        :param splines: A vector of un-optimized quintic splines.
        
        :returns: A vector of optimized quintic splines.
        """
    @staticmethod
    def quinticSplinesFromControlVectors(controlVectors: list[Spline5.ControlVector]) -> list[QuinticHermiteSpline]:
        """
        Returns a set of quintic splines corresponding to the provided control
        vectors. The user is free to set the direction of all waypoints. Continuous
        curvature is guaranteed throughout the path.
        
        :param controlVectors: The control vectors.
        
        :returns: A vector of quintic hermite splines that interpolate through the
                  provided waypoints.
        """
    @staticmethod
    def quinticSplinesFromWaypoints(waypoints: list[wpimath.geometry._geometry.Pose2d]) -> list[QuinticHermiteSpline]:
        """
        Returns quintic splines from a set of waypoints.
        
        :param waypoints: The waypoints
        
        :returns: List of quintic splines.
        """
    def __init__(self) -> None:
        ...
class SplineParameterizer:
    """
    Class used to parameterize a spline by its arc length.
    """
    @staticmethod
    @typing.overload
    def parameterize(spline: Spline3, t0: float = 0.0, t1: float = 1.0) -> list[tuple[wpimath.geometry._geometry.Pose2d, wpimath.units.radians_per_meter]]:
        """
        Parametrizes the spline. This method breaks up the spline into various
        arcs until their dx, dy, and dtheta are within specific tolerances.
        
        :param spline: The spline to parameterize.
        :param t0:     Starting internal spline parameter. It is recommended to leave
                       this as default.
        :param t1:     Ending internal spline parameter. It is recommended to leave this
                       as default.
        
        :returns: A vector of poses and curvatures that represents various points on
                  the spline.
        """
    @staticmethod
    @typing.overload
    def parameterize(spline: Spline5, t0: float = 0.0, t1: float = 1.0) -> list[tuple[wpimath.geometry._geometry.Pose2d, wpimath.units.radians_per_meter]]:
        """
        Parametrizes the spline. This method breaks up the spline into various
        arcs until their dx, dy, and dtheta are within specific tolerances.
        
        :param spline: The spline to parameterize.
        :param t0:     Starting internal spline parameter. It is recommended to leave
                       this as default.
        :param t1:     Ending internal spline parameter. It is recommended to leave this
                       as default.
        
        :returns: A vector of poses and curvatures that represents various points on
                  the spline.
        """
