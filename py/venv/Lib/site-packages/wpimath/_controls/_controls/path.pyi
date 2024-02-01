from __future__ import annotations
import typing
import wpimath.geometry._geometry
__all__ = ['TravelingSalesman']
class TravelingSalesman:
    """
    Given a list of poses, this class finds the shortest possible route that
    visits each pose exactly once and returns to the origin pose.
    
    @see <a
    href="https://en.wikipedia.org/wiki/Travelling_salesman_problem">https://en.wikipedia.org/wiki/Travelling_salesman_problem</a>
    """
    @typing.overload
    def __init__(self) -> None:
        """
        Constructs a traveling salesman problem solver with a cost function defined
        as the 2D distance between poses.
        """
    @typing.overload
    def __init__(self, cost: typing.Callable[[wpimath.geometry._geometry.Pose2d, wpimath.geometry._geometry.Pose2d], float]) -> None:
        """
        Constructs a traveling salesman problem solver with a user-provided cost
        function.
        
        :param cost: Function that returns the cost between two poses. The sum of
                     the costs for every pair of poses is minimized.
        """
    def solve(self, poses: list[wpimath.geometry._geometry.Pose2d], iterations: int) -> list[wpimath.geometry._geometry.Pose2d]:
        """
        Finds the path through every pose that minimizes the cost. The first pose
        in the returned array is the first pose that was passed in.
        
        This overload supports a dynamically-sized list of poses for Python to use.
        
        :param poses:      An array of Pose2ds the path must pass through.
        :param iterations: The number of times the solver attempts to find a better
                           random neighbor.
        
        :returns: The optimized path as an array of Pose2ds.
        """
