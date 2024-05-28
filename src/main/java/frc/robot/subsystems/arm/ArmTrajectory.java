package frc.robot.subsystems.arm;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.Num;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.numbers.N4;
import org.ejml.simple.SimpleMatrix;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

// Cubic spline trajectories
public class ArmTrajectory {
  private double[] m_times;
  private ArmTrajectoryState[] m_states;
  private double m_startTime;
  private double m_finalTime;

  public record ArmTrajectoryState(
          double wristPositionDegs,
          double wristVelocityDegsPerSec,
          double armPositionDegs,
          double armVelocityDegsPerSec
  ) {}

  public ArmTrajectory(double[] times, ArmTrajectoryState[] states) {
    if (times.length != states.length) {
      throw new IllegalStateException("Number of states must match number of times");
    }

    m_times = times;
    m_states = states;
    m_startTime = m_times[0];
    m_finalTime = m_times[m_times.length - 1];
  }

  public double clipTime(double time) {
    /*
    Limits a given time between the trajectories start and end time
    */

    return MathUtil.clamp(time, m_startTime, m_finalTime);
  }

  public ArmTrajectoryState sample(double desiredTime) {
    /*
    Samples the trajectory at a given time.

    Linearly interpolates between trajectory samples.
    */

    // finding the indexes of the next and last closest times
    double time = clipTime(desiredTime);
    int nextIdx = nextTimeIdx(m_times, time);
    int prevIdx = prevTimeIdx(m_times, time);

    if (prevIdx == nextIdx) {
      return m_states[prevIdx];
    }

    // find the next and last closest states and times
    ArmTrajectoryState prevState = m_states[prevIdx];
    ArmTrajectoryState nextState = m_states[nextIdx];
    double prevTime = m_times[prevIdx];
    double nextTime = m_times[nextIdx];

    // lots of math to create lots of terms ;w;
    double newArmPosition =
            prevState.armPositionDegs + (nextState.armPositionDegs - prevState.armPositionDegs)
                    / (nextTime - prevTime) * (time - prevTime);

    double newArmVelocity =
            prevState.armVelocityDegsPerSec + (nextState.armVelocityDegsPerSec - prevState.armVelocityDegsPerSec)
                    / (nextTime - prevTime) * (time - prevTime);

    double newWristPosition =
            prevState.wristPositionDegs + (nextState.wristPositionDegs - prevState.wristPositionDegs)
                    / (nextTime - prevTime) * (time - prevTime);

    double newWristVelocity =
            prevState.wristVelocityDegsPerSec + (nextState.wristVelocityDegsPerSec - prevState.wristVelocityDegsPerSec)
                    / (nextTime - prevTime) * (time - prevTime);

    return new ArmTrajectoryState(newWristPosition, newWristVelocity, newArmPosition, newArmVelocity);
  }

  public static ArmTrajectory fromCoeffs(Matrix<N4, N2> coeffs, double t_0, double t_f) {
    /*
    Generate a trajectory from a polynomial coefficients matrix.

    Keyword arguments:
    coeffs -- Polynomial coefficients as columns in increasing order.
              Can have arbitrarily many columns.
    t_0 -- time to start the interpolation
    t_f -- time to end the interpolation

    This will only create a quadratic function, it will not create a quintic one.

    Returns:
    Trajectory following the interpolation. The states will be in the form:
    [pos_1, ..., pos_n, vel_1, ..., vel_n]
    Where n is the number of columns in coeffs.
     */

    // create an array of timestamps to sample from
    ArrayList<Double> tList = new ArrayList<>();
    double x = t_0;
    while (x < t_f) {
      tList.add(x);
      x += 100;
    }
    tList.add(t_f);

    double[] t = tList.stream().mapToDouble(Double::valueOf).toArray();
  }

  public static Matrix<N4, N2> cubic_interpolation(
          double t_0,
          double t_f,
          ArmTrajectoryState state_0,
          ArmTrajectoryState state_f) {
    /*
    Perform cubic interpolation between state₀ at t = t₀ and state_f at t = t_f.
    Solves using the matrix equation:

      [1  t_0   t_0²   t_0³][c₀₁  c₀₂]   [x_0₁  x_0₂]
      [0  1    2t_0   3t_0²][c₁₁  c₁₂] = [v_0₁  v_0₂]
      [1  t_f   t_f²   t_f³][c₂₁  c₂₂]   [x_f₁  x_f₂]
      [0  1    2t_f   3t_f²][c₃₁  c₃₂]   [v_f₁  v_f₂]

    To find the cubic polynomials:

      x₁(t) = c₀₁ + c₁₁t + c₂₁t² + c₃₁t³
      x₂(t) = c₀₂ + c₁₂t + c₂₂t² + c₃₂t³

    where x₁ is the first joint position and x₂ is the second joint position,
    such that the arm is in state_0 [x_0₁, x_0₂, v_0₁, v_0₂]ᵀ at t_0 and state_f
    [x_f₁, x_f₂, v_f₁, v_f₂]ᵀ at t_f.

    Make sure to only use the interpolated cubic for t between t_0 and t_f.

    Keyword arguments:
    t_0 -- start time of interpolation
    t_f -- end time of interpolation
    state_0 -- start state [θ₁, θ₂, ω₁, ω₂]ᵀ
    state_f -- end state [θ₁, θ₂, ω₁, ω₂]ᵀ

    Returns:
    coeffs -- 4x2 matrix containing the interpolation coefficients for joint 1
              in column 1 and joint 2 in column 2
    */

    // copy our states into the "output" matrix
    Matrix<N4, N2> rhs = new Matrix<>(new SimpleMatrix(new double[][]{
            {state_0.armPositionDegs, state_0.armVelocityDegsPerSec, state_f.armPositionDegs, state_0.armVelocityDegsPerSec},
            {state_0.wristPositionDegs, state_0.wristVelocityDegsPerSec, state_f.wristPositionDegs, state_0.wristVelocityDegsPerSec}
    }));

    SimpleMatrix lhsSimple = SimpleMatrix.filled(4, 4, 0.0);
    lhsSimple.setRow(0, 0, posRow(t_0));
    lhsSimple.setRow(0, 0, velRow(t_0));
    lhsSimple.setRow(0, 0, posRow(t_f));
    lhsSimple.setRow(0, 0, velRow(t_f));

    Matrix<N4, N4> lhs = new Matrix<>(lhsSimple);

    return lhs.inv().times(rhs);
  }

  private static double[] posRow(double t) {
    return new double[]{1, t, t * t, t * t * t};
  }

  private static double[] velRow(double t) {
    return new double[]{0, 1, 2 * t, 3 * t * t};
  }

  private static int nextTimeIdx(double[] array, double time) {
    int returnvalue = -1;
    for (int i = 0; i < array.length; ++i) {
      if (array[i] >= time) {
        returnvalue = i;
        break;
      }
    }
    return returnvalue;
  }

  private static int prevTimeIdx(double[] array, double time) {
    int returnvalue = -1;
    for (int i = 0; i < array.length; ++i) {
      if (array[i] <= time) {
        returnvalue = i;
      }
    }
    return returnvalue;
  }
}
