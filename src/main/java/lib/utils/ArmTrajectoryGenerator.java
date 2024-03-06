package lib.utils;

import java.util.ArrayList;
import java.util.List;

public class ArmTrajectoryGenerator {

  private static double lerp(double startValue, double endValue, double t) {
    return startValue + (endValue - startValue) * t;
  }

  public static ArmTrajectory generateTrajectory(List<ArmTrajectoryPose> setpoints, ArmTrajectoryConfig config) {
    List<ArmTrajectory.ArmTrajectoryState> m_states = new ArrayList<>();
    // forwards pass for ramp up
    double timestamp = 0.0;
    for (int i = 0; i < setpoints.size() - 1; i++) {
      ArmTrajectoryPose current = setpoints.get(i);
      ArmTrajectoryPose next = setpoints.get(i + 1);

      // look ahead a consistent time step (20 ms) and calculate the state
      ArmTrajectory.ArmTrajectoryState prevState = null;
      while (true) {
        if (prevState == null) {
          // inital state
          prevState = new ArmTrajectory.ArmTrajectoryState(
              timestamp,
              current.armPoseDegs,
              current.armVelocityDegsPerSecond,
              config.armMaxAccelerationDegsPerSecondSq,
              current.wristPoseDegs,
              current.wristVelocityDegsPerSecond,
              config.wristMaxAccelerationDegsPerSecondSq);

          m_states.add(prevState);
        }

        // check to see if we're past the setpoint trying to move to, if we are then continue to the next set of
        // setpoints
        if (((prevState.armPoseDegs > next.armPoseDegs && prevState.armVelocityDegsPerSecond > 0.0)
            || (prevState.armPoseDegs < next.armPoseDegs && prevState.armVelocityDegsPerSecond < 0.0))
            && ((prevState.wristPoseDegs > next.wristPoseDegs && prevState.wristVelocityDegsPerSecond > 0.0)
            || (prevState.wristPoseDegs < next.wristPoseDegs && prevState.wristVelocityDegsPerSecond < 0.0))) {
          break;
        }

        timestamp += 0.02;

        // calculate new joint positions
        // dx = x + vt + 1/2at^2
        double newArmPose = prevState.armPoseDegs +
            (1.0 / 2.0) * config.armMaxAccelerationDegsPerSecondSq * Math.pow(0.02, 2);
        double newWristPose = prevState.wristPoseDegs +
            (1.0 / 2.0) * config.wristMaxAccelerationDegsPerSecondSq * Math.pow(0.02, 2);

        // calculate new joint velocities
        // vf = sqrt(vi^2 + 2at)
        double newArmVel = Math.min(Math.sqrt(Math.pow(prevState.armVelocityDegsPerSecond, 2)
            + 2 * config.armMaxAccelerationDegsPerSecondSq + 0.02),
            config.armMaxVelocityDegsPerSecond);
        double newWristVel = Math.min(Math.sqrt(Math.pow(prevState.wristVelocityDegsPerSecond, 2)
            + 2 * config.wristMaxAccelerationDegsPerSecondSq + 0.02),
            config.wristMaxVelocityDegsPerSecond);

        // create new point at time step
        prevState = new ArmTrajectory.ArmTrajectoryState(
            timestamp,
            newArmPose,
            newArmVel,
            config.armMaxAccelerationDegsPerSecondSq,
            newWristPose,
            newWristVel,
            config.wristMaxAccelerationDegsPerSecondSq);

        m_states.add(prevState);
      }
    }

    return new ArmTrajectory(m_states);
  }

  public record ArmTrajectoryPose(double armPoseDegs, double armVelocityDegsPerSecond,
                                  double wristPoseDegs, double wristVelocityDegsPerSecond) {}

  public record ArmTrajectoryConfig(double armMaxVelocityDegsPerSecond, double armMaxAccelerationDegsPerSecondSq,
                                    double wristMaxVelocityDegsPerSecond, double wristMaxAccelerationDegsPerSecondSq) {}

  public static class ArmTrajectory {
    private final List<ArmTrajectoryState> states;

    public ArmTrajectory(List<ArmTrajectoryState> states) {
      this.states = states;
    }

    public record ArmTrajectoryState(double timeSeconds,
                                     double armPoseDegs,
                                     double armVelocityDegsPerSecond,
                                     double armAccelerationDegsPerSecondSq,
                                     double wristPoseDegs,
                                     double wristVelocityDegsPerSecond,
                                     double wristAccelerationDegsPerSecondSq) {
      ArmTrajectoryState interpolate(ArmTrajectoryState endValue, double i) {
        // Find the new t value.
        final double newT = lerp(timeSeconds, endValue.timeSeconds, i);

        // Find the delta time between the current state and the interpolated state.
        final double deltaT = newT - timeSeconds;

        // If delta time is negative, flip the order of interpolation.
        if (deltaT < 0) {
          return endValue.interpolate(this, 1 - i);
        }

        // run interpolation on arm positions and velocities
        // Check whether the robot is reversing at this stage.
        final boolean armReversing = armVelocityDegsPerSecond < 0;

        // Calculate the new velocity
        // v_f = v_0 + at
        final double newArmV = armVelocityDegsPerSecond + (armAccelerationDegsPerSecondSq * deltaT);

        // Calculate the change in position.
        // delta_s = v_0 t + 0.5at²
        final double newArmS =
                (armVelocityDegsPerSecond * deltaT
                        + 0.5 * armAccelerationDegsPerSecondSq * Math.pow(deltaT, 2))
                        * (armReversing ? -1.0 : 1.0);

        // To find the new position for the new state, we need
        // to interpolate between the two endpoint poses. The fraction for
        // interpolation is the change in position (delta s) divided by the total
        // distance between the two endpoints.
        final double armInterpolationFrac =
                newArmS / endValue.armPoseDegs() - armPoseDegs;

        // run interpolation on wrist positions and velocities
        // Check whether the robot is reversing at this stage.
        final boolean wristReversing = wristVelocityDegsPerSecond < 0;

        // Calculate the new velocity
        // v_f = v_0 + at
        final double newWristV = wristVelocityDegsPerSecond + (wristAccelerationDegsPerSecondSq * deltaT);

        // Calculate the change in position.
        // delta_s = v_0 t + 0.5at²
        final double newWristS =
                (wristVelocityDegsPerSecond * deltaT
                        + 0.5 * wristAccelerationDegsPerSecondSq * Math.pow(deltaT, 2))
                        * (wristReversing ? -1.0 : 1.0);

        // Return the new state. To find the new position for the new state, we need
        // to interpolate between the two endpoint poses. The fraction for
        // interpolation is the change in position (delta s) divided by the total
        // distance between the two endpoints.
        final double wristInterpolationFrac =
                newWristS / endValue.armPoseDegs() - wristPoseDegs;

        return new ArmTrajectoryState(
                newT,
                lerp(armPoseDegs, endValue.armPoseDegs, armInterpolationFrac),
                newArmV,
                armAccelerationDegsPerSecondSq,
                lerp(wristPoseDegs, endValue.wristPoseDegs, wristInterpolationFrac),
                newWristV,
                wristAccelerationDegsPerSecondSq);
      }
    }
  }
}
