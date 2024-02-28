// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.subsystems.drive;

import com.gos.lib.properties.pid.PidProperty;
import com.gos.lib.properties.pid.WpiPidPropertyBuilder;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.drive.module.Module;
import frc.robot.subsystems.drive.module.ModuleIO;
import frc.robot.subsystems.vision.VisionSubsystem;
import frc.robot.util.LocalADStarAK;

import java.util.List;
import java.util.Optional;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;

import frc.robot.util.PoseEstimator;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class DriveSubsystem extends SubsystemBase {

  public static final Lock odometryLock = new ReentrantLock();
  private final GyroIO gyroIO;
  private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
  private final Module[] modules = new Module[4]; // FL, FR, BL, BR

  private final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(getModuleTranslations());
  private final PoseEstimator m_poseEstimator;
  private Pose2d pose = new Pose2d();
  private Rotation2d lastGyroRotation = new Rotation2d();

  private final VisionSubsystem[] m_cameras;

  private final PIDController m_thetaPid;
  private final PidProperty m_thetaPidProperty;

  public DriveSubsystem(
      GyroIO gyroIO,
      ModuleIO flModuleIO,
      ModuleIO frModuleIO,
      ModuleIO blModuleIO,
      ModuleIO brModuleIO) {
    this (gyroIO,
        flModuleIO,
        frModuleIO,
        blModuleIO,
        brModuleIO,
        new VisionSubsystem[]{});
  }

  public DriveSubsystem(
      GyroIO gyroIO,
      ModuleIO flModuleIO,
      ModuleIO frModuleIO,
      ModuleIO blModuleIO,
      ModuleIO brModuleIO,
      VisionSubsystem[] cameras) {
    this.gyroIO = gyroIO;
    modules[0] = new Module(flModuleIO);
    modules[1] = new Module(frModuleIO);
    modules[2] = new Module(blModuleIO);
    modules[3] = new Module(brModuleIO);

    m_thetaPid = new PIDController(0.0, 0.0, 0.0);
    m_thetaPid.enableContinuousInput(-Math.PI, Math.PI);
    m_thetaPidProperty = new WpiPidPropertyBuilder("Drive/Theta Alignment", false, m_thetaPid)
        .addP(0.5)
        .addI(0.0)
        .addD(0.0)
        .build();

    m_cameras = cameras;
    m_poseEstimator =
        new PoseEstimator(VecBuilder.fill(
            Units.inchesToMeters(1.0),
            Units.inchesToMeters(1.0),
            Units.degreesToRadians(30)));

    m_poseEstimator.resetPose(new Pose2d());

    // Configure AutoBuilder for PathPlanner
    AutoBuilder.configureHolonomic(
        this::getVisionPose,
        this::setPose,
        () -> kinematics.toChassisSpeeds(getModuleStates()),
        this::runVelocity,
        DriveConstants.HOLONOMIC_CONFIG,
        () ->
            DriverStation.getAlliance().isPresent()
                && DriverStation.getAlliance().get() == Alliance.Red,
        this);
    Pathfinding.setPathfinder(new LocalADStarAK());
    PathPlannerLogging.setLogActivePathCallback(
        (List<Pose2d> activePath) -> Logger.recordOutput(
            "Odometry/Trajectory", activePath.toArray(new Pose2d[0])));
    PathPlannerLogging.setLogTargetPoseCallback(
        (Pose2d targetPose) -> Logger.recordOutput("Odometry/TrajectorySetpoint", targetPose));
  }

  @Override
  public void periodic() {
    odometryLock.lock(); // Prevents odometry updates while reading data
    gyroIO.updateInputs(gyroInputs);
    for (var module : modules) {
      module.updateInputs();
    }
    odometryLock.unlock();
    Logger.processInputs("Drive/Gyro", gyroInputs);
    for (var module : modules) {
      module.periodic();
    }

    // Stop moving when disabled
    if (DriverStation.isDisabled()) {
      for (var module : modules) {
        module.stop();
      }
    }
    // Log empty setpoint states when disabled
    if (DriverStation.isDisabled()) {
      Logger.recordOutput("SwerveStates/Setpoints");
      Logger.recordOutput("SwerveStates/SetpointsOptimized");
    }

    // Update odometry
    int deltaCount =
        gyroInputs.isConnected() ? gyroInputs.getOdometryYawPositions().length : Integer.MAX_VALUE;
    for (int i = 0; i < 4; i++) {
      deltaCount = Math.min(deltaCount, modules[i].getPositionDeltas().length);
    }
    Logger.recordOutput("Odometry/Delta Count", deltaCount);
    for (int deltaIndex = 0; deltaIndex < deltaCount; deltaIndex++) {
      // Read wheel deltas from each module
      SwerveModulePosition[] wheelDeltas = new SwerveModulePosition[4];
      for (int moduleIndex = 0; moduleIndex < 4; moduleIndex++) {
        wheelDeltas[moduleIndex] = modules[moduleIndex].getPositionDeltas()[deltaIndex];
      }

      // The twist represents the motion of the robot since the last
      // sample in x, y, and theta based only on the modules, without
      // the gyro. The gyro is always disconnected in simulation.
      var twist = kinematics.toTwist2d(wheelDeltas);
      if (gyroInputs.isConnected()) {
        // If the gyro is connected, replace the theta component of the twist
        // with the change in angle since the last sample.
        Rotation2d gyroRotation = gyroInputs.getOdometryYawPositions()[deltaIndex];
        twist = new Twist2d(twist.dx, twist.dy, gyroRotation.minus(lastGyroRotation).getRadians());
        lastGyroRotation = gyroRotation;
      }
      Logger.recordOutput("Odometry/Twist", twist);
      // Apply the twist (change since last sample) to the current pose
      m_poseEstimator.addDriveData(Timer.getFPGATimestamp(), twist);
      pose = pose.exp(twist);
    }

    List<PoseEstimator.TimestampedVisionUpdate> visionUpdates = new java.util.ArrayList<>(List.of());
    for (VisionSubsystem camera : m_cameras) {
      camera.updateInputs();
      Optional<PoseEstimator.TimestampedVisionUpdate> update = camera.getPose(m_poseEstimator.getLatestPose());
      update.ifPresent(visionUpdates::add);
    }

//    m_poseEstimator.addVisionData(visionUpdates);
  }

  /**
   * Runs the drive at the desired velocity.
   *
   * @param speeds Speeds in meters/sec
   */
  public void runVelocity(ChassisSpeeds speeds) {
    // Calculate module setpoints
    ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(speeds, 0.02);
    SwerveModuleState[] setpointStates = kinematics.toSwerveModuleStates(discreteSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(setpointStates, DriveConstants.MAX_LINEAR_SPEED);

    // Send setpoints to modules
    SwerveModuleState[] optimizedSetpointStates = new SwerveModuleState[4];
    for (int i = 0; i < 4; i++) {
      // The module returns the optimized state, useful for logging
      optimizedSetpointStates[i] = modules[i].runSetpoint(setpointStates[i]);
    }

    // Log setpoint states
    Logger.recordOutput("SwerveStates/Setpoints", setpointStates);
    Logger.recordOutput("SwerveStates/SetpointsOptimized", optimizedSetpointStates);
  }

  /**
   * Calculates theta output to align to an arbitrary point
   *
   * @param point the desired point to align to
   */
  public double alignToPoint(Pose3d point) {
    Transform3d robotToPoint = new Transform3d(new Pose3d(pose), point);
    double desiredRotation = Math.atan2(robotToPoint.getX(), robotToPoint.getY());
    return m_thetaPid.calculate(getRotation().getRadians(), desiredRotation);
  }

  /** Stops the drive. */
  public void stop() {
    runVelocity(new ChassisSpeeds());
  }

  /**
   * Stops the drive and turns the modules to an X arrangement to resist movement. The modules will
   * return to their normal orientations the next time a nonzero velocity is requested.
   */
  public void stopWithX() {
    Rotation2d[] headings = new Rotation2d[4];
    for (int i = 0; i < 4; i++) {
      headings[i] = getModuleTranslations()[i].getAngle();
    }
    kinematics.resetHeadings(headings);
    stop();
  }

  /** Runs forwards at the commanded voltage. */
  public void runCharacterizationVolts(double volts) {
    for (int i = 0; i < 4; i++) {
      modules[i].runCharacterization(volts);
    }
  }

  /** Returns the average drive velocity in radians/sec. */
  public double getCharacterizationVelocity() {
    double driveVelocityAverage = 0.0;
    for (var module : modules) {
      driveVelocityAverage += module.getCharacterizationVelocity();
    }
    return driveVelocityAverage / 4.0;
  }

  /** Returns the module states (turn angles and drive velocities) for all the modules. */
  @AutoLogOutput(key = "SwerveStates/Measured")
  private SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (int i = 0; i < 4; i++) {
      states[i] = modules[i].getState();
    }
    return states;
  }

  /** Returns the current odometry pose. */
  @AutoLogOutput(key = "Odometry/Robot")
  public Pose2d getPose() {
    return pose;
  }

  /** Returns the robot pose with vision updates */
  @AutoLogOutput(key = "Odometry/RobotVision")
  public Pose2d getVisionPose() {
    return m_poseEstimator.getLatestPose();
  }

  /** Returns the current odometry rotation. */
  public Rotation2d getRotation() {
    return m_poseEstimator.getLatestPose().getRotation();
  }

  /** Resets the current odometry pose. */
  public void setPose(Pose2d pose) {
    m_poseEstimator.resetPose(pose);
  }

  /** Returns the maximum linear speed in meters per sec. */
  public double getMaxLinearSpeedMetersPerSec() {
    return DriveConstants.MAX_LINEAR_SPEED;
  }

  /** Returns the maximum angular speed in radians per sec. */
  public double getMaxAngularSpeedRadPerSec() {
    return DriveConstants.MAX_ANGULAR_SPEED;
  }

  public Command pathfollowFactory(Pose2d pose) {
    return AutoBuilder.pathfindToPose(pose, DriveConstants.DEFAULT_CONSTRAINTS);
  }

  /** Returns an array of module translations. */
  public static Translation2d[] getModuleTranslations() {
    return new Translation2d[] {
      new Translation2d(DriveConstants.TRACK_WIDTH_X / 2.0, DriveConstants.TRACK_WIDTH_Y / 2.0),
      new Translation2d(DriveConstants.TRACK_WIDTH_X / 2.0, -DriveConstants.TRACK_WIDTH_Y / 2.0),
      new Translation2d(-DriveConstants.TRACK_WIDTH_X / 2.0, DriveConstants.TRACK_WIDTH_Y / 2.0),
      new Translation2d(-DriveConstants.TRACK_WIDTH_X / 2.0, -DriveConstants.TRACK_WIDTH_Y / 2.0)
    };
  }
}
