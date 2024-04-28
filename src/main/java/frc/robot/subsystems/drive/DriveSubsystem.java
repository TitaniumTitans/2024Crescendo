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

import com.ctre.phoenix6.SignalLogger;
import com.gos.lib.properties.pid.PidProperty;
import com.gos.lib.properties.pid.WpiPidPropertyBuilder;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.PathPlannerLogging;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.drive.module.Module;
import frc.robot.subsystems.drive.module.ModuleIO;
import frc.robot.subsystems.drive.module.PhoenixOdometryThread;
import frc.robot.subsystems.vision.VisionSubsystem;
import lib.utils.*;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardBoolean;

import java.util.Arrays;
import java.util.List;
import java.util.Queue;
import java.util.concurrent.ArrayBlockingQueue;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;

import static edu.wpi.first.units.Units.Volts;

public class DriveSubsystem extends SubsystemBase {

  private final GyroIO gyroIO;
  private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
  private final Module[] modules = new Module[4]; // FL, FR, BL, BR

  private final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(getModuleTranslations());

  private final SwerveDrivePoseEstimator m_wpiPoseEstimator;
  private final SwerveDrivePoseEstimator m_wheelOnlyPoseEstimator;

  private final Field2d m_field = new Field2d();

  public static final Lock odometryLock = new ReentrantLock();
  public static final Queue<Double> timestampQueue = new ArrayBlockingQueue<>(20);

  private Rotation2d m_rawGyroRotation = new Rotation2d();
  private SwerveModulePosition[] m_lastModulePositions = new SwerveModulePosition[] {
      new SwerveModulePosition(),
      new SwerveModulePosition(),
      new SwerveModulePosition(),
      new SwerveModulePosition()
  };

  private final VisionSubsystem[] m_cameras;

  // heading controllers
  private final PIDController m_thetaPid;
  private final PidProperty m_thetaPidProperty;

  private final PIDController m_largeThetaPid;

  private final SysIdRoutine m_sysId;

  // used to log and control module setpoints
  private final SwerveModuleState[] m_optimizedStates = new SwerveModuleState[] {
      new SwerveModuleState(),
      new SwerveModuleState(),
      new SwerveModuleState(),
      new SwerveModuleState()
  };

  // field relative speeds used for shoot on move logic
  private FieldRelativeSpeed m_fieldRelVel = new FieldRelativeSpeed();
  private FieldRelativeSpeed m_lastFieldRelVel = new FieldRelativeSpeed();
  private FieldRelativeAccel m_fieldRelAccel = new FieldRelativeAccel();

  // Manual mode selector
  private final LoggedDashboardBoolean m_useAutoCrap = new LoggedDashboardBoolean("Use Auto Crap?", true);

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

    PhoenixOdometryThread.getInstance().start();

    m_thetaPid = new PIDController(0.0, 0.0, 0.0);
    m_thetaPid.enableContinuousInput(0, 360);
    m_thetaPid.setTolerance(5.0);

    m_thetaPidProperty = new WpiPidPropertyBuilder("Drive/Theta Alignment", false, m_thetaPid)
        .addP(0.02)
        .addI(0.01)
        .addD(0.004)
        .build();

    // 0.04, 0.01, 0.0

    m_largeThetaPid = new PIDController(0.04, 0, 0.002);
    m_largeThetaPid.setTolerance(15);
    m_largeThetaPid.enableContinuousInput(0, 360);

    m_cameras = cameras;

    m_wpiPoseEstimator = new SwerveDrivePoseEstimator(
        kinematics,
        new Rotation2d(),
        getModulePositions(),
        new Pose2d(),
        VecBuilder.fill(
            Units.inchesToMeters(0.5),
            Units.inchesToMeters(0.5),
            Units.degreesToRadians(0.75)),
        VecBuilder.fill(
            Units.inchesToMeters(4.5),
            Units.inchesToMeters(4.5),
            Units.degreesToRadians(12.5))
    );

    m_wheelOnlyPoseEstimator = new SwerveDrivePoseEstimator(
        kinematics,
        new Rotation2d(),
        getModulePositions(),
        new Pose2d(),
        VecBuilder.fill(
            Units.inchesToMeters(0.5),
            Units.inchesToMeters(0.5),
            Units.degreesToRadians(0.75)),
        VecBuilder.fill(
            Units.inchesToMeters(5.0),
            Units.inchesToMeters(5.0),
            Units.degreesToRadians(30.0))
    );

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

    // turn on logging
    SmartDashboard.putData("Field",m_field);

    m_sysId = new SysIdRoutine(
            new SysIdRoutine.Config(
                    null,
                    null,
                    null,
                    (SysIdRoutineLog.State state) -> SignalLogger.writeString("Drive/SysidState", state.toString())
            ),
            new SysIdRoutine.Mechanism(
                    (Measure<Voltage> volts) -> runCharacterizationVolts(volts.in(Volts)),
                    null,
                    this
            )
    );
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
    for (VisionSubsystem camera : m_cameras) {
      // make sure we're not moving too fast before trying to update vision poses
      if ((MathUtil.applyDeadband(kinematics.toChassisSpeeds(getModuleStates()).vxMetersPerSecond,
          Units.inchesToMeters(6.0)) == 0.0)
          && (MathUtil.applyDeadband(kinematics.toChassisSpeeds(getModuleStates()).vyMetersPerSecond,
          Units.inchesToMeters(6.0)) == 0.0)
          && (MathUtil.applyDeadband(kinematics.toChassisSpeeds(getModuleStates()).omegaRadiansPerSecond,
          Units.degreesToRadians(7.5)) == 0.0)
          || !DriverStation.isAutonomousEnabled()) {

        camera.getPose(m_wpiPoseEstimator.getEstimatedPosition()).ifPresent(
            (PoseEstimator.TimestampedVisionUpdate pose) ->
                m_wpiPoseEstimator.addVisionMeasurement(pose.pose(), pose.timestamp(), pose.stdDevs())
        );
      }
      camera.updateInputs();
    }

    m_wpiPoseEstimator.update(gyroInputs.yawPosition, getModulePositions());
    m_wheelOnlyPoseEstimator.update(gyroInputs.yawPosition, getModulePositions());
    m_thetaPidProperty.updateIfChanged();

    // log field positioning data
    Logger.recordOutput("Odometry/Wheel Only Odometry", m_wheelOnlyPoseEstimator.getEstimatedPosition());
    Logger.recordOutput("Drive/DistanceToTarget",
        Units.metersToInches(AimbotUtils.getDistanceFromSpeaker(getVisionPose())));
    Logger.recordOutput("Drive/AngleToTarget",
        -AimbotUtils.getDrivebaseAimingAngle(getVisionPose()).getDegrees());

    m_field.setRobotPose(getVisionPose());

    // calculate field relative velocities and acceleration
    m_fieldRelVel = new FieldRelativeSpeed(kinematics.toChassisSpeeds(getModuleStates()), getRotation());
    m_fieldRelAccel = new FieldRelativeAccel(m_fieldRelVel, m_lastFieldRelVel, 0.02);
    m_lastFieldRelVel = m_fieldRelVel;
  }

  /**
   * Runs the drive at the desired velocity.
   *
   * @param speeds Speeds in meters/sec
   */
  public void runVelocity(ChassisSpeeds speeds) {
    // Calculate module setpoints
    SwerveModuleState[] setpointStates = kinematics.toSwerveModuleStates(speeds);

    // Send setpoints to modules
    for (int i = 0; i < 4; i++) {
      // The module returns the optimized state, useful for logging
      m_optimizedStates[i] = modules[i].runSetpoint(setpointStates[i]);
    }

    Logger.recordOutput("SwerveStates/Setpoints", setpointStates);
    Logger.recordOutput("SwerveStates/Optimized", m_optimizedStates);

    Logger.recordOutput("Drive/Desired Linear Velocity FPS",
        Units.metersToFeet(Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond)));
  }

  public void davidDrive(double xVel, double yVel, double angle) {
    Rotation2d heading;

    // if red change heading goal
    if (DriverStation.getAlliance().isPresent()
            && DriverStation.getAlliance().get() == DriverStation.Alliance.Red) {
      heading = getRotation().plus(Rotation2d.fromDegrees(180));
    } else {
      heading = getRotation();
    }

    double steerVelocity = alignToAngle(Rotation2d.fromDegrees(angle));
    ChassisSpeeds speeds =
        ChassisSpeeds.fromFieldRelativeSpeeds(
            xVel,
            yVel,
            steerVelocity,
            heading);

    runVelocity(speeds);
  }

    /**
     * Calculates theta output to align to an arbitrary angle
     *
     * @param angle the desired angle to hold relative to the field
     */
  public double alignToAngle(Rotation2d angle) {
    double currentAngle = getVisionPose().getRotation().getDegrees();
    double desiredAngle = angle.getDegrees();
    double outputDegsPerSec = m_thetaPid.calculate(currentAngle, desiredAngle);
    Logger.recordOutput("Drive/Theta Output DegsS", outputDegsPerSec);

    double cubicOutput = Math.pow(m_thetaPid.getPositionError(), 3.0) * 0.006;
    Logger.recordOutput("Drive/Theta Cubic Output", cubicOutput);

    outputDegsPerSec = outputDegsPerSec + cubicOutput;
    // apply deadband, wpilib version borked?
    if (0.5 > outputDegsPerSec && outputDegsPerSec > -0.5) {
      outputDegsPerSec = 0.0;
    }

    Logger.recordOutput("Drive/Final Theta Output", outputDegsPerSec);

    return Units.degreesToRadians(outputDegsPerSec);
  }

  @AutoLogOutput(key = "Drive/Theta Error")
  public double getThetaError() {
    return m_thetaPid.getPositionError();
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

  /** Returns whether there are connected april tag cameras */
  public boolean useAutoControl() {
    return Arrays.stream(m_cameras).anyMatch((VisionSubsystem::apriltagConnected))
        && m_useAutoCrap.get();
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

  private SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition[] positions = new SwerveModulePosition[4];
    for (int i = 0; i < 4; i++) {
      positions[i] = modules[i].getPosition();
    }
    return positions;
  }

  public Pose2d getPose() {
    return m_wheelOnlyPoseEstimator.getEstimatedPosition();
  }

  /** Returns the robot pose with vision updates */
  @AutoLogOutput(key = "Odometry/RobotPose")
  public Pose2d getVisionPose() {
    return m_wpiPoseEstimator.getEstimatedPosition();
  }

  /** Returns the current odometry rotation. */
  public Rotation2d getRotation() {
    return m_wpiPoseEstimator.getEstimatedPosition().getRotation();
  }

  public Rotation2d getGyroRotation() {
    return gyroInputs.yawPosition;
  }

  /** Returns the robot's field relative velocity, used for shoot on move */
  public FieldRelativeSpeed getFieldRelativeVelocity() {
    return m_fieldRelVel;
  }

  /** Returns the robot's field relative acceleration, used for shoot on move */
  public FieldRelativeAccel getFieldRelativeAcceleration() {
    return m_fieldRelAccel;
  }

  /** Resets the current odometry pose. */
  public void setPose(Pose2d pose) {
    gyroIO.resetGyro(pose.getRotation().getDegrees());
    m_wpiPoseEstimator.resetPosition(new Rotation2d(), getModulePositions(), pose);
    m_wheelOnlyPoseEstimator.resetPosition(new Rotation2d(), getModulePositions(), pose);
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
    return AutoBuilder.pathfindToPoseFlipped(
        pose, DriveConstants.DEFAULT_CONSTRAINTS).withInterruptBehavior(Command.InterruptionBehavior.kCancelSelf)
        .unless(() -> !useAutoControl());
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

  public Command runSysidQuasistatic(SysIdRoutine.Direction direction) {
    return m_sysId.quasistatic(direction);
  }

  public Command runSysidDynamic(SysIdRoutine.Direction direction) {
    return m_sysId.dynamic(direction);
  }

  public ChassisSpeeds getRobotRelativeSpeeds() {
    return kinematics.toChassisSpeeds(getModuleStates());
  }
}
