// Copyright (c) 2023 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

// originally from https://github.com/Mechanical-Advantage/RobotCode2023

package frc.robot.commands;

import static frc.robot.Constants.*;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.team3061.RobotConfig;
import frc.lib.team3061.drivetrain.Drivetrain;
import frc.lib.team6328.util.TunableNumber;
import frc.robot.limelightVision.ApriltagVision.VisionApriltagSubsystem;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

/**
 * This command, when executed, instructs the drivetrain subsystem to drive to the specified pose in
 * a straight line. The execute method invokes the drivetrain subsystem's drive method. For
 * following a predetermined path, refer to the FollowPath Command class. For generating a path on
 * the fly and following that path, refer to the MoveToPose Command class.
 *
 * <p>Requires: the Drivetrain subsystem
 *
 * <p>Finished When: the robot is at the specified pose (within the specified tolerances)
 *
 * <p>At End: stops the drivetrain
 */
public class DriveToPose extends Command {
  private final Drivetrain drivetrain;
  private final Supplier<Pose2d> poseSupplier;
  private final Supplier<Boolean> hasTargetSupplier;
  private final DoubleSupplier translationXSupplier;
  private final DoubleSupplier translationYSupplier;
  private final DoubleSupplier rotationSupplier;
  private final VisionApriltagSubsystem visionApriltagSubsystem;
  private Pose2d targetPose;
  private int driveToPoseState = 0;
  private boolean poseSet;

  private boolean running = false;
  private Timer timer;
  private Alliance alliance;

  private static final TunableNumber driveKp = new TunableNumber("DriveToPose/DriveKp", 4);
  private static final TunableNumber driveKd = new TunableNumber("DriveToPose/DriveKd", 0);
  private static final TunableNumber driveKi = new TunableNumber("DriveToPose/DriveKi", 0);
  private static final TunableNumber thetaKp = new TunableNumber("DriveToPose/ThetaKp", 7);
  private static final TunableNumber thetaKd = new TunableNumber("DriveToPose/ThetaKd", 0);
  private static final TunableNumber thetaKi = new TunableNumber("DriveToPose/ThetaKi", 0);
  private static final TunableNumber driveMaxVelocity =
      new TunableNumber(
          "DriveToPose/DriveMaxVelocity",
          RobotConfig.getInstance().getDriveToPoseDriveMaxVelocity());
  private static final TunableNumber driveMaxAcceleration =
      new TunableNumber(
          "DriveToPose/DriveMaxAcceleration",
          RobotConfig.getInstance().getDriveToPoseDriveMaxAcceleration());
  private static final TunableNumber thetaMaxVelocity =
      new TunableNumber(
          "DriveToPose/ThetaMaxVelocity",
          RobotConfig.getInstance().getDriveToPoseTurnMaxVelocity());
  private static final TunableNumber thetaMaxAcceleration =
      new TunableNumber(
          "DriveToPose/ThetaMaxAcceleration",
          RobotConfig.getInstance().getDriveToPoseTurnMaxAcceleration());
  private static final TunableNumber driveTolerance =
      new TunableNumber(
          "DriveToPose/DriveTolerance", RobotConfig.getInstance().getDriveToPoseDriveTolerance());
  private static final TunableNumber thetaTolerance =
      new TunableNumber(
          "DriveToPose/ThetaTolerance", RobotConfig.getInstance().getDriveToPoseThetaTolerance());
  private static final TunableNumber timeout = new TunableNumber("DriveToPose/timeout", 2.0);

  private final ProfiledPIDController xController =
      new ProfiledPIDController(
          driveKp.get(),
          driveKi.get(),
          driveKd.get(),
          new TrapezoidProfile.Constraints(driveMaxVelocity.get(), driveMaxAcceleration.get()),
          LOOP_PERIOD_SECS);
  private final ProfiledPIDController yController =
      new ProfiledPIDController(
          driveKp.get(),
          driveKi.get(),
          driveKd.get(),
          new TrapezoidProfile.Constraints(driveMaxVelocity.get(), driveMaxAcceleration.get()),
          LOOP_PERIOD_SECS);
  private final ProfiledPIDController thetaController =
      new ProfiledPIDController(
          thetaKp.get(),
          thetaKi.get(),
          thetaKd.get(),
          new TrapezoidProfile.Constraints(thetaMaxVelocity.get(), thetaMaxAcceleration.get()),
          LOOP_PERIOD_SECS);

  /**
   * Constructs a new DriveToPose command that drives the robot in a straight line to the specified
   * pose. A pose supplier is specified instead of a pose since the target pose may not be known
   * when this command is created.
   *
   * @param drivetrain the drivetrain subsystem required by this command
   * @param poseSupplier a supplier that returns the pose to drive to
   */
  public DriveToPose(
      Drivetrain drivetrain,
      DoubleSupplier translationXSupplier,
      DoubleSupplier translationYSupplier,
      DoubleSupplier rotationSupplier,
      VisionApriltagSubsystem visionApriltagSubsystem,
      Supplier<Pose2d> poseSupplier,
      Supplier<Boolean> hasTargetSupplier) {
    this.drivetrain = drivetrain;
    this.translationXSupplier = translationXSupplier;
    this.translationYSupplier = translationYSupplier;
    this.rotationSupplier = rotationSupplier;
    this.poseSupplier = poseSupplier;
    this.hasTargetSupplier = hasTargetSupplier;
    this.visionApriltagSubsystem = visionApriltagSubsystem;
    this.timer = new Timer();
    addRequirements(drivetrain);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);
  }

  /**
   * This method is invoked once when this command is scheduled. It resets all the PID controllers
   * and initializes the current and target poses. It is critical that this initialization occurs in
   * this method and not the constructor as this object is constructed well before the command is
   * scheduled and the robot's pose will definitely have changed and the target pose may not be
   * known until this command is scheduled.
   */
  @Override
  public void initialize() {
    Logger.recordOutput("ActiveCommands/DriveToPose", true);

    // Reset all controllers
    Pose2d currentPose = drivetrain.getPose();
    xController.reset(currentPose.getX());
    yController.reset(currentPose.getY());
    thetaController.reset(currentPose.getRotation().getRadians());
    xController.setTolerance(driveTolerance.get());
    yController.setTolerance(driveTolerance.get());
    thetaController.setTolerance(thetaTolerance.get());
    poseSet = hasTargetSupplier.get();
    driveToPoseState = poseSet ? 2 : 0;
    if (poseSet) {
      drivetrain.resetPoseToVision(() -> visionApriltagSubsystem.getMegaTag2Pose2dFromLimelight());
    }
    this.targetPose =
        hasTargetSupplier.get() ? poseSupplier.get() : new Pose2d(0, 0, new Rotation2d(0));

    Logger.recordOutput("DriveToPose/targetPose", targetPose);

    this.timer.restart();
  }

  /**
   * This method is invoked periodically while this command is scheduled. It calculates the
   * velocities based on the current and target poses and invokes the drivetrain subsystem's drive
   * method.
   */
  @Override
  public void execute() {
    // set running to true in this method to capture that the calculate method has been invoked on
    // the PID controllers. This is important since these controllers will return true for atGoal if
    // the calculate method has not yet been invoked.
    if (poseSet) {
      running = true;

      // Update from tunable numbers
      if (driveKp.hasChanged()
          || driveKd.hasChanged()
          || driveKi.hasChanged()
          || thetaKp.hasChanged()
          || thetaKd.hasChanged()
          || thetaKi.hasChanged()
          || driveMaxVelocity.hasChanged()
          || driveMaxAcceleration.hasChanged()
          || thetaMaxVelocity.hasChanged()
          || thetaMaxAcceleration.hasChanged()
          || driveTolerance.hasChanged()
          || thetaTolerance.hasChanged()) {
        xController.setP(driveKp.get());
        xController.setD(driveKd.get());
        xController.setI(driveKi.get());
        xController.setConstraints(
            new TrapezoidProfile.Constraints(driveMaxVelocity.get(), driveMaxAcceleration.get()));
        xController.setTolerance(driveTolerance.get());
        yController.setP(driveKp.get());
        yController.setD(driveKd.get());
        yController.setI(driveKi.get());
        yController.setConstraints(
            new TrapezoidProfile.Constraints(driveMaxVelocity.get(), driveMaxAcceleration.get()));
        yController.setTolerance(driveTolerance.get());
        thetaController.setP(thetaKp.get());
        thetaController.setD(thetaKd.get());
        thetaController.setI(thetaKi.get());
        thetaController.setConstraints(
            new TrapezoidProfile.Constraints(thetaMaxVelocity.get(), thetaMaxAcceleration.get()));
        thetaController.setTolerance(thetaTolerance.get());
      }

      Pose2d currentPose = drivetrain.getPose();

      double xVelocity = xController.calculate(currentPose.getX(), this.targetPose.getX());
      double yVelocity = yController.calculate(currentPose.getY(), this.targetPose.getY());
      double thetaVelocity =
          thetaController.calculate(
              currentPose.getRotation().getRadians(), this.targetPose.getRotation().getRadians());
      if (xController.atGoal()) xVelocity = 0.0;
      if (yController.atGoal()) yVelocity = 0.0;
      if (thetaController.atGoal()) thetaVelocity = 0.0;
      double multiplier = this.alliance == Alliance.Blue ? 1 : -1;
      drivetrain.driveWOAlliance(
          multiplier * xVelocity, multiplier * yVelocity, thetaVelocity, true, true);
    } else {
      if (hasTargetSupplier.get()) {
        if (driveToPoseState == 0) {
          drivetrain.resetPoseToVision(
              () -> visionApriltagSubsystem.getMegaTag2Pose2dFromLimelight());
          driveToPoseState = 1;
        } else if (driveToPoseState == 1) {
          targetPose = poseSupplier.get();
          Pose2d currentPose = drivetrain.getPose();
          xController.reset(currentPose.getX());
          yController.reset(currentPose.getY());
          thetaController.reset(currentPose.getRotation().getRadians());
          poseSet = true;
          driveToPoseState = 2;
        }
      } else {
        double xPercentage = TeleopSwerve.modifyAxis(translationXSupplier.getAsDouble(), 2.0);
        double yPercentage = TeleopSwerve.modifyAxis(translationYSupplier.getAsDouble(), 2.0);
        double rotationPercentage =
            TeleopSwerve.modifyAxis(rotationSupplier.getAsDouble(), 2.0) * 1.0;

        double xVelocity = xPercentage * RobotConfig.getInstance().getRobotMaxVelocity();
        double yVelocity = yPercentage * RobotConfig.getInstance().getRobotMaxVelocity();
        double rotationalVelocity =
            rotationPercentage * RobotConfig.getInstance().getRobotMaxAngularVelocity();

        drivetrain.drive(
            xVelocity, yVelocity, rotationalVelocity, true, drivetrain.getFieldRelative());
      }
    }
  }

  /**
   * This method returns true if the command has finished. It is invoked periodically while this
   * command is scheduled (after execute is invoked). This command is considered finished if the
   * move-to-pose feature is disabled on the drivetrain subsystem or if the timeout has elapsed or
   * if all the PID controllers are at their goal.
   *
   * @return true if the command has finished
   */
  @Override
  public boolean isFinished() {
    Logger.recordOutput("DriveToPose/xErr", xController.atGoal());
    Logger.recordOutput("DriveToPose/yErr", yController.atGoal());
    Logger.recordOutput("DriveToPose/tErr", thetaController.atGoal());
    // check that running is true (i.e., the calculate method has been invoked on the PID
    // controllers) and that each of the controllers is at their goal. This is important since these
    // controllers will return true for atGoal if the calculate method has not yet been invoked.
    // return !drivetrain.isMoveToPoseEnabled()
    //    || this.timer.hasElapsed(timeout.get())
    //    ||
    return (running && xController.atGoal() && yController.atGoal() && thetaController.atGoal());
  }

  /**
   * This method will be invoked when this command finishes or is interrupted. It stops the motion
   * of the drivetrain.
   *
   * @param interrupted true if the command was interrupted by another command being scheduled
   */
  @Override
  public void end(boolean interrupted) {
    drivetrain.stop();
    running = false;
    Logger.recordOutput("ActiveCommands/DriveToPose", false);
  }
}
