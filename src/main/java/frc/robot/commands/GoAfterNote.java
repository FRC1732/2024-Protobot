// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static frc.robot.Constants.LOOP_PERIOD_SECS;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.team3061.RobotConfig;
import frc.lib.team3061.drivetrain.Drivetrain;
import frc.lib.team6328.util.TunableNumber;
import frc.robot.limelightVision.ObjectDetectionVision.VisionObjectDetectionSubsytem;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.statusrgb.StatusRgb;

public class GoAfterNote extends Command {
  private final Drivetrain drivetrain;
  private final VisionObjectDetectionSubsytem visionObjectDetectionSubsystem;
  private final Intake intake;
  private final StatusRgb statusRgb;

  protected static final TunableNumber thetaKp = new TunableNumber("GoAfterNote/ThetaKp", 7);
  protected static final TunableNumber thetaKi = new TunableNumber("GoAfterNote/ThetaKi", 0);
  protected static final TunableNumber thetaKd = new TunableNumber("GoAfterNote/ThetaKd", 0);
  protected static final TunableNumber thetaMaxVelocity = new TunableNumber(
      "GoAfterNote/ThetaMaxVelocity",
      RobotConfig.getInstance().getRobotMaxAngularVelocity() / 2);
  protected static final TunableNumber thetaMaxAcceleration = new TunableNumber("GoAfterNote/ThetaMaxAcceleration", 10);
  protected static final TunableNumber thetaTolerance = new TunableNumber("GoAfterNote/ThetaTolerance", 2);

  protected final ProfiledPIDController thetaController = new ProfiledPIDController(
      thetaKp.get(),
      thetaKi.get(),
      thetaKd.get(),
      new TrapezoidProfile.Constraints(thetaMaxVelocity.get(), thetaMaxAcceleration.get()),
      LOOP_PERIOD_SECS);

  private DoubleSupplier translationXSupplier;
  private DoubleSupplier translationYSupplier;
  private DoubleSupplier rotationSupplier;
  private BooleanSupplier manualRotationOverrideSupplier;

  private boolean previousVisionAssist = false;

  /** Creates a new GoAfterNote. */
  public GoAfterNote(Drivetrain drivetrain,
      VisionObjectDetectionSubsytem visionObjectDetectionSubsystem, Intake intake,
      StatusRgb statusRgb) {
    addRequirements(drivetrain);
    this.drivetrain = drivetrain;
    this.visionObjectDetectionSubsystem = visionObjectDetectionSubsystem;
    this.intake = intake;
    this.statusRgb = statusRgb;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Logger.recordOutput("ActiveCommands/GoAfterNote", true);
    previousVisionAssist = visionObjectDetectionSubsystem.getEnabled();
    visionObjectDetectionSubsystem.setEnabled(true);

    Pose2d currentPose = drivetrain.getPose();
    thetaController.reset(currentPose.getRotation().getRadians());
    thetaController.setTolerance(Math.toRadians(thetaTolerance.get()));

    // configure the controller such that the range of values is centered on the
    // target angle
    thetaController.enableContinuousInput(
        currentPose.getRotation().getRadians() - Math.PI,
        currentPose.getRotation().getRadians() + Math.PI);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (thetaKp.hasChanged()
        || thetaKd.hasChanged()
        || thetaKi.hasChanged()
        || thetaMaxVelocity.hasChanged()
        || thetaMaxAcceleration.hasChanged()
        || thetaTolerance.hasChanged()) {
      thetaController.setP(thetaKp.get());
      thetaController.setI(thetaKi.get());
      thetaController.setD(thetaKd.get());
      thetaController.setConstraints(
          new TrapezoidProfile.Constraints(thetaMaxVelocity.get(), thetaMaxAcceleration.get()));
      thetaController.setTolerance(thetaTolerance.get());
    }

    double thetaVelocity = thetaController.calculate(Units.degreesToRadians(visionObjectDetectionSubsystem.getTX()), 0);

    // switch distance to meters and use distance as percentage of speed.
    double distanceInMeters = visionObjectDetectionSubsystem.getDistanceToTarget() / 39.3701;

    double xPercentage = MathUtil.clamp(distanceInMeters, 0.1, 0.5);

    double xVelocity = xPercentage * RobotConfig.getInstance().getRobotMaxVelocity();
    double yVelocity = 0 * RobotConfig.getInstance().getRobotMaxVelocity(); // robot centric
    double rotationalVelocity = thetaVelocity;
    // rotationPercentage *RobotConfig.getInstance().getRobotMaxAngularVelocity();
    
    Logger.recordOutput("GoAfterNote/VisionTx", visionObjectDetectionSubsystem.getTX());
    Logger.recordOutput("GoAfterNote/VisionTy", visionObjectDetectionSubsystem.getTY());
    Logger.recordOutput("GoAfterNote/HasTarget", visionObjectDetectionSubsystem.hasTarget());
    Logger.recordOutput("GoAfterNote/DriveX", xVelocity);
    Logger.recordOutput("GoAfterNote/DriveY", yVelocity);
    Logger.recordOutput("GoAfterNote/DriveRot", rotationalVelocity);

    drivetrain.drive(xVelocity, yVelocity, rotationalVelocity, true, false);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Logger.recordOutput("ActiveCommands/GoAfterNote", true);
    visionObjectDetectionSubsystem.setEnabled(previousVisionAssist);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return intake.hasNote() || !visionObjectDetectionSubsystem.hasTarget();
  }
}
