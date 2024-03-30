// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static frc.robot.Constants.LOOP_PERIOD_SECS;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.team3061.RobotConfig;
import frc.lib.team3061.drivetrain.Drivetrain;
import frc.lib.team6328.util.TunableNumber;
import frc.robot.subsystems.statusrgb.StatusRgb;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class StrafeToPosition extends Command {
  private final Drivetrain drivetrain;
  private final DoubleSupplier txSupplier;
  private final DoubleSupplier translationXSupplier;
  private final DoubleSupplier translationYSupplier;
  private final DoubleSupplier rotationSupplier;
  private final StatusRgb statusRgb;

  private final double STRAFE_MAX_TX = 10;

  protected static final TunableNumber strafeKp = new TunableNumber("Strafe/xKp", .05);
  protected static final TunableNumber strafeKi = new TunableNumber("Strafe/xKi", 0);
  protected static final TunableNumber strafeKd = new TunableNumber("Strafe/xKd", 0);
  protected static final TunableNumber strafeMaxVelocity =
      new TunableNumber(
          "Strafe/XMaxVelocity", RobotConfig.getInstance().getRobotMaxAngularVelocity() / 2);
  protected static final TunableNumber strafeMaxAcceleration =
      new TunableNumber("Strafe/XMaxAcceleration", 10);
  protected static final TunableNumber strafeTolerance = new TunableNumber("Strafe/XTolerance", 2);

  private boolean lastManualOverrideValue;

  protected final ProfiledPIDController strafeController =
      new ProfiledPIDController(
          strafeKp.get(),
          strafeKi.get(),
          strafeKd.get(),
          new TrapezoidProfile.Constraints(strafeMaxVelocity.get(), strafeMaxAcceleration.get()),
          LOOP_PERIOD_SECS);

  public StrafeToPosition(
      Drivetrain drivetrain,
      DoubleSupplier translationXSupplier,
      DoubleSupplier translationYSupplier,
      DoubleSupplier rotationSupplier,
      DoubleSupplier txSupplier,
      StatusRgb statusRgb) {
    this.drivetrain = drivetrain;
    this.statusRgb = statusRgb;
    addRequirements(drivetrain);
    this.translationXSupplier = translationXSupplier;
    this.translationYSupplier = translationYSupplier;
    this.rotationSupplier = rotationSupplier;
   // this.manualOverrideSupplier = manualOverrideSupplier;
    this.txSupplier = txSupplier;
    strafeController.setGoal(0);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // update from tunable numbers
    if (strafeKp.hasChanged()
        || strafeKd.hasChanged()
        || strafeKi.hasChanged()
        || strafeMaxVelocity.hasChanged()
        || strafeMaxAcceleration.hasChanged()
        || strafeTolerance.hasChanged()) {
      strafeController.setP(strafeKp.get());
      strafeController.setI(strafeKi.get());
      strafeController.setD(strafeKd.get());
      strafeController.setConstraints(
          new TrapezoidProfile.Constraints(strafeMaxVelocity.get(), strafeMaxAcceleration.get()));
      strafeController.setTolerance(strafeTolerance.get());
    }

    /*if (lastManualOverrideValue != manualOverrideSupplier.getAsBoolean()) {
      strafeController.reset(0);
      //strafeController.setGoal(targetPositionSupplier.getAsDouble());
    }*/
    double strafePercentage =
        strafeController.calculate(txSupplier.getAsDouble(), 0)
            * .25;
    double strafeCmd = strafePercentage * RobotConfig.getInstance().getRobotMaxVelocity();

    double xPercentage = TeleopSwerve.modifyAxis(translationXSupplier.getAsDouble(), 2.0);
    double yPercentage = TeleopSwerve.modifyAxis(translationYSupplier.getAsDouble(), 2.0);
    double rotationPercentage = TeleopSwerve.modifyAxis(rotationSupplier.getAsDouble(), 2.0) * 1.0;

    double xVelocity = xPercentage * RobotConfig.getInstance().getRobotMaxVelocity();
    double yVelocity = yPercentage * RobotConfig.getInstance().getRobotMaxVelocity();
    double rotationalVelocity =
        rotationPercentage * RobotConfig.getInstance().getRobotMaxAngularVelocity();

    boolean usingOverride = false;//manualOverrideSupplier.getAsBoolean();
    double xVelocityCmd = usingOverride ? xVelocity : -xVelocity;
    double yVelocityCmd = usingOverride ? yVelocity : -strafeCmd;
    double rotVelCmd = usingOverride ? rotationalVelocity : 0;

    drivetrain.drive(xVelocityCmd, yVelocityCmd, rotVelCmd, true, drivetrain.getFieldRelative());

    //lastManualOverrideValue = manualOverrideSupplier.getAsBoolean();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
