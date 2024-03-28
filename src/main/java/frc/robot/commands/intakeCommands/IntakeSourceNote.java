// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intakeCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.shooterPose.Pose;
import frc.robot.subsystems.shooterPose.ShooterPose;
import frc.robot.subsystems.statusrgb.StatusRgb;

public class IntakeSourceNote extends Command {
  private final Feeder feeder;
  private final ShooterPose shooterPose;
  private final StatusRgb statusRgb;

  /**
   * Creates a new IntakeSourceNote.
   *
   * @param feeder feeder subsystem for this command
   * @param shooterPose shooterPose subsystem for this command
   * @param statusRgb statusRgb subsystem for this command
   */
  public IntakeSourceNote(Feeder feeder, ShooterPose shooterPose, StatusRgb statusRgb) {
    addRequirements(feeder, shooterPose);
    this.feeder = feeder;
    this.shooterPose = shooterPose;
    this.statusRgb = statusRgb;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    feeder.runFeeder();
    shooterPose.setShooterPose(Pose.SOURCE);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    feeder.stopFeeder();
    shooterPose.setShooterPose(Pose.HANDOFF);
    if (feeder.hasNote()) {
      statusRgb.acquiredNote();
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return feeder.hasNote();
  }
}
