// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intakeCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.shooterPose.Pose;
import frc.robot.subsystems.shooterPose.ShooterPose;

public class IntakeSourceNote extends Command {
  private final Feeder feeder;
  private final ShooterPose shooterPose;

  /**
   * Creates a new IntakeSourceNote.
   *
   * @param feeder feeder subsystem for this command
   * @param shooterPose shooterPose subsystem for this command
   */
  public IntakeSourceNote(Feeder feeder, ShooterPose shooterPose) {
    addRequirements(feeder, shooterPose);
    this.feeder = feeder;
    this.shooterPose = shooterPose;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    feeder.runFeeder();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    feeder.stopFeeder();
    shooterPose.setShooterPose(Pose.HANDOFF);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return feeder.hasNote();
  }
}
