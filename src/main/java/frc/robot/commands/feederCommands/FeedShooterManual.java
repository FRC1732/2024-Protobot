// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.feederCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.intake.Intake;

public class FeedShooterManual extends Command {
  /** Creates a new FeedShooterManual. */
  private Feeder feeder;

  private Intake intake;

  public FeedShooterManual(Intake intake, Feeder feeder) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intake, feeder);
    this.feeder = feeder;
    this.intake = intake;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    feeder.runFeeder();
    intake.runIntake();
    System.out.println("feeder running");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    feeder.stopFeeder();
    intake.stopIntake();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
