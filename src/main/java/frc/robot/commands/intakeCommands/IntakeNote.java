// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intakeCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooterPose.ShooterPose;

public class IntakeNote extends Command {
  /** Creates a new IntakeNote. */
  private Intake intake;

  private Feeder feeder;
  private ShooterPose shooterPose;

  public IntakeNote(Intake intake, Feeder feeder, ShooterPose shooterPose) {
    addRequirements(intake, feeder); // TODO: fix ShooterPose shooterPose
    // Use addRequirements() here to declare subsystem dependencies.
    this.intake = intake;
    this.feeder = feeder;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (feeder.hasNote()) {
      return;
    }
    intake.runIntake();
    feeder.runFeederIn();
    System.out.println("Intake and Feeder running in");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    feeder.stopFeederIn();
    intake.stopIntake();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return feeder.hasNote();
  }
}
