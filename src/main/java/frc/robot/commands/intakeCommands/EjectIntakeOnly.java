// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intakeCommands;

import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.intake.Intake;

public class EjectIntakeOnly extends WaitCommand {
  private Intake intake;

  public EjectIntakeOnly(Intake intake) {
    super(0.5);
    addRequirements(intake);
    this.intake = intake;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    super.initialize();
    intake.runIntakeOut();
  }

  @Override
  public void end(boolean interrupted) {
    super.end(interrupted);
    intake.stopIntake();
  }
}
