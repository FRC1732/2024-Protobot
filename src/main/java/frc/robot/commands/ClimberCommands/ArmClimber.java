// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ClimberCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.climber.Climber;

public class ArmClimber extends Command {
  private Climber climber;

  private static final double MAX_HEIGHT = 23.5;

  public ArmClimber(Climber climber) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(climber);
    this.climber = climber;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (climber.getHeight() < MAX_HEIGHT) {
      climber.ClimberUp();
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (climber.getHeight() > MAX_HEIGHT - 4.0) {
      climber.ClimberUpSlow();
    } else {
      climber.ClimberUp();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    climber.ClimberStop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return climber.getHeight() >= MAX_HEIGHT;
  }
}
