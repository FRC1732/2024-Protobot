// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ClimberCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.climber.ClimberConstants;

public class DisarmClimber extends Command {
  private Climber climber;

  private static final double MIN_HEIGHT = ClimberConstants.MIN_SETPOINT_INCHES;

  public DisarmClimber(Climber climber) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(climber);
    this.climber = climber;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (climber.getHeight() > MIN_HEIGHT) {
      climber.ClimberDown();
      ;
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (climber.getHeight() < MIN_HEIGHT + 4.0) {
      climber.ClimberDownSlow();
    } else {
      climber.ClimberDown();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    climber.ClimberStop();
    climber.ClimberDisarm();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return climber.getHeight() <= MIN_HEIGHT;
  }
}
