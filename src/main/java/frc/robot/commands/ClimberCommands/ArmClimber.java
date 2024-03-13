// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ClimberCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.shooterPose.Pose;
import frc.robot.subsystems.shooterPose.ShooterPose;

public class ArmClimber extends Command {
  private ShooterPose shooterPose;
  private Climber climber;

  private static final double MAX_HEIGHT = 23.0;

  public ArmClimber(Climber climber, ShooterPose shooterPose) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(climber);
    addRequirements(shooterPose);
    this.climber = climber;
    this.shooterPose = shooterPose;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (climber.getHeight() < MAX_HEIGHT) {
      shooterPose.setShooterPose(Pose.CLIMBER);
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
    shooterPose.setShooterPose(Pose.HANDOFF);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return climber.getHeight() >= MAX_HEIGHT;
  }
}
