// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.feederCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.shooterWheels.ShooterWheels;

public class BrakeFeeder extends Command {
  private Feeder feeder;
  private ShooterWheels shooterWheels;

  public BrakeFeeder(Feeder feeder, ShooterWheels shooterWheels) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(feeder);
    this.feeder = feeder;
    this.shooterWheels = shooterWheels;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (shooterWheels.getShooterSpeed() > 0.8) {
      feeder.brakeFeeder();
    } else {
      feeder.stopFeeder();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    feeder.stopFeeder();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
