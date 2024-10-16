// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.feederCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.shooterWheels.ShooterWheels;

public class FeedShooterSmart extends Command {
  private final Feeder feeder;
  private final ShooterWheels shooter;

  /**
   * Creates a new FeedShooterSmart.
   *
   * @param feeder feeder this command uses
   */
  public FeedShooterSmart(Feeder feeder, ShooterWheels shooter) {
    addRequirements(feeder);
    this.feeder = feeder;
    this.shooter = shooter;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.stopShooter();
    feeder.stopFeeder();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (feeder.hasNote()) {
      feeder.stopFeeder();
      return true;
    } else {
      return false;
    }
  }
}
