// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.feeder.Feeder;

public class FeedShooterSmart extends Command {
  private final Feeder feederSystem;

  /**
   * Creates a new FeedShooterSmart.
   *
   * @param feeder feeder this command uses
   */
  public FeedShooterSmart(Feeder feeder) {
    addRequirements(feeder);
    feederSystem = feeder;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // FIXME add conditions for this to actually start
    feederSystem.runFeederIn();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (feederSystem.hasNote()) {
      feederSystem.stopFeederIn();
      return true;
    } else {
      return false;
    }
  }
}
