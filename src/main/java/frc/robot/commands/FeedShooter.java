// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.feeder.Feeder;

public class FeedShooter extends Command {
  private final Feeder feederSystem;
  private final AnalogInput analog;


  /** Creates a new FeedShooter. 
    * @param feeder the drivetrain subsystem required by this command
  */
  
  public FeedShooter(Feeder feeder) {
    addRequirements(feeder);
    feederSystem = feeder;
    analog = new AnalogInput(0);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
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
    if (analog.getValue() > 700) {
      feederSystem.stopFeederIn();
      return true;
    } else {
      return false;
    }
  }
}
