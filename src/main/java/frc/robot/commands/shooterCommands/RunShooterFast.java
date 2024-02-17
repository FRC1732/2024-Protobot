package frc.robot.commands.shooterCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooterWheels.ShooterWheels;

public class RunShooterFast extends Command {
  private final ShooterWheels shooterWheels;

  public RunShooterFast(ShooterWheels shooterWheels) {
    this.shooterWheels = shooterWheels;
  }

  public void initialize() {
    shooterWheels.setShooterSpeedFast();
  }

  public void execute() {}

  public void end(boolean isInterupted) {}

  public boolean isFinished() {
    return false;
  }
}
