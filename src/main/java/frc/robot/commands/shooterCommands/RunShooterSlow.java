package frc.robot.commands.shooterCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooterWheels.ShooterWheels;

public class RunShooterSlow extends Command {
  private final ShooterWheels shooterWheels;

  public RunShooterSlow(ShooterWheels shooterWheels) {
    this.shooterWheels = shooterWheels;
  }

  public void initialize() {}

  public void execute() {
    shooterWheels.setShooterSpeedSlow();
  }

  public void end(boolean isInterupted) {}

  public boolean isFinnished() {
    return true;
  }
}
