package frc.robot.commands.shooterCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooterWheels.ShooterWheels;

public class RunShooterSlow extends Command {
  private final ShooterWheels shooterWheels;

  public RunShooterSlow(ShooterWheels shooterWheels) {
    addRequirements(shooterWheels);
    this.shooterWheels = shooterWheels;
  }

  public void initialize() {
    shooterWheels.setShooterSpeedSlow();
  }

  public void execute() {}

  public void end(boolean isInterupted) {}

  public boolean isFinished() {
    return true;
  }
}
