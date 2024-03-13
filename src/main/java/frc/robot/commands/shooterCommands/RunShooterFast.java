package frc.robot.commands.shooterCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooterWheels.ShooterWheels;
import frc.robot.subsystems.statusrgb.StatusRgb;

public class RunShooterFast extends Command {
  private final ShooterWheels shooterWheels;
  private final StatusRgb statusRgb;

  public RunShooterFast(ShooterWheels shooterWheels, StatusRgb statusRgb) {
    addRequirements(shooterWheels);
    this.shooterWheels = shooterWheels;
    this.statusRgb = statusRgb;
  }

  public void initialize() {
    shooterWheels.setShooterSpeedFast();
    statusRgb.targetVelocity(shooterWheels.isShooterAtFastVelocity());
  }

  public void execute() {}

  public void end(boolean isInterupted) {
    statusRgb.targetVelocity(false);
  }

  public boolean isFinished() {
    return true;
  }
}
