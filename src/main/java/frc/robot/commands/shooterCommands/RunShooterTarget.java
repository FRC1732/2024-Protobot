package frc.robot.commands.shooterCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer.ShooterTarget;
import frc.robot.subsystems.shooterWheels.ShooterWheels;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

public class RunShooterTarget extends Command {
  private final ShooterWheels shooterWheels;
  private final Supplier<ShooterTarget> type;
  private final BooleanSupplier popShotEnabled;

  public RunShooterTarget(ShooterWheels shooterWheels, Supplier<ShooterTarget> type, BooleanSupplier popShotEnabled) {
    addRequirements(shooterWheels);
    this.shooterWheels = shooterWheels;
    this.type = type;
    this.popShotEnabled = popShotEnabled;
  }

  public void initialize() {
    if(popShotEnabled.getAsBoolean()) {
      shooterWheels.setShooterSpeedPop();
    } else if (type.get() == ShooterTarget.SPEAKER) {
      shooterWheels.setShooterSpeedFast();
    } else {
      shooterWheels.setShooterSpeedMedium();
    }
  }

  public void execute() {}

  public void end(boolean isInterupted) {}

  public boolean isFinished() {
    return true;
  }
}
