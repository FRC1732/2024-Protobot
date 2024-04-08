package frc.robot.commands.shooterCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooterPose.Pose;
import frc.robot.subsystems.shooterPose.ShooterPose;
import java.util.function.DoubleSupplier;

public class SetShooterDistanceContinuous extends Command {
  private final ShooterPose shooterPose;
  private DoubleSupplier distanceSupplierInches;

  public SetShooterDistanceContinuous(
      ShooterPose shooterPose, DoubleSupplier distanceSupplierInches) {
    addRequirements(shooterPose);
    this.shooterPose = shooterPose;
    this.distanceSupplierInches = distanceSupplierInches;
  }

  public void initialize() {
    shooterPose.setShooterDistance(distanceSupplierInches.getAsDouble());
  }

  public void execute() {
    shooterPose.setShooterDistance(distanceSupplierInches.getAsDouble());
  }

  public void end(boolean isInterupted) {
    this.shooterPose.setShooterPose(Pose.HANDOFF);
  }

  public boolean isFinished() {
    return false;
  }
}
