package frc.robot.commands.shooterCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer.ShooterTarget;
import frc.robot.subsystems.shooterPose.Pose;
import frc.robot.subsystems.shooterPose.ShooterPose;
import frc.robot.subsystems.shooterPose.ShooterPose.ShotType;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class SetShooterDistanceContinuous extends Command {
  private final ShooterPose shooterPose;
  private DoubleSupplier distanceSupplierInches;
  private Supplier<ShooterTarget> target;
  BooleanSupplier isPopShot;

  public SetShooterDistanceContinuous(
      ShooterPose shooterPose,
      DoubleSupplier distanceSupplierInches,
      Supplier<ShooterTarget> target,
      BooleanSupplier isPopShot) {
    addRequirements(shooterPose);
    this.shooterPose = shooterPose;
    this.distanceSupplierInches = distanceSupplierInches;
    this.target = target;
    this.isPopShot = isPopShot;
  }

  public void initialize() {}

  public void execute() {
    switch (target.get()) {
      case AMP_ZONE:
      case NEUTRAL_ZONE:
        shooterPose.setShooterDistance(distanceSupplierInches.getAsDouble(), ShotType.PASS);
        break;
      case AMP_ZONE_SKIP:
        shooterPose.setShooterDistance(distanceSupplierInches.getAsDouble(), ShotType.SKIP);
        break;
      case SPEAKER:
      default:
        if (isPopShot.getAsBoolean()) {
          shooterPose.setPopShotDistance(distanceSupplierInches.getAsDouble());
        } else {
          shooterPose.setShooterDistance(distanceSupplierInches.getAsDouble(), ShotType.SPEAKER);
        }
        break;
    }
  }

  public void end(boolean isInterupted) {
    this.shooterPose.setShooterPose(Pose.HANDOFF);
  }

  public boolean isFinished() {
    return false;
  }
}
