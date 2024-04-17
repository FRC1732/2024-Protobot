package frc.robot.commands.shooterCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooterPose.Pose;
import frc.robot.subsystems.shooterPose.ShooterPose;
import frc.robot.subsystems.shooterPose.ShooterPose.ShotType;
import java.util.function.DoubleSupplier;

public class ShootThrough extends Command {
  private final ShooterPose shooterPose;
  private DoubleSupplier distanceSupplierInches;
  private Intake intake;
  private Feeder feeder;

  public ShootThrough(
      ShooterPose shooterPose,
      DoubleSupplier distanceSupplierInches,
      Intake intake,
      Feeder feeder) {
    addRequirements(shooterPose, intake, feeder);
    this.shooterPose = shooterPose;
    this.distanceSupplierInches = distanceSupplierInches;
    this.intake = intake;
    this.feeder = feeder;
  }

  public void initialize() {
    shooterPose.setShooterDistance(distanceSupplierInches.getAsDouble(), ShotType.SPEAKER);
    intake.runIntake();
    feeder.runFeeder();
  }

  public void execute() {}

  public void end(boolean isInterupted) {
    this.shooterPose.setShooterPose(Pose.HANDOFF);
  }

  public boolean isFinished() {
    return false;
  }
}
