package frc.robot.commands.shooterCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooterPose.ShooterPose;

public class SetShooterDistance extends Command {
  private final ShooterPose shooterPose;
  private double distanceInches;

  public SetShooterDistance(ShooterPose shooterPose, double distanceInches) {
    addRequirements(shooterPose);
    this.shooterPose = shooterPose;
    this.distanceInches = distanceInches;
  }

  public void initialize() {
    shooterPose.setShooterDistance(distanceInches);
  }

  public void execute() {}

  public void end(boolean isInterupted) {}

  public boolean isFinished() {
    return true;
  }
}
