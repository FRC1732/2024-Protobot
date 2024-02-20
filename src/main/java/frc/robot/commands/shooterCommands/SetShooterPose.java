package frc.robot.commands.shooterCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooterPose.Pose;
import frc.robot.subsystems.shooterPose.ShooterPose;

public class SetShooterPose extends Command {
  private final ShooterPose shooterPose;
  private final Pose pose;

  public SetShooterPose(ShooterPose shooterPose, Pose pose) {
    this.shooterPose = shooterPose;
    addRequirements(shooterPose);
    this.pose = pose;
  }

  public void initialize() {
    shooterPose.setShooterPose(pose);
  }

  public void execute() {}

  public void end(boolean isInterupted) {}

  public boolean isFinished() {
    return shooterPose.isAtGoal();
  }
}
