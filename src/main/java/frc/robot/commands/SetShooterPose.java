package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooterPose.ShooterPose;

public class SetShooterPose extends Command {
private final ShooterPose shooterPose;

    public SetShooterPose(ShooterPose shooterPose, double distanceInches) {
        this.shooterPose = shooterPose;
    }
    public void initialize() {

    }
    public void execute() {
    
    }
    public void end(boolean isInterupted) {

    }
    public boolean isFinnished() {
        return true;
    }
}
