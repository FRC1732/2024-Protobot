// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intakeCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.limelightVision.LimelightHelpers;
import frc.robot.limelightVision.VisionConstants;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooterPose.Pose;
import frc.robot.subsystems.shooterPose.ShooterPose;
import frc.robot.subsystems.statusrgb.StatusRgb;

public class IntakeNote extends Command {
  /** Creates a new IntakeNote. */
  private Intake intake;

  private Feeder feeder;
  private ShooterPose shooterPose;
  private StatusRgb statusRgb;

  public IntakeNote(Intake intake, Feeder feeder, ShooterPose shooterPose, StatusRgb statusRgb) {
    addRequirements(intake, feeder, shooterPose);

    this.intake = intake;
    this.feeder = feeder;
    this.shooterPose = shooterPose;
    this.statusRgb = statusRgb;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (feeder.hasNote()) {
      return;
    }
    shooterPose.setShooterPose(Pose.HANDOFF);
    intake.runIntake();
    feeder.runFeeder();
    System.out.println("Intake and Feeder running in");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (intake.hasNote()) {
      statusRgb.acquiredNote();
      LimelightHelpers.setLEDMode_ForceOn(VisionConstants.LIMELIGHT_NAME);
      new FinishIntakingCommand(intake, feeder, shooterPose)
          .schedule(); // TODO: remove when LEDs are fixed
    } else {
      intake.stopIntake();
      feeder.stopFeeder();
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return intake.hasNote();
  }
}
