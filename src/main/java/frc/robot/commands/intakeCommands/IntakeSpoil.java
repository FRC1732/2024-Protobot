// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intakeCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.limelightVision.ObjectDetectionVision.VisionObjectDetectionSubsytem;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooterPose.Pose;
import frc.robot.subsystems.shooterPose.ShooterPose;

public class IntakeSpoil extends Command {
  /** Creates a new IntakeNote. */
  private Intake intake;

  private Feeder feeder;
  private ShooterPose shooterPose;
  private boolean intakeFlag;
  private VisionObjectDetectionSubsytem vision;
  private int hasTargetCount;

  public IntakeSpoil(
      Intake intake, Feeder feeder, ShooterPose shooterPose, VisionObjectDetectionSubsytem vision) {
    addRequirements(intake, feeder, shooterPose);

    this.intake = intake;
    this.feeder = feeder;
    this.shooterPose = shooterPose;
    this.vision = vision;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    intakeFlag = false;
    hasTargetCount = 0;
    shooterPose.setShooterPose(Pose.HANDOFF);
    if (feeder.hasNote()) {
      return;
    }
    intake.runIntake();
    feeder.runFeeder();
    System.out.println("Intake and Feeder running in");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (hasTargetCount < 10) {
      if (vision.hasTarget()) {
        hasTargetCount++;
      } else {
        hasTargetCount = 0;
      }
    } else {
      feeder.runFeeder();
      if (!feeder.hasNote()) {
        intake.runIntake();
        intakeFlag = true;
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (!intake.hasNote() || feeder.hasNote()) {
      intake.stopIntake();
      feeder.stopFeeder();
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return intakeFlag && (intake.hasNote() || feeder.hasNote());
  }
}
