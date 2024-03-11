// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ClimberCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.shooterPose.Pose;
import frc.robot.subsystems.shooterPose.ShooterPose;
import frc.robot.subsystems.shooterWheels.ShooterWheels;

public class AutoClimb extends Command {
  private Feeder feeder;
  private ShooterWheels shooterWheels;
  private ShooterPose shooterPose;
  private Climber climber;

  private static final double MAX_HEIGHT = 22.6;
  private static final double CHAIN_GRAB_HEIGHT = 20.5;
  private static final double SCORING_HEIGHT = 3.0;
  private static final double MIN_HEIGHT = 0.0;
  private boolean firstTimeFlag;

  private Timer timer;

  public AutoClimb(
      Climber climber, ShooterPose shooterPose, ShooterWheels shooterWheels, Feeder feeder) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(feeder);
    addRequirements(shooterWheels);
    addRequirements(climber);
    addRequirements(shooterPose);
    this.feeder = feeder;
    this.shooterWheels = shooterWheels;
    this.climber = climber;
    this.shooterPose = shooterPose;
    timer = new Timer();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    firstTimeFlag = true;
    // since this can be interrupted and resumed, no starting state is assumed
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // state is based on climber height

    if (climber.getHeight() < MIN_HEIGHT) {
      if (firstTimeFlag) {
        timer.restart();
        firstTimeFlag = false;
      }
      climber.ClimberStop();
      if (timer.get() > 1.25) {
        feeder.stopFeeder();
        shooterWheels.stopShooter();
        shooterPose.setShooterPose(Pose.HANDOFF);
      } else {
        shooterWheels.setShooterSpeedSlow();
        feeder.runFeeder();
      }
      return;
    }

    climber.ClimberDown();

    if (climber.getHeight() > CHAIN_GRAB_HEIGHT) {

      return;
    }

    if (climber.getHeight() < CHAIN_GRAB_HEIGHT) {
      if (feeder.hasNote()) {
        if (climber.getHeight() < CHAIN_GRAB_HEIGHT - 4) {
          shooterPose.setShooterPose(Pose.TRAP);
        } else {
          shooterPose.setShooterPose(Pose.TRAP_PREP);
        }
      }
    }

    // if (climber.getHeight() < SCORING_HEIGHT) {
    //   shooterWheels.setShooterSpeedSlow();
    //   feeder.runFeeder();
    // }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    climber.ClimberStop();
    feeder.stopFeeder();
    shooterWheels.stopShooter();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
