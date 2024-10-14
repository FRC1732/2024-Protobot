// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooterCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.team3061.drivetrain.Drivetrain;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.shooterWheels.ShooterWheels;
import frc.robot.subsystems.statusrgb.StatusRgb;

public class FolliesSpin extends Command {
  /** Creates a new FolliesSpin. */
  // private final ShooterWheels shooterWheels;
  // private final Feeder feederSystem;
  private final StatusRgb statusRgb;
  private final Timer useTimer;
  private final Drivetrain drivetrain;

  // private final double FIRE_TIME = 3;
  // private final double END_TIME = 6;
  private final double SPIN_AMOUNT = 120; // degrees
  private double startDegrees;

  // private boolean startNote;

  public FolliesSpin(StatusRgb statusRgb, Drivetrain drive) {
    addRequirements(drive);
    // this.shooterWheels = shooterWheels;
    // this.feederSystem = feederSystem;
    this.statusRgb = statusRgb;
    this.useTimer = new Timer();
    this.drivetrain = drive;
    // startNote = false;
  }

  public void initialize() {
    useTimer.reset();
    useTimer.start();
    // shooterWheels.setShooterSpeedMedium();
    // startNote = false;
    startDegrees = drivetrain.getRotation().getDegrees();
    statusRgb.changeFollies(true);
  }

  public void execute() {
    System.out.println("gyro degrees: " + drivetrain.getRotation().getDegrees());
    drivetrain.drive(0, 0, 3, false, false);
    // if (useTimer.hasElapsed(FIRE_TIME) && !startNote) {
    // // feederSystem.runFeeder();
    // startNote = true;
    // }
  }

  public void end(boolean isInterupted) {
    // shooterWheels.stopShooter();
    // feederSystem.stopFeeder();
    drivetrain.drive(0, 0, 0, false, false);
    statusRgb.changeFollies(false);
  }

  public boolean isFinished() {
    // return useTimer.hasElapsed(END_TIME);
    if (drivetrain.getRotation().getDegrees() > startDegrees + SPIN_AMOUNT
        || drivetrain.getRotation().getDegrees() < startDegrees - SPIN_AMOUNT) {
      return true;
    }
    return false;
  }
}
