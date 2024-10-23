// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooterCommands;

import java.util.Map;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.team3061.drivetrain.Drivetrain;
import frc.robot.subsystems.statusrgb.StatusRgb;

public class FolliesSpin extends Command {
  /** Creates a new FolliesSpin. */
  // private final ShooterWheels shooterWheels;
  // private final Feeder feederSystem;
  private final StatusRgb statusRgb;
  private final Timer useTimer;
  private final Drivetrain drivetrain;

  private final double FIRE_TIME = 4;
  private final double SPIN_AMOUNT = 90; // degrees from start position
  private double startDegrees;
  private boolean endOnSpin;

  private GenericEntry spinTime;
  private ShuffleboardTab shuffleTab;

  public FolliesSpin(StatusRgb statusRgb, Drivetrain drive) {
    addRequirements(drive);
    // this.shooterWheels = shooterWheels;
    // this.feederSystem = feederSystem;
    this.statusRgb = statusRgb;
    this.useTimer = new Timer();
    this.drivetrain = drive;
    endOnSpin = false;

    // setup shuffleboard

    shuffleTab = Shuffleboard.getTab("Follies");
    spinTime = shuffleTab.add("Spin Time", FIRE_TIME)
        .withWidget(BuiltInWidgets.kNumberSlider)
        .withProperties(Map.of("min", 0, "max", 10))
        .getEntry();
  }

  public void initialize() {
    useTimer.reset();
    useTimer.start();
    // shooterWheels.setShooterSpeedMedium();
    // startNote = false;
    startDegrees = drivetrain.getRotation().getDegrees();
    statusRgb.changeFollies(true);
    endOnSpin = false;
  }

  public void execute() {
    // System.out.println("gyro degrees: " + drivetrain.getRotation().getDegrees());
    drivetrain.drive(0, 0, 6, false, false);
    if (useTimer.hasElapsed(spinTime.getDouble(FIRE_TIME)) && !endOnSpin) {
      endOnSpin = true;
    }
  }

  public void end(boolean isInterupted) {
    // shooterWheels.stopShooter();
    // feederSystem.stopFeeder();
    drivetrain.drive(0, 0, 0, false, false);
    statusRgb.changeFollies(false);
  }

  public boolean isFinished() {
    // return useTimer.hasElapsed(END_TIME);
    if (endOnSpin && (drivetrain.getRotation().getDegrees() > startDegrees + SPIN_AMOUNT
        || drivetrain.getRotation().getDegrees() < startDegrees - SPIN_AMOUNT)) {
      return true;
    }
    return false;
  }
}
