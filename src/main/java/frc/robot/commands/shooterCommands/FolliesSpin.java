// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooterCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.shooterWheels.ShooterWheels;
import frc.robot.subsystems.statusrgb.StatusRgb;

public class FolliesSpin extends Command {
  /** Creates a new FolliesSpin. */
  private final ShooterWheels shooterWheels;

  private final Feeder feederSystem;
  private final StatusRgb statusRgb;
  private final Timer useTimer;

  private final double FIRE_TIME = 3;
  private final double END_TIME = 6;

  public FolliesSpin(ShooterWheels shooterWheels, Feeder feederSystem, StatusRgb statusRgb) {
    addRequirements(shooterWheels);
    this.shooterWheels = shooterWheels;
    this.feederSystem = feederSystem;
    this.statusRgb = statusRgb;
    this.useTimer = new Timer();
  }

  public void initialize() {
    useTimer.start();
    shooterWheels.setShooterSpeedFast();
    statusRgb.changeFollies(true);
  }

  public void execute() {
    if (useTimer.hasElapsed(FIRE_TIME)) {
      feederSystem.runFeeder();
    }
  }

  public void end(boolean isInterupted) {
    shooterWheels.stopShooter();
    feederSystem.stopFeeder();
    statusRgb.changeFollies(false);
  }

  public boolean isFinished() {
    return useTimer.hasElapsed(END_TIME);
  }
}
