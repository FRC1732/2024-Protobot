// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooterCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooterWheels.ShooterSpeed;
import frc.robot.subsystems.shooterWheels.ShooterWheels;

public class Eject extends Command {
  /** Creates a new Eject. */
  private final Feeder feeder;

  private final Intake intake;
  private final ShooterWheels shooter;

  public Eject(Feeder feeder, Intake intake, ShooterWheels shooter) {
    addRequirements(feeder, intake, shooter);
    this.feeder = feeder;
    this.intake = intake;
    this.shooter = shooter;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    feeder.runFeederOut();
    intake.runIntakeOut();
    shooter.setShooterSpeed(ShooterSpeed.RANGE_BACKWARDS);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
