// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.statusrgb;

import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.RobotContainer.ScoringMode;
import java.util.function.BooleanSupplier;

public class StatusRgb extends SubsystemBase {
  private DigitalOutput out0 = new DigitalOutput(0);
  private DigitalOutput out1 = new DigitalOutput(1);
  private DigitalOutput out2 = new DigitalOutput(2);
  private DigitalOutput out3 = new DigitalOutput(3);
  private DigitalOutput out4 = new DigitalOutput(4);

  private SpecialMode specialMode = SpecialMode.NONE;

  private Timer timer;
  private double targetElapsedTimeSeconds;

  private RobotContainer robotContainer;

  private BooleanSupplier hasClearence;
  private boolean targetReady;
  private BooleanSupplier whenClimbing;
  private BooleanSupplier noteTarget;

  /** Creates a new StatusRGB. */
  public StatusRgb(
      BooleanSupplier hasClearence, BooleanSupplier whenClimbing, RobotContainer robotContainer, BooleanSupplier noteTarget) {
    this.hasClearence = hasClearence;
    this.whenClimbing = whenClimbing;
    this.robotContainer = robotContainer;
    this.noteTarget = noteTarget;
    timer = new Timer();
    out3.set(!false); // always off, not used
  }

  public void acquiredNote() {
    timer.start();
    targetElapsedTimeSeconds = 1.5;
    specialMode = SpecialMode.NOTE_CAPTURED;
    System.out.println("Started note special");
  }

  public void targetReady(boolean targetReady) {
    this.targetReady = targetReady;
  }

  @Override
  public void periodic() {
    if (specialMode != SpecialMode.NONE) {
      if (timer.hasElapsed(targetElapsedTimeSeconds)) {
        specialMode = SpecialMode.NONE;
        timer.stop();
        timer.reset();
      } else {
        switch (specialMode) {
          case NOTE_CAPTURED: // blue and gold
            out4.set(!true);
            System.out.println("Special mode active");
            break;
          default: // do nothing
            break;
        }
      }
    } else {
      out4.set(!false);
    }

    if (DriverStation.isDisabled()) { // blue and gold
      out0.set(!false);
      out1.set(!false);
      out2.set(!false);
    } else if (targetReady) { // Blue and gold
      out0.set(!true);
      out1.set(!true);
      out2.set(!false);
    } else if (noteTarget.getAsBoolean()) { // orange FIXME unsure if this is the right priority, move if want to change this
      out0.set(!true);
      out1.set(!false);
      out2.set(!true);
    } else if (whenClimbing.getAsBoolean()) { // Blue and gold
      out0.set(!false);
      out1.set(!false);
      out2.set(!true);
    } else if (!hasClearence.getAsBoolean()) { // sparkley
      out0.set(!false);
      out1.set(!true);
      out2.set(!false);
    } else if (robotContainer.scoringMode == ScoringMode.SPEAKER) { // red
      out0.set(!true);
      out1.set(!false);
      out2.set(!false);
    } else { // Blue and Gold
      out0.set(!false);
      out1.set(!false);
      out2.set(!false);
    }
  }

  public enum SpecialMode {
    NOTE_CAPTURED,
    NONE;
  }
}
