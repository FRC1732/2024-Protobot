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
  private BooleanSupplier intaking;
  private BooleanSupplier whenClimbing;
  private BooleanSupplier noteTarget;
  private BooleanSupplier isAtSpeed;
  private BooleanSupplier teamColor;

  /** Creates a new StatusRGB. */
  public StatusRgb(
      BooleanSupplier hasClearence,
      BooleanSupplier whenClimbing,
      RobotContainer robotContainer,
      BooleanSupplier noteTarget,
      BooleanSupplier intaking,
      BooleanSupplier isAtSpeed,
      BooleanSupplier teamColor) {
    this.hasClearence = hasClearence;
    this.whenClimbing = whenClimbing;
    this.robotContainer = robotContainer;
    this.noteTarget = noteTarget;
    this.intaking = intaking;
    this.isAtSpeed = isAtSpeed;
    this.teamColor = teamColor;
    timer = new Timer();
    out3.set(!false); // FIXME remove this when we can flash the arduino again
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
    //out3.set(!teamColor.getAsBoolean()); // set eye color
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

    if (DriverStation.isDisabled()) {
      // mode 0 - blue and gold
      out0.set(!false);
      out1.set(!false);
      out2.set(!false);
    } else if (targetReady && isAtSpeed.getAsBoolean()) {
      // mode 3 - solid green
      out0.set(!true);
      out1.set(!true);
      out2.set(!false);
    } else if (noteTarget.getAsBoolean() && intaking.getAsBoolean()) {
      // mode 5 - solid orange
      out0.set(!true);
      out1.set(!false);
      out2.set(!true);
    } else if (whenClimbing.getAsBoolean()) {
      // mode 4 - sparkley
      out0.set(!false);
      out1.set(!false);
      out2.set(!true);
    } else if (!hasClearence.getAsBoolean()) {
      // mode 2 - solid red
      out0.set(!false);
      out1.set(!true);
      out2.set(!false);
    } else if (robotContainer.scoringMode == ScoringMode.SPEAKER) {
      // mode 1 - solid white
      out0.set(!true);
      out1.set(!false);
      out2.set(!false);
    } else {
      // mode 0 - Blue and Gold
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
