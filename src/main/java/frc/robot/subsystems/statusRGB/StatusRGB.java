// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.statusRGB;

import java.sql.Driver;

import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class StatusRGB extends SubsystemBase {
  private DigitalOutput out0 = new DigitalOutput(1);
  private DigitalOutput out1 = new DigitalOutput(2);
  private DigitalOutput out2 = new DigitalOutput(3);
  private DigitalOutput out3 = new DigitalOutput(4);
  private DigitalOutput out4 = new DigitalOutput(5);

  private SpecialMode specialMode = SpecialMode.NONE;

  private Timer timer;
  private double targetElapsedTimeSeconds;

  private RobotContainer robotContainer;

  private boolean hasBeenEnabled = false;

  /** Creates a new StatusRGB. */
  public StatusRGB() {
    timer = new Timer();
  }

  @Override
  public void periodic() {

    if (!hasBeenEnabled && DriverStation.isEnabled()) {
      hasBeenEnabled = true;
    }
    if (specialMode != SpecialMode.NONE) {
      if (targetElapsedTimeSeconds > 0 && timer.hasElapsed(targetElapsedTimeSeconds)) {
        specialMode = SpecialMode.NONE;
        timer.stop();
      } else {

        switch (specialMode) {
          case GAME_PIECE_CAPTURED:
            // bits 4, 3, 2 -- 28
            out0.set(!true);
            out1.set(!false);
            out2.set(!false);
            out3.set(!false);
            out4.set(!false);
            break;

          case GAME_IDLE_MODE:
            // all bits off
            out0.set(!false);
            out1.set(!true);
            out2.set(!false);
            out3.set(!false);
            out4.set(!false);
            break;
        }
      }
    }
    if (!hasBeenEnabled && DriverStation.isEnabled()) {
      hasBeenEnabled = true;
    }

    if (hasBeenEnabled && DriverStation.isDisabled()) {
      specialMode = SpecialMode.GAME_IDLE_MODE;
      targetElapsedTimeSeconds = 0;
    }
    if (specialMode == SpecialMode.NONE) {
      if (DriverStation.isEnabled()){
        if (DriverStation.getAlliance().get() == Alliance.Red ) {
          out0.set(!false);
          out1.set(!false);
          out2.set(!true);  //red
          out3.set(!false);
          out4.set(!false);
        } else {
          out0.set(!false);
          out1.set(!false);
          out2.set(!true);  //blue
          out3.set(!false);
          out4.set(!false);
        }
      }
    }
  }
    public enum SpecialMode {
      GAME_PIECE_CAPTURED,
      GAME_IDLE_MODE,
      NONE;
    }
}
