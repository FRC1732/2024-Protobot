// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/*
 * Initially from https://github.com/Mechanical-Advantage/RobotCode2022
 */

package frc.robot.operator_interface;

import edu.wpi.first.wpilibj2.command.button.*;

/** Interface for all driver and operator controls. */
public interface OperatorInterface {

  public default double getTranslateX() {
    return 0.0;
  }

  public default double getTranslateY() {
    return 0.0;
  }

  public default double getRotate() {
    return 0.0;
  }

  public default Trigger fieldCentricButton() {
    return new Trigger(() -> false);
  }

  public default Trigger robotCentricButton() {
    return new Trigger(() -> false);
  }

  public default Trigger resetGyroButton() {
    return new Trigger(() -> false);
  }

  public default Trigger slowModeSwitch() {
    return new Trigger(() -> false);
  }

  // public default Trigger aimSpeakerButton() {
  //   return new Trigger(() -> false);
  // }

  // public default Trigger ampScoreButton() {
  //   return new Trigger(() -> false);
  // }

  public default Trigger sourceLoadButton() {
    return new Trigger(() -> false);
  }

  public default Trigger feedThroughButton() {
    return new Trigger(() -> false);
  }

  public default Trigger ejectButton() {
    return new Trigger(() -> false);
  }

  // public default Trigger groundIntakeButton() {
  //   return new Trigger(() -> false);
  // }

  // public default Trigger smartFeedButton() {
  //   return new Trigger(() -> false);
  // }

  //   public default Trigger manualFeedButton() {
  //     return new Trigger(() -> false);
  //   }

  public default Trigger aimOrSourceButton() {
    return new Trigger(() -> false);
  }

  public default Trigger IntakeOrScoreButton() {
    return new Trigger(() -> false);
  }

  public default Trigger ampModeButton() {
    return new Trigger(() -> false);
  }

  public default Trigger speakerModeButton() {
    return new Trigger(() -> false);
  }

  public default Trigger armClimberSwitch() {
    return new Trigger(() -> false);
  }

  public default Trigger autoClimbButton() {
    return new Trigger(() -> false);
  }

  public default Trigger operatorAmpButton() {
    return new Trigger(() -> false);
  }

  public default Trigger operatorSpeakerButton() {
    return new Trigger(() -> false);
  }

  public default Trigger operatorFeedButton() {
    return new Trigger(() -> false);
  }

  public default Trigger operatorEjectButton() {
    return new Trigger(() -> false);
  }

  public default Trigger operatorObjectDetectionAssistButton() {
    return new Trigger(() -> false);
  }

  public default Trigger alignToClimbButton() {
    return new Trigger(() -> false);
  }

}
