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

  public default Trigger aimSpeakerButton() {
    return new Trigger(() -> false);
  }

  public default Trigger ampScoreButton() {
    return new Trigger(() -> false);
  }

  public default Trigger sourceLoadButton() {
    return new Trigger(() -> false);
  }

  public default Trigger feedThroughButton() {
    return new Trigger(() -> false);
  }

  public default Trigger ejectButton() {
    return new Trigger(() -> false);
  }

  public default Trigger groundIntakeButton() {
    return new Trigger(() -> false);
  }

  public default Trigger smartFeedButton() {
    return new Trigger(() -> false);
  }

  public default Trigger manualFeedButton() {
    return new Trigger(() -> false);
  }
}
