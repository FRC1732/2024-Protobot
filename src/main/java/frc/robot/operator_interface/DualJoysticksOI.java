// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.operator_interface;

import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/** Class for controlling the robot with two joysticks. */
public class DualJoysticksOI implements OperatorInterface {
  private final CommandJoystick translateJoystick;
  private final CommandJoystick rotateJoystick;
  private final Trigger[] translateJoystickButtons;
  private final Trigger[] rotateJoystickButtons;

  public DualJoysticksOI(int translatePort, int rotatePort) {
    translateJoystick = new CommandJoystick(translatePort);
    rotateJoystick = new CommandJoystick(rotatePort);

    // buttons use 1-based indexing such that the index matches the button number; leave index 0 set
    // to null
    this.translateJoystickButtons = new Trigger[13];
    this.rotateJoystickButtons = new Trigger[13];

    for (int i = 1; i < translateJoystickButtons.length; i++) {
      translateJoystickButtons[i] = translateJoystick.button(i);
      rotateJoystickButtons[i] = rotateJoystick.button(i);
    }
  }

  // Translate Joystick
  @Override
  public double getTranslateX() {
    return -translateJoystick.getY();
  }

  @Override
  public double getTranslateY() {
    return -translateJoystick.getX();
  }

  @Override
  public Trigger aimOrSourceButton() {
    return translateJoystickButtons[1];
  }

  // @Override
  // public Trigger aimSpeakerButton() {
  //   return translateJoystickButtons[1];
  // }

  // @Override
  // public Trigger ampScoreButton() {
  //   return translateJoystickButtons[2];
  // }

  // @Override
  // public Trigger sourceLoadButton() {
  //   return translateJoystickButtons[3];
  // }

  @Override
  public Trigger feedThroughButton() {
    return translateJoystickButtons[6];
  }

  @Override
  public Trigger ejectButton() {
    return translateJoystickButtons[7];
  }

  // Rotate Joystick
  @Override
  public double getRotate() {
    return -rotateJoystick.getX();
  }

  @Override
  public Trigger IntakeOrScoreButton() {
    return rotateJoystickButtons[1];
  }

  @Override
  public Trigger ampModeButton() {
    return rotateJoystickButtons[2];
  }

  @Override
  public Trigger speakerModeButton() {
    return rotateJoystickButtons[3];
  }

  // @Override
  // public Trigger groundIntakeButton() {
  //   return rotateJoystickButtons[1];
  // }

  // @Override
  // public Trigger smartFeedButton() {
  //   return rotateJoystickButtons[2];
  // }

  // @Override
  // public Trigger manualFeedButton() {
  //   return rotateJoystickButtons[3];
  // }

  @Override
  public Trigger resetGyroButton() {
    return rotateJoystickButtons[8];
  }

  @Override
  public Trigger robotCentricButton() {
    return rotateJoystickButtons[10];
  }

  @Override
  public Trigger fieldCentricButton() {
    return rotateJoystickButtons[11];
  }
}
