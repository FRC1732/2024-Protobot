// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.operator_interface;

import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * Class for controlling the robot with two joysticks, 1 Xbox controller, and 1 operator button
 * panel.
 */
public class FullOperatorConsoleOI extends DualJoysticksOI {
  private final CommandJoystick operatorPanelOne;
  private final CommandJoystick operatorPanelTwo;

  private final Trigger[] operatorPanelButtonsOne;
  private final Trigger[] operatorPanelButtonsTwo;

  public FullOperatorConsoleOI(
      int translatePort, int rotatePort, int operatorPanelPortOne, int operatorPanelPortTwo) {
    super(translatePort, rotatePort);
    operatorPanelOne = new CommandJoystick(operatorPanelPortOne);
    operatorPanelTwo = new CommandJoystick(operatorPanelPortTwo);

    this.operatorPanelButtonsOne = new Trigger[13];
    for (int i = 1; i < operatorPanelButtonsOne.length; i++) {
      operatorPanelButtonsOne[i] = operatorPanelOne.button(i);
    }

    this.operatorPanelButtonsTwo = new Trigger[13];
    for (int i = 1; i < operatorPanelButtonsTwo.length; i++) {
      operatorPanelButtonsTwo[i] = operatorPanelTwo.button(i);
    }
  }

  // Operator Panel
  public Trigger operatorSpeakerButton() {
    return operatorPanelButtonsOne[5];
  }

  public Trigger operatorAmpButton() {
    return operatorPanelButtonsOne[4];
  }

  public Trigger operatorFeedButton() {
    return operatorPanelButtonsOne[6];
  }

  public Trigger operatorEjectButton() {
    return operatorPanelButtonsOne[7];
  }

  public Trigger slowModeSwitch() {
    return operatorPanelButtonsTwo[1];
  }

  public Trigger armClimberSwitch() {
    return operatorPanelButtonsTwo[2];
  }

  public Trigger autoClimbButton() {
    return operatorPanelButtonsTwo[3];
  }
}
