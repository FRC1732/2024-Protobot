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
  private final CommandJoystick operatorPanel;
  private final Trigger[] operatorPanelButtons;

  public FullOperatorConsoleOI(int translatePort, int rotatePort, int operatorPanelPort) {
    super(translatePort, rotatePort);
    operatorPanel = new CommandJoystick(operatorPanelPort);

    this.operatorPanelButtons = new Trigger[13];
    for (int i = 1; i < operatorPanelButtons.length; i++) {
      operatorPanelButtons[i] = operatorPanel.button(i);
    }
  }

  // Operator Panel
  public Trigger slowModeSwitch() {
    return operatorPanelButtons[1];
  }
}
