// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.operator_interface;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/** Class for controlling the robot with a single Xbox controller. */
public class SingleHandheldOI implements OperatorInterface {
  private final XboxController controller;

  public SingleHandheldOI(int port) {
    controller = new XboxController(port);
  }

  @Override
  public double getTranslateX() {
    return -controller.getLeftY();
  }

  @Override
  public double getTranslateY() {
    return -controller.getLeftX();
  }

  @Override
  public double getRotate() {
    return -controller.getRightX();
  }

  @Override
  public Trigger fieldCentricButton() {
    return new Trigger(controller::getLeftStickButton);
  }

  @Override
  public Trigger resetGyroButton() {
    return new Trigger(controller::getStartButton);
  }

  @Override
  public Trigger robotCentricButton() {
    return new Trigger(controller::getBackButton);
  }

  @Override
  public Trigger aimSpeakerButton() {
    return new Trigger(() -> controller.getLeftTriggerAxis() >= 0.5);
  }

  @Override
  public Trigger ampScoreButton() {
    return new Trigger(controller::getLeftBumper);
  }

  @Override
  public Trigger sourceLoadButton() {
    return new Trigger(controller::getAButton);
  }

  @Override
  public Trigger feedThroughButton() {
    return new Trigger(controller::getXButton);
  }

  @Override
  public Trigger ejectButton() {
    return new Trigger(controller::getBButton);
  }

  @Override
  public Trigger groundIntakeButton() {
    return new Trigger(() -> controller.getRightTriggerAxis() >= 0.5);
  }

  @Override
  public Trigger smartFeedButton() {
    return new Trigger(controller::getRightBumper);
  }

  @Override
  public Trigger manualFeedButton() {
    return new Trigger(controller::getYButton);
  }
}
