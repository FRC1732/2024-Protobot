// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.team3061.vision;

import edu.wpi.first.wpilibj.Timer;
import frc.lib.team6328.util.Alert;
import frc.lib.team6328.util.Alert.AlertType;
import frc.robot.limelightVision.LimelightHelpers;
import frc.robot.limelightVision.LimelightHelpers.LimelightResults;

public class VisionIOLimelight implements VisionIO {
  private Alert noCameraConnectedAlert =
      new Alert("specified camera not connected", AlertType.WARNING);
  private double lastTimestamp = 0;
  String limelightName;

  public VisionIOLimelight(String limelightName) {
    this.limelightName = limelightName;
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    double latestTimestamp =
        Timer.getFPGATimestamp()
            - (LimelightHelpers.getLatency_Pipeline(limelightName) / 1000.0)
            - (LimelightHelpers.getLatency_Capture(limelightName) / 1000.0);
    boolean newResult = Math.abs(latestTimestamp - lastTimestamp) > 1e-5;

    if (newResult && LimelightHelpers.getTV(limelightName)) { // FIXME untested, needs to be tested to ensure if this actually works
      inputs.estimatedRobotPose = LimelightHelpers.getBotPose3d(limelightName);
      inputs.estimatedRobotPoseTimestamp = latestTimestamp; // FIXME: What is equivalent LL for this?
      inputs.lastCameraTimestamp = latestTimestamp;

      int[] tags = new int[1];
      tags[0] = (int)LimelightHelpers.getFiducialID(limelightName);
      inputs.estimatedRobotPoseTags = tags;

      lastTimestamp = latestTimestamp;
    }
  }
}
