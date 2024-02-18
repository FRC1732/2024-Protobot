// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.limelightVision;

import edu.wpi.first.wpilibj.Timer;
import frc.lib.team3061.vision.VisionIO;
import frc.lib.team6328.util.Alert;
import frc.lib.team6328.util.Alert.AlertType;
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

    if (newResult) { // FIXME untested, needs to be tested to ensure if this actually works
      inputs.estimatedRobotPose = LimelightHelpers.getBotPose3d_wpiBlue(limelightName);
      // inputs.estimatedRobotPoseTimestamp = estimate.timestampSeconds; FIXME find the
      // non-photonvision equivalent of this
      LimelightResults results = LimelightHelpers.getLatestResults(limelightName);
      int[] tags = new int[results.targetingResults.targets_Fiducials.length];
      for (int i = 0; i < results.targetingResults.targets_Fiducials.length; i++) {
        tags[i] = (int) results.targetingResults.targets_Fiducials[i].fiducialID;
      }
      inputs.estimatedRobotPoseTags = tags;
      inputs.lastCameraTimestamp = latestTimestamp;
      lastTimestamp = latestTimestamp;
    }
  }
}
