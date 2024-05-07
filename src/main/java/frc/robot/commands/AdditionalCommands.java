package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.lib.team3061.drivetrain.Drivetrain;
import frc.robot.RobotContainer.ShooterTarget;
import frc.robot.commands.feederCommands.WaitForNote;
import frc.robot.commands.shooterCommands.RunShooterSlow;
import frc.robot.commands.shooterCommands.RunShooterTarget;
import frc.robot.commands.shooterCommands.SetShooterDistanceContinuous;
import frc.robot.commands.shooterCommands.SetShooterPose;
import frc.robot.limelightVision.ApriltagVision.VisionApriltagConstants;
import frc.robot.limelightVision.ApriltagVision.VisionApriltagSubsystem;
import frc.robot.operator_interface.OperatorInterface;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.shooterPose.Pose;
import frc.robot.subsystems.shooterPose.ShooterPose;
import frc.robot.subsystems.shooterWheels.ShooterWheels;
import frc.robot.subsystems.statusrgb.StatusRgb;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public final class AdditionalCommands {
  private AdditionalCommands() {
    // disallow construction of this class
  }

  public static SequentialCommandGroup alignToClimbCommand(
      VisionApriltagSubsystem visionApriltagSubsystem,
      Drivetrain drivetrain,
      OperatorInterface oi,
      Supplier<Pose2d> startPoseSupplier,
      Supplier<Pose2d> poseSupplier,
      Supplier<Boolean> hasTargetSupplier) {
    return new InstantCommand(
            () -> {
              visionApriltagSubsystem.setPipeline(VisionApriltagConstants.Pipelines.STAGE);
            })
        .andThen(new WaitCommand(.02))
        .andThen(
            new DriveToPose(
                drivetrain,
                oi::getTranslateX,
                oi::getTranslateY,
                oi::getRotate,
                visionApriltagSubsystem,
                startPoseSupplier,
                hasTargetSupplier))
        .andThen(
            new DriveToPose(
                drivetrain,
                oi::getTranslateX,
                oi::getTranslateY,
                oi::getRotate,
                visionApriltagSubsystem,
                poseSupplier,
                hasTargetSupplier))
        .andThen(
            new InstantCommand(
                () -> {
                  drivetrain.enableTranslationSlowMode();
                  drivetrain.enableRotationSlowMode();
                  drivetrain.disableFieldRelative();
                  drivetrain.enableInvertedY();
                }));
  }

  public static ConditionalCommand restoreFieldConfigurationCommand(
      Drivetrain drivetrain,
      OperatorInterface oi,
      VisionApriltagSubsystem visionApriltagSubsystem) {
    return new ConditionalCommand(
        new InstantCommand(
            () -> {
              drivetrain.enableFieldRelative();
              visionApriltagSubsystem.setPipeline(VisionApriltagConstants.Pipelines.SPEAKER);
              drivetrain.disableInvertedY();
            }),
        new InstantCommand(
            () -> {
              drivetrain.enableFieldRelative();
              visionApriltagSubsystem.setPipeline(VisionApriltagConstants.Pipelines.SPEAKER);
              drivetrain.disableInvertedY();
            }),
        oi.fieldCentricButton()::getAsBoolean);
  }

  public static ConditionalCommand restoreSlowModeConfigurationCommand(
      Drivetrain drivetrain, OperatorInterface oi) {
    return new ConditionalCommand(
        new InstantCommand(
            () -> {
              drivetrain.enableTranslationSlowMode();
              drivetrain.enableRotationSlowMode();
            }),
        new InstantCommand(
            () -> {
              drivetrain.disableTranslationSlowMode();
              drivetrain.disableRotationSlowMode();
            }),
        oi.slowModeSwitch()::getAsBoolean);
  }

  public static ConditionalCommand aimOrSourceCommand(
      ShooterWheels shooterWheels,
      OperatorInterface oi,
      Feeder feeder,
      ShooterPose shooterPose,
      Drivetrain drivetrain,
      StatusRgb statusRgb,
      DoubleSupplier distanceSupplierInches,
      DoubleSupplier rotationSupplierDegrees,
      Supplier<ShooterTarget> targetSupplier,
      BooleanSupplier isPopShotSupplier,
      BooleanSupplier scoringModeSupplier) {

    return new ConditionalCommand(
        // Has note AND is in AMP scoring mode
        new RunShooterSlow(shooterWheels)
            .andThen(
                new WaitForNote(feeder)
                    .andThen(new SetShooterPose(shooterPose, Pose.AMP).asProxy())
                    .alongWith(
                        new RotateToAngle(
                                drivetrain,
                                oi::getTranslateX,
                                oi::getTranslateY,
                                oi::getRotate,
                                () -> -90,
                                () -> false,
                                statusRgb)
                            .asProxy())),
        // Has note AND is in SPEAKER scoring mode
        new RunShooterTarget(shooterWheels, targetSupplier)
            .andThen(
                new WaitForNote(feeder)
                    .andThen(
                        new SetShooterDistanceContinuous(
                                shooterPose,
                                distanceSupplierInches,
                                targetSupplier,
                                isPopShotSupplier)
                            .asProxy())
                    .alongWith(
                        new RotateToAngle(
                                drivetrain,
                                oi::getTranslateX,
                                oi::getTranslateY,
                                oi::getRotate,
                                rotationSupplierDegrees,
                                (() -> false),
                                statusRgb)
                            .asProxy())),
        // Check ScoringMode
        scoringModeSupplier);
  }
}
