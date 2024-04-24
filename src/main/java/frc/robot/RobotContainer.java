// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.team3061.RobotConfig;
import frc.lib.team3061.drivetrain.Drivetrain;
import frc.lib.team3061.drivetrain.DrivetrainIOCTRE;
import frc.lib.team3061.util.RobotOdometry;
import frc.robot.commands.ClimberCommands.ArmClimber;
import frc.robot.commands.ClimberCommands.AutoClimb;
import frc.robot.commands.ClimberCommands.DisarmClimber;
import frc.robot.commands.DriveToPose;
import frc.robot.commands.RotateToAngle;
import frc.robot.commands.TeleopSwerve;
import frc.robot.commands.feederCommands.FeedShooterManual;
import frc.robot.commands.feederCommands.WaitForNote;
import frc.robot.commands.intakeCommands.Eject;
import frc.robot.commands.intakeCommands.FeedThrough;
import frc.robot.commands.intakeCommands.FinishIntakingCommand;
import frc.robot.commands.intakeCommands.IntakeNote;
import frc.robot.commands.intakeCommands.IntakeSourceNote;
import frc.robot.commands.intakeCommands.IntakeSpoil;
import frc.robot.commands.intakeCommands.StartIntakingNote;
import frc.robot.commands.shooterCommands.RunShooterFast;
import frc.robot.commands.shooterCommands.RunShooterMedium;
import frc.robot.commands.shooterCommands.RunShooterSlow;
import frc.robot.commands.shooterCommands.RunShooterSpoil;
import frc.robot.commands.shooterCommands.RunShooterTarget;
import frc.robot.commands.shooterCommands.SetShooterDistance;
import frc.robot.commands.shooterCommands.SetShooterDistanceContinuous;
import frc.robot.commands.shooterCommands.SetShooterPose;
import frc.robot.commands.shooterCommands.ShootThrough;
import frc.robot.commands.shooterCommands.StopShooter;
import frc.robot.configs.DefaultRobotConfig;
import frc.robot.limelightVision.ApriltagVision.VisionApriltagConstants;
import frc.robot.limelightVision.ApriltagVision.VisionApriltagSubsystem;
import frc.robot.limelightVision.LimelightHelpers;
import frc.robot.limelightVision.ObjectDetectionVision.VisionObjectDetectionSubsytem;
import frc.robot.operator_interface.OISelector;
import frc.robot.operator_interface.OperatorInterface;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooterPose.Pose;
import frc.robot.subsystems.shooterPose.ShooterPose;
import frc.robot.subsystems.shooterWheels.ShooterWheels;
import frc.robot.subsystems.statusrgb.StatusRgb;
import java.util.HashMap;
import java.util.LinkedList;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.littletonrobotics.junction.networktables.LoggedDashboardNumber;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  private OperatorInterface oi = new OperatorInterface() {};
  private RobotConfig config;
  private Drivetrain drivetrain;
  private Alliance lastAlliance = DriverStation.Alliance.Blue;
  private VisionApriltagSubsystem visionApriltagSubsystem;
  private VisionObjectDetectionSubsytem visionObjectDetectionSubsystem;
  public Intake intake;
  public Feeder feeder;
  public ShooterWheels shooterWheels;
  public ShooterPose shooterPose;
  public Climber climber;
  public StatusRgb statusRgb;

  public enum ScoringMode {
    AMP,
    SPEAKER
  }

  public boolean popShotEnabled = false;

  public enum ShooterTarget {
    SPEAKER,
    AMP_ZONE,
    AMP_ZONE_SKIP,
    NEUTRAL_ZONE,
    NEUTRAL_ZONE_SKIP
  }

  private double lastVisionError;
  private double lastRotateGoal;

  // private double lastObjectDetectionVisionError;
  // private double lastObjectDetectionRotateGoal;

  public ScoringMode scoringMode = ScoringMode.AMP;

  private HashMap<Double, Double> alignToClimbLookupRot = new HashMap<>();
  private HashMap<Double, Pose2d> alignToClimbLookupPose = new HashMap<>();
  private HashMap<Double, Pose2d> alignToClimbLookupStartPose = new HashMap<>();

  // use AdvantageKit's LoggedDashboardChooser instead of SendableChooser to
  // ensure accurate logging
  private final LoggedDashboardChooser<Command> autoChooser =
      new LoggedDashboardChooser<>("Auto Routine");

  private final LoggedDashboardNumber endgameAlert1 =
      new LoggedDashboardNumber("Endgame Alert #1", 20.0);
  private final LoggedDashboardNumber endgameAlert2 =
      new LoggedDashboardNumber("Endgame Alert #2", 10.0);

  // RobotContainer singleton
  private static RobotContainer robotContainer = new RobotContainer();

  private final LinkedList<AngleTimePair> previousAngles = new LinkedList<>();

  private static class AngleTimePair {
    final double angle;
    final double time;

    AngleTimePair(double angle, double time) {
      this.angle = angle;
      this.time = time;
    }
  }

  private final Translation2d blueSpeakerLocation = new Translation2d(0.2286, 5.541518 - 0.1);
  private final Translation2d redSpeakerLocation = new Translation2d(16.38935, 5.541518 - 0.1);
  private final Translation2d blueAmpZoneLocation = new Translation2d(0.0, 7.3);
  private final Translation2d redAmpZoneLocation = new Translation2d(16.61795, 7.3);
  private final Translation2d blueNeutralZoneLocation = new Translation2d(6.60, 6.9);
  private final Translation2d redNeutralZoneLocation = new Translation2d(10.0, 6.9);

  /**
   * Create the container for the robot. Contains subsystems, operator interface (OI) devices, and
   * commands.
   */
  public RobotContainer() {
    /*
     * IMPORTANT: The RobotConfig subclass object *must* be created before any other
     * objects
     * that use it directly or indirectly. If this isn't done, a null pointer
     * exception will result.
     */
    createRobotConfig();

    // LEDs.getInstance();

    createSubsystems();

    // disable all telemetry in the LiveWindow to reduce the processing during each
    // iteration
    LiveWindow.disableAllTelemetry();

    constructField();

    updateOI();

    configureAutoCommands();
  }

  /**
   * The RobotConfig subclass object *must* be created before any other objects that use it directly
   * or indirectly. If this isn't done, a null pointer exception will result.
   */
  private void createRobotConfig() {
    config = new DefaultRobotConfig();
  }

  private void createSubsystems() {
    /*
     * int[] driveMotorCANIDs = config.getSwerveDriveMotorCANIDs();
     * int[] steerMotorCANDIDs = config.getSwerveSteerMotorCANIDs();
     * int[] steerEncoderCANDIDs = config.getSwerveSteerEncoderCANIDs();
     * double[] steerOffsets = config.getSwerveSteerOffsets();
     *
     * SwerveModuleIO flModule =
     * new SwerveModuleIOTalonFXPhoenix6(
     * 0, driveMotorCANIDs[0], steerMotorCANDIDs[0], steerEncoderCANDIDs[0],
     * steerOffsets[0]);
     *
     * SwerveModuleIO frModule =
     * new SwerveModuleIOTalonFXPhoenix6(
     * 1, driveMotorCANIDs[1], steerMotorCANDIDs[1], steerEncoderCANDIDs[1],
     * steerOffsets[1]);
     *
     * SwerveModuleIO blModule =
     * new SwerveModuleIOTalonFXPhoenix6(
     * 2, driveMotorCANIDs[2], steerMotorCANDIDs[2], steerEncoderCANDIDs[2],
     * steerOffsets[2]);
     *
     * SwerveModuleIO brModule =
     * new SwerveModuleIOTalonFXPhoenix6(
     * 3, driveMotorCANIDs[3], steerMotorCANDIDs[3], steerEncoderCANDIDs[3],
     * steerOffsets[3]);
     */
    // GyroIO gyro = new GyroIOPigeon2Phoenix6(config.getGyroCANID());
    // DrivetrainIO drivetrainIO =
    // new DrivetrainIOGeneric(gyro, flModule, frModule, blModule, brModule);
    DrivetrainIOCTRE drivetrainIO = new DrivetrainIOCTRE();
    drivetrain = new Drivetrain(drivetrainIO);

    intake = new Intake();
    feeder = new Feeder();
    shooterWheels = new ShooterWheels();
    shooterPose = new ShooterPose();
    climber = new Climber();

    alignToClimbLookupRot.put(15.0, 120.0);
    alignToClimbLookupRot.put(16.0, -120.0);
    alignToClimbLookupRot.put(14.0, 0.0);
    alignToClimbLookupRot.put(13.0, 180.0);
    alignToClimbLookupRot.put(12.0, -60.0);
    alignToClimbLookupRot.put(11.0, 60.0);
    double deltaX = 0;
    double deltaY = 0;
    alignToClimbLookupPose.put(
        16.0,
        new Pose2d(
            (182.73 + deltaX) * .0254 - 7 * .0254,
            (146.19 + deltaY) * .0254 - 12.8 * .0254,
            new Rotation2d(Math.toRadians(-120))));
    alignToClimbLookupPose.put(
        15.0,
        new Pose2d(
            (182.73 + deltaX + 1) * .0254 - 7.25 * .0254,
            (177.1 + deltaY + 1) * .0254 + 6.5 * .0254,
            new Rotation2d(Math.toRadians(120))));
    alignToClimbLookupPose.put(
        14.0,
        new Pose2d(
            (209.48 + deltaX) * .0254 + .35,
            (161.62 + deltaY - 7) * .0254,
            new Rotation2d(Math.toRadians(0))));
    alignToClimbLookupPose.put(
        13.0,
        new Pose2d(
            441.74 * .0254 - .27, (161.62 - 3) * .0254, new Rotation2d(Math.toRadians(-180))));
    alignToClimbLookupPose.put(
        12.0,
        new Pose2d(
            468.69 * .0254 + 6.25 * .0254,
            177.1 * .0254 + 4.8 * .0254,
            new Rotation2d(Math.toRadians(60))));
    alignToClimbLookupPose.put(
        11.0,
        new Pose2d(
            468.693 * .0254 + 7 * .0254,
            146.19 * .0254 - 12.8 * .0254,
            new Rotation2d(Math.toRadians(-60))));

    alignToClimbLookupStartPose.put(
        16.0,
        new Pose2d(
            (182.73 + deltaX) * .0254 - 26.5 * .0254,
            (146.19 + deltaY) * .0254 - (29 + 17) * .0254,
            new Rotation2d(Math.toRadians(-120))));

    alignToClimbLookupStartPose.put(
        15.0,
        new Pose2d(
            (182.73 + deltaX) * .0254 - 23.75 * .0254,
            (177.1 + deltaY) * .0254 + (5.3 + 36) * .0254,
            new Rotation2d(Math.toRadians(120))));

    alignToClimbLookupStartPose.put(
        14.0,
        new Pose2d(
            (209.48 + deltaX) * .0254 + 1.30,
            (161.62 + deltaY - 3) * .0254,
            new Rotation2d(Math.toRadians(0))));

    alignToClimbLookupStartPose.put(
        13.0,
        new Pose2d(
            441.74 * .0254 - 1.30, (161.62 - 3) * .0254, new Rotation2d(Math.toRadians(-180))));
    alignToClimbLookupStartPose.put(
        12.0,
        new Pose2d(
            468.69 * .0254 + 23.75 * .0254,
            177.1 * .0254 + 41.3 * .0254,
            new Rotation2d(Math.toRadians(60))));
    alignToClimbLookupStartPose.put(
        11.0,
        new Pose2d(
            468.693 * .0254 + 26.5 * .0254,
            146.19 * .0254 - 36 * .0254,
            new Rotation2d(Math.toRadians(-60))));

    visionApriltagSubsystem =
        new VisionApriltagSubsystem(
            () ->
                drivetrain
                    .getPose()
                    .getRotation()
                    // .plus(
                    //     lastAlliance == Alliance.Blue
                    //         ? Rotation2d.fromDegrees(0.0)
                    //         : Rotation2d.fromDegrees(180.0))
                    .getDegrees());
    visionObjectDetectionSubsystem = new VisionObjectDetectionSubsytem();

    statusRgb =
        new StatusRgb(
            () -> shooterPose.hasClearence(),
            () -> climber.isClimbing(),
            this,
            () -> visionObjectDetectionSubsystem.hasTargetRgb(),
            () -> intake.isIntaking(),
            () -> shooterWheels.isAtSpeed(),
            () -> climber.isClimberArmed());

    // String[] cameraNames = config.getCameraNames(); //TODO: Uncomment Camera
    // stuff
    // Transform3d[] robotToCameraTransforms = config.getRobotToCameraTransforms();
    // VisionIO[] visionIOs = new VisionIO[cameraNames.length];
    // AprilTagFieldLayout layout;
    // try {
    // layout = new AprilTagFieldLayout(VisionConstants.APRILTAG_FIELD_LAYOUT_PATH);
    // } catch (IOException e) {
    // layout = new AprilTagFieldLayout(new ArrayList<>(), 16.4592, 8.2296);
    // }
    // for (int i = 0; i < visionIOs.length; i++) {
    // visionIOs[i] = new VisionIOPhotonVision(cameraNames[i], layout,
    // robotToCameraTransforms[i]);
    // }
    // vision = new Vision(visionIOs);
  }

  /**
   * Creates the field from the defined regions and transition points from one region to its
   * neighbor. The field is used to generate paths.
   *
   * <p>FIXME: update for 2024 regions
   */
  private void constructField() {
    Field2d.getInstance().setRegions(new Region2d[] {});
  }

  /**
   * This method scans for any changes to the connected operator interface (e.g., joysticks). If
   * anything changed, it creates a new OI object and binds all of the buttons to commands.
   */
  public void updateOI() {
    OperatorInterface prevOI = oi;
    oi = OISelector.getOperatorInterface();
    if (oi == prevOI) {
      return;
    }

    configureButtonBindings();
  }

  /**
   * Factory method to create the singleton robot container object.
   *
   * @return the singleton robot container object
   */
  public static RobotContainer getInstance() {
    return robotContainer;
  }

  /** Use this method to define your button->command mappings. */
  private void configureButtonBindings() {
    // oi.alignToClimbButton()
    //     .whileTrue(
    //         new InstantCommand(
    //                 () -> {
    //                   visionApriltagSubsystem.setPipeline(VisionApriltagConstants.Pipelines.STAGE);
    //                 })
    //             .andThen(new WaitCommand(.02))
    //             .andThen(
    //                 new DriveToPose(
    //                     drivetrain,
    //                     oi::getTranslateX,
    //                     oi::getTranslateY,
    //                     oi::getRotate,
    //                     visionApriltagSubsystem,
    //                     () ->
    //                         (lastAlliance == Alliance.Blue
    //                                 ? visionApriltagSubsystem.hasStageTargetBlue()
    //                                 : visionApriltagSubsystem.hasStageTargetRed())
    //                             ? alignToClimbLookupStartPose.get(
    //                                 visionApriltagSubsystem.getAprilTagId())
    //                             : alignToClimbLookupStartPose.get(13.0),
    //                     () ->
    //                         (lastAlliance == Alliance.Blue
    //                             ? visionApriltagSubsystem.hasStageTargetBlue()
    //                             : visionApriltagSubsystem.hasStageTargetRed())))
    //             .andThen(
    //                 new DriveToPose(
    //                     drivetrain,
    //                     oi::getTranslateX,
    //                     oi::getTranslateY,
    //                     oi::getRotate,
    //                     visionApriltagSubsystem,
    //                     () ->
    //                         (lastAlliance == Alliance.Blue
    //                                 ? visionApriltagSubsystem.hasStageTargetBlue()
    //                                 : visionApriltagSubsystem.hasStageTargetRed())
    //                             ? alignToClimbLookupPose.get(
    //                                 visionApriltagSubsystem.getAprilTagId())
    //                             : alignToClimbLookupPose.get(13.0),
    //                     () ->
    //                         (lastAlliance == Alliance.Blue
    //                             ? visionApriltagSubsystem.hasStageTargetBlue()
    //                             : visionApriltagSubsystem.hasStageTargetRed())))
    //             .andThen(
    //                 new InstantCommand(
    //                     () -> {
    //                       drivetrain.enableTranslationSlowMode();
    //                       drivetrain.enableRotationSlowMode();
    //                       drivetrain.disableFieldRelative();
    //                       drivetrain.enableInvertedY();
    //                     })));

    // oi.alignToClimbButton()
    //     .onFalse(
    //         new ConditionalCommand(
    //                 new InstantCommand(
    //                     () -> {
    //                       drivetrain.enableFieldRelative();
    //                       visionApriltagSubsystem.setPipeline(
    //                           VisionApriltagConstants.Pipelines.SPEAKER);
    //                       drivetrain.disableInvertedY();
    //                     }),
    //                 new InstantCommand(
    //                     () -> {
    //                       drivetrain.enableFieldRelative();
    //                       visionApriltagSubsystem.setPipeline(
    //                           VisionApriltagConstants.Pipelines.SPEAKER);
    //                       drivetrain.disableInvertedY();
    //                     }),
    //                 oi.fieldCentricButton()::getAsBoolean)
    //             .andThen(
    //                 new ConditionalCommand(
    //                     new InstantCommand(
    //                         () -> {
    //                           drivetrain.enableTranslationSlowMode();
    //                           drivetrain.enableRotationSlowMode();
    //                         }),
    //                     new InstantCommand(
    //                         () -> {
    //                           drivetrain.disableTranslationSlowMode();
    //                           drivetrain.disableRotationSlowMode();
    //                         }),
    //                     oi.slowModeSwitch()::getAsBoolean)));

    oi.aimOrSourceButton()
        .whileTrue(
            new ConditionalCommand(
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
                new RunShooterTarget(shooterWheels, () -> getShooterTarget())
                    .andThen(
                        new WaitForNote(feeder)
                            .andThen(
                                new SetShooterDistanceContinuous(
                                        shooterPose,
                                        () -> getDistanceToTargetInches(getRobotToTargetVector()),
                                        () -> getShooterTarget(),
                                        () -> popShotEnabled)
                                    .asProxy())),
                            // .alongWith(
                            //     // new BrakeFeeder(feeder, shooterWheels).asProxy(),
                            //     new RotateToAngle(
                            //             drivetrain,
                            //             oi::getTranslateX,
                            //             oi::getTranslateY,
                            //             oi::getRotate,
                            //             () -> getRotationToTargetDegrees(getRobotToTargetVector()),
                            //             (() -> false),
                            //             statusRgb)
                            //         .asProxy())),
                // Check ScoringMode
                () -> scoringMode == ScoringMode.AMP));

    oi.aimOrSourceButton()
        .onFalse(
            new StopShooter(shooterWheels).andThen(new SetShooterPose(shooterPose, Pose.HANDOFF)));

    oi.sourceLoadButton()
        .whileTrue(
            new IntakeSourceNote(feeder, shooterPose, statusRgb)
                .alongWith(
                    new RotateToAngle(
                        drivetrain,
                        oi::getTranslateX,
                        oi::getTranslateY,
                        oi::getRotate,
                        () -> lastAlliance == Alliance.Blue ? 120 : 60,
                        () -> false,
                        statusRgb)));

    oi.IntakeOrScoreButton()
        .whileTrue(
            new ConditionalCommand( // scores if feeder has note
                (new FeedShooterManual(feeder).asProxy()),
                new IntakeNote(intake, feeder, shooterPose, statusRgb)
                    .asProxy() // intakes with object detection
                    .deadlineWith(
                        new ConditionalCommand(
                            new RotateToAngle(
                                    drivetrain,
                                    oi::getTranslateX,
                                    oi::getTranslateY,
                                    oi::getRotate,
                                    () -> noteDetectionAngleHelper(visionObjectDetectionSubsystem),
                                    // targetAngleHelper(
                                    // visionObjectDetectionSubsystem.getTX(),
                                    // visionObjectDetectionSubsystem.getLatencyPipeline()
                                    // + visionObjectDetectionSubsystem.getLatencyCapture()),
                                    () -> !visionObjectDetectionSubsystem.isAssistEnabled(),
                                    statusRgb)
                                .asProxy(),
                            new InstantCommand(),
                            () -> visionObjectDetectionSubsystem.isAssistEnabled()))
                    .andThen(
                        new ConditionalCommand(
                            new RunShooterMedium(shooterWheels).asProxy(),
                            new InstantCommand(),
                            () -> scoringMode == ScoringMode.SPEAKER)),
                feeder::hasNote));

    oi.speakerModeButton()
        .onTrue(
            new InstantCommand(() -> scoringMode = ScoringMode.SPEAKER)
                .andThen(
                    new ConditionalCommand(
                        new RunShooterMedium(shooterWheels),
                        new InstantCommand(),
                        () -> feeder.hasNote())));
    oi.operatorSpeakerButton()
        .onTrue(
            new InstantCommand(() -> scoringMode = ScoringMode.SPEAKER)
                .andThen(
                    new ConditionalCommand(
                        new RunShooterMedium(shooterWheels),
                        new InstantCommand(),
                        () -> feeder.hasNote())));
    oi.ampModeButton()
        .onTrue(
            new ConditionalCommand(
                    new StopShooter(shooterWheels).asProxy(),
                    new InstantCommand(),
                    () -> scoringMode == ScoringMode.SPEAKER)
                .andThen(new InstantCommand(() -> scoringMode = ScoringMode.AMP)));
    oi.operatorAmpButton()
        .onTrue(
            new ConditionalCommand(
                    new StopShooter(shooterWheels).asProxy(),
                    new InstantCommand(),
                    () -> scoringMode == ScoringMode.SPEAKER)
                .andThen(new InstantCommand(() -> scoringMode = ScoringMode.AMP)));

    oi.popShotToggleButton().onTrue(new InstantCommand(() -> popShotEnabled = true));
    oi.popShotToggleButton().onFalse(new InstantCommand(() -> popShotEnabled = false));

    oi.armClimberSwitch().onTrue(new ArmClimber(climber));
    oi.armClimberSwitch().onFalse(new DisarmClimber(climber));
    oi.autoClimbButton().whileTrue(new AutoClimb(climber, shooterPose, shooterWheels, feeder));

    // new SetShooterPose(shooterPose, Pose.TRAP)
    // .andThen(new InstantCommand(() -> climber.ClimberDown())));
    // oi.autoClimbButton()
    // .onFalse(
    // new InstantCommand(() -> climber.ClimberStop())
    // .andThen(new SetShooterPose(shooterPose, Pose.HANDOFF)));

    // oi.groundIntakeButton().whileTrue(new IntakeNote(intake, feeder,
    // shooterPose));

    // oi.sourceLoadButton().whileTrue(new IntakeSourceNote(feeder, shooterPose));

    // oi.ampScoreButton()
    // .whileTrue(
    // new RunShooterSlow(shooterWheels).andThen(new SetShooterPose(shooterPose,
    // Pose.AMP)));
    // oi.ampScoreButton()
    // .onFalse(
    // new StopShooter(shooterWheels).andThen(new SetShooterPose(shooterPose,
    // Pose.HANDOFF)));

    // oi.aimSpeakerButton()
    // .whileTrue(
    // new RunShooterFast(shooterWheels)
    // .andThen(
    // new SetShooterDistanceContinuous(
    // shooterPose,
    // () ->
    // visionSubsystem.hasTarget()
    // ? visionSubsystem.getDistanceToTarget()
    // : 105)
    // .alongWith(
    // new BrakeFeeder(feeder, shooterWheels),
    // new RotateToAngle(
    // drivetrain,
    // oi::getTranslateX,
    // oi::getTranslateY,
    // oi::getRotate,
    // () ->
    // drivetrain.getPose().getRotation().getDegrees()
    // + visionSubsystem.getTX(),
    // () -> !visionSubsystem.hasTarget()))));

    // oi.aimSpeakerButton()
    // .onFalse(
    // new StopShooter(shooterWheels).andThen(new SetShooterPose(shooterPose,
    // Pose.HANDOFF)));

    // oi.smartFeedButton().whileTrue(new FeedShooterManual(feeder));

    // oi.manualFeedButton().whileTrue(new FeedShooterManual(feeder));

    oi.ejectButton().whileTrue(new Eject(feeder, intake, shooterWheels));
    oi.operatorEjectButton().whileTrue(new Eject(feeder, intake, shooterWheels));

    // oi.feedThroughButton().whileTrue(new FeedThrough(feeder, intake,
    // shooterWheels));
    oi.operatorFeedButton().whileTrue(new FeedThrough(feeder, intake, shooterWheels));

    oi.operatorObjectDetectionAssistButton()
        .onTrue(new InstantCommand(() -> visionObjectDetectionSubsystem.setEnabled(true)));
    oi.operatorObjectDetectionAssistButton()
        .onFalse(new InstantCommand(() -> visionObjectDetectionSubsystem.setEnabled(false)));

    configureDrivetrainCommands();

    configureSubsystemCommands();

    // Endgame alerts
    new Trigger(
            () ->
                DriverStation.isTeleopEnabled()
                    && DriverStation.getMatchTime() > 0.0
                    && DriverStation.getMatchTime() <= Math.round(endgameAlert1.get()))
        .onTrue(new PrintCommand("End Game Alert 1."));
    /*
     * Commands.run(() -> LEDs.getInstance().setEndgameAlert(true))
     * .withTimeout(1.5)
     * .andThen(
     * Commands.run(() -> LEDs.getInstance().setEndgameAlert(false))
     * .withTimeout(1.0)));
     */
    new Trigger(
            () ->
                DriverStation.isTeleopEnabled()
                    && DriverStation.getMatchTime() > 0.0
                    && DriverStation.getMatchTime() <= Math.round(endgameAlert2.get()))
        .onTrue(new PrintCommand("End Game Alert 2."));
    /*
     * Commands.sequence(
     * Commands.run(() ->
     * LEDs.getInstance().setEndgameAlert(true)).withTimeout(0.5),
     * Commands.run(() ->
     * LEDs.getInstance().setEndgameAlert(false)).withTimeout(0.5),
     * Commands.run(() ->
     * LEDs.getInstance().setEndgameAlert(true)).withTimeout(0.5),
     * Commands.run(() ->
     * LEDs.getInstance().setEndgameAlert(false)).withTimeout(1.0)));
     */

  }

  public Translation2d getRobotToTargetVector() {
    ShooterTarget target = getShooterTarget();
    Logger.recordOutput("RobotContainer/ShooterTarget", target);
    return getRobotToTargetVector(target);
  }

  public Translation2d getRobotToTargetVector(ShooterTarget target) {
    switch (target) {
      case NEUTRAL_ZONE:
        return getRobotToneutralZoneVector();
      case AMP_ZONE:
      case AMP_ZONE_SKIP:
        return getRobotToAmpZoneVector();
      case SPEAKER:
      default:
        return getRobotToSpeakerVector();
    }
  }

  public ShooterTarget getShooterTarget() {
    return ShooterTarget.SPEAKER;
    // Translation2d currentPose = drivetrain.getPose().getTranslation();
    // Translation2d fieldSize = new Translation2d(16.54, 0.0);

    // Logger.recordOutput("RobotContainer/CurrentPose", currentPose);
    // if (lastAlliance != Alliance.Blue) {
    //   currentPose = currentPose.minus(fieldSize);
    //   currentPose = new Translation2d(currentPose.getX() * -1.0, currentPose.getY());
    // }
    // Logger.recordOutput("RobotContainer/AlliancePose", currentPose);

    // double NeutralZoneLocation = 6.75;
    // double farWingLocation = 12.0;
    // double centerLineLocation = 4.1;

    // if (currentPose.getX() < NeutralZoneLocation) {
    //   return ShooterTarget.SPEAKER;
    // }
    // if (currentPose.getX() > farWingLocation) {
    //   if (currentPose.getY() < centerLineLocation) {
    //     return ShooterTarget.NEUTRAL_ZONE;
    //   }
    //   return ShooterTarget.NEUTRAL_ZONE_SKIP;
    // }
    // if (currentPose.getY() < centerLineLocation) {
    //   return ShooterTarget.AMP_ZONE;
    // }
    // return ShooterTarget.AMP_ZONE_SKIP;
  }

  public double getDistanceToTargetInches(Translation2d robotToTargetVector) {
    return Units.metersToInches(robotToTargetVector.getNorm()) - 13.5;
  }

  public double getRotationToTargetDegrees(Translation2d robotToTargetVector) {
    return robotToTargetVector.getAngle().getDegrees() + 180.0;
  }

  public Translation2d getRobotToSpeakerVector() {
    Translation2d speakerPose =
        lastAlliance == Alliance.Blue ? blueSpeakerLocation : redSpeakerLocation;
    Translation2d currentPose = drivetrain.getPose().getTranslation();
    return speakerPose.minus(currentPose);
  }

  public Translation2d getRobotToAmpZoneVector() {
    Translation2d ampZone =
        lastAlliance == Alliance.Blue ? blueAmpZoneLocation : redAmpZoneLocation;
    Translation2d currentPose = drivetrain.getPose().getTranslation();
    return ampZone.minus(currentPose);
  }

  public Translation2d getRobotToneutralZoneVector() {
    Translation2d neutralZone =
        lastAlliance == Alliance.Blue ? blueNeutralZoneLocation : redNeutralZoneLocation;
    Translation2d currentPose = drivetrain.getPose().getTranslation();
    return neutralZone.minus(currentPose);
  }

  public boolean isDrivetrainStopped() {
    return drivetrain.getRobotRelativeSpeeds().omegaRadiansPerSecond < 0.1
        && drivetrain.getRobotRelativeSpeeds().vxMetersPerSecond < 0.1
        && drivetrain.getRobotRelativeSpeeds().vyMetersPerSecond < 0.1;
  }

  public boolean readyToShoot() {
    return isDrivetrainStopped()
        && shooterPose.isAtGoal()
        && shooterWheels.isAtSpeed(); // @todo check if vision is aligned
  }

  public void updateVisionPose() {
    LimelightHelpers.PoseEstimate limelightMeasurement = visionApriltagSubsystem.getPoseEstimate();
    if (limelightMeasurement.tagCount >= 2
        || (limelightMeasurement.tagCount == 1 && limelightMeasurement.avgTagDist < 1.25)
        || visionApriltagSubsystem.hasStageTarget()) {
      RobotOdometry.getInstance()
          .addVisionMeasurement(
              limelightMeasurement.pose,
              limelightMeasurement.timestampSeconds,
              VecBuilder.fill(.6, .6, 9999999));
    }
  }

  public double targetAngleHelper(double tx, double latency) {
    latency *= 0.0;
    double curVisionError = tx;
    // treat identical values as stale
    if (lastVisionError == curVisionError) {
      return lastRotateGoal;
    }
    double captureTime = Timer.getFPGATimestamp() - latency;
    double angleAtTime = getInterpolatedAngle(captureTime);

    lastVisionError = curVisionError;
    lastRotateGoal = angleAtTime - curVisionError;
    return lastRotateGoal;
  }

  public double noteDetectionAngleHelper(
      VisionObjectDetectionSubsytem visionObjectDetectionSubsystem) {
    // lets try .. the closer the note is the less correction to take. Adjust tx by
    // the size/height of ty;
    double fovVertical = 49.7; // LL 3 docs

    double tx = visionObjectDetectionSubsystem.getTX();
    double ty = visionObjectDetectionSubsystem.getTY();

    double scale = (ty + fovVertical * 0.75) / (fovVertical) * 1.0;

    return (-1 * scale * tx) + drivetrain.getPose().getRotation().getDegrees();
  }

  public void updateAngleTable() {
    double currentTime = Timer.getFPGATimestamp();
    // Remove oldest value if list size exceeds MAX_SIZE
    if (previousAngles.size() == 10) {
      previousAngles.poll();
    }
    previousAngles.add(
        new AngleTimePair(drivetrain.getPose().getRotation().getDegrees(), currentTime));
    // System.out.println("Angle Table" + previousAngles);
  }

  public synchronized double getInterpolatedAngle(double targetTime) {
    AngleTimePair before = null, after = null;

    // Find the two recorded angles immediately before and after the target time
    for (AngleTimePair pair : previousAngles) {
      if (pair.time <= targetTime) {
        before = pair;
      } else {
        after = pair;
        break;
      }
    }

    // Handle edge cases: no data, target time before first record, or after last
    // record
    if (before == null && after == null) {
      return 0;
    } else if (before == null) {
      return after.angle;
    } else if (after == null) {
      return before.angle;
    }

    // Perform linear interpolation
    double timeDiff = after.time - before.time;
    double weight = (targetTime - before.time) / timeDiff;
    return before.angle + weight * (after.angle - before.angle);
  }

  /** Use this method to define your commands for autonomous mode. */
  private void configureAutoCommands() {
    // Waypoints
    NamedCommands.registerCommand("command1", Commands.print("passed marker 1")); //
    NamedCommands.registerCommand("command2", Commands.print("passed marker 2"));
    NamedCommands.registerCommand(
        "enableXStance", Commands.runOnce(drivetrain::enableXstance, drivetrain));
    NamedCommands.registerCommand(
        "disableXStance", Commands.runOnce(drivetrain::disableXstance, drivetrain));
    NamedCommands.registerCommand(
        "wait5Seconds", Commands.print("passed marker 1")); // Commands.waitSeconds(5.0));
    NamedCommands.registerCommand("SpinShooter", new RunShooterFast(shooterWheels));
    NamedCommands.registerCommand("SpinShooterSlow", new RunShooterSlow(shooterWheels));
    NamedCommands.registerCommand("SpinShooterSpoil", new RunShooterSpoil(shooterWheels));
    NamedCommands.registerCommand("ShootNote", new FeedShooterManual(feeder));
    NamedCommands.registerCommand(
        "IntakeNote", new IntakeNote(intake, feeder, shooterPose, statusRgb));
    NamedCommands.registerCommand(
        "StartIntakingNote", new StartIntakingNote(intake, feeder, shooterPose));
    NamedCommands.registerCommand("StopShooter", new StopShooter(shooterWheels));
    NamedCommands.registerCommand(
        "FinishIntakingNote", new FinishIntakingCommand(intake, feeder, shooterPose));
    NamedCommands.registerCommand(
        "SetShooterDistance115", new SetShooterDistance(shooterPose, 65 + 5));
    NamedCommands.registerCommand(
        "SetShooterDistance125", new SetShooterDistance(shooterPose, 94 - 10));
    NamedCommands.registerCommand(
        "SetShooterDistance150", new SetShooterDistance(shooterPose, 126));
    NamedCommands.registerCommand(
        "SetShooterDistanceFadeaway", new SetShooterDistance(shooterPose, 110 - 30 - 24));
    NamedCommands.registerCommand(
        "SetShooterDistanceFadeawaySource", new SetShooterDistance(shooterPose, 110 - 30));
    NamedCommands.registerCommand(
        "SetShooterDistanceFadeawayAmp", new SetShooterDistance(shooterPose, 110 - 30));
    NamedCommands.registerCommand("SetShooterDistanceF3", new SetShooterDistance(shooterPose, 55));
    NamedCommands.registerCommand(
        "SetShooterDistanceF4", new SetShooterDistance(shooterPose, 115 + 10));
    NamedCommands.registerCommand("SetShooterDistanceF5", new SetShooterDistance(shooterPose, 150));
    NamedCommands.registerCommand(
        "ShootThrough", new ShootThrough(shooterPose, () -> 125.0, intake, feeder));
    NamedCommands.registerCommand(
        "ShootThroughAmp", new ShootThrough(shooterPose, () -> 125.0, intake, feeder));
    NamedCommands.registerCommand(
        "IntakeSpoil",
        new IntakeSpoil(intake, feeder, shooterPose, visionObjectDetectionSubsystem));
    NamedCommands.registerCommand(
        "AimShot",
        new RotateToAngle(
                drivetrain,
                () -> 0.0,
                () -> 0.0,
                () -> 0.0,
                () -> getRotationToTargetDegrees(getRobotToTargetVector()),
                () -> false,
                statusRgb,
                () -> false)
            .deadlineWith(
                // new BrakeFeeder(feeder, shooterWheels).asProxy(),
                new SetShooterDistanceContinuous(
                    shooterPose,
                    () -> getDistanceToTargetInches(getRobotToTargetVector()),
                    () -> ShooterTarget.SPEAKER,
                    () -> false),
                new FunctionalCommand(
                    () -> {}, () -> updateVisionPose(), (temp) -> {}, () -> false)));

    // build auto path commands

    // add commands to the auto chooser
    autoChooser.addOption("Do Nothing", new InstantCommand());

    /************
     * Test Path ************
     *
     * demonstration of PathPlanner path group with event markers
     *
     */
    Command testAmpSide = new PathPlannerAuto("Amp Side Test");
    autoChooser.addDefaultOption("Amp Side 1 then 2", testAmpSide);

    Command testAmpSide2 = new PathPlannerAuto("Amp Side Test 2");
    autoChooser.addDefaultOption("Amp Side 2 then 1", testAmpSide2);

    Command Fadeaway = new PathPlannerAuto("Fadeaway New");
    autoChooser.addOption("Source Side Fadeaway", Fadeaway);

    Command CenterFadeaway = new PathPlannerAuto("Center Fadeaway");
    autoChooser.addOption("Center Fadeaway", CenterFadeaway);

    Command CenterFadeawayBasic = new PathPlannerAuto("Center Fadeaway Basic");
    autoChooser.addOption("Center Fadeaway No Spoil", CenterFadeawayBasic);

    // Command autoTest = new PathPlannerAuto("TestAuto");
    // Command testLine = new PathPlannerAuto("DistanceTest");
    // autoChooser.addOption("Test Auto", autoTest);
    // autoChooser.addOption("Distance Test", testLine);

    /************
     * Start Point ************
     *
     * useful for initializing the pose of the robot to a known location
     *
     */

    // Command startPoint =
    // Commands.runOnce(
    // () ->
    // drivetrain.resetPose(
    //
    // PathPlannerPath.fromPathFile("StartPoint").getPreviewStartingHolonomicPose()),
    // drivetrain);
    // autoChooser.addOption("Start Point", startPoint);

    /************
     * Drive Characterization ************
     *
     * useful for characterizing the swerve modules for driving (i.e, determining kS
     * and kV)
     *
     */
    // autoChooser.addOption(
    // "Swerve Drive Characterization",
    // new FeedForwardCharacterization(
    // drivetrain,
    // true,
    // new FeedForwardCharacterizationData("drive"),
    // drivetrain::runDriveCharacterizationVolts,
    // drivetrain::getDriveCharacterizationVelocity,
    // drivetrain::getDriveCharacterizationAcceleration));

    /************
     * Swerve Rotate Characterization ************
     *
     * useful for characterizing the swerve modules for rotating (i.e, determining
     * kS and kV)
     *
     */
    // autoChooser.addOption(
    // "Swerve Rotate Characterization",
    // new FeedForwardCharacterization(
    // drivetrain,
    // true,
    // new FeedForwardCharacterizationData("rotate"),
    // drivetrain::runRotateCharacterizationVolts,
    // drivetrain::getRotateCharacterizationVelocity,
    // drivetrain::getRotateCharacterizationAcceleration));

    /************
     * Distance Test ************
     *
     * used for empirically determining the wheel diameter
     *
     */
    // Command distanceTestPathCommand = new PathPlannerAuto("DistanceTest");
    // autoChooser.addOption("Distance Path", distanceTestPathCommand);

    /************
     * Auto Tuning ************
     *
     * useful for tuning the autonomous PID controllers
     *
     */
    // Command tuningCommand = new PathPlannerAuto("Tuning");
    // autoChooser.addOption("Auto Tuning", tuningCommand);

    /************
     * Drive Velocity Tuning ************
     *
     * useful for tuning the drive velocity PID controller
     *
     */
    // autoChooser.addOption(
    // "Drive Velocity Tuning",
    // Commands.sequence(
    // Commands.runOnce(drivetrain::disableFieldRelative, drivetrain),
    // Commands.repeatingSequence(
    // Commands.deadline(
    // Commands.waitSeconds(1.0),
    // Commands.run(() -> drivetrain.drive(2.0, 0.0, 0.0, false, false),
    // drivetrain)),
    // Commands.deadline(
    // Commands.waitSeconds(1.0),
    // Commands.run(() -> drivetrain.drive(-0.5, 0.0, 0.0, false, false),
    // drivetrain)),
    // Commands.deadline(
    // Commands.waitSeconds(1.0),
    // Commands.run(() -> drivetrain.drive(1.0, 0.0, 0.0, false, false),
    // drivetrain)),
    // Commands.deadline(
    // Commands.waitSeconds(0.5),
    // Commands.run(() -> drivetrain.drive(3.0, 0.0, 0.0, false, false),
    // drivetrain)),
    // Commands.deadline(
    // Commands.waitSeconds(2.0),
    // Commands.run(() -> drivetrain.drive(1.0, 0.0, 0.0, false, false),
    // drivetrain)),
    // Commands.deadline(
    // Commands.waitSeconds(2.0),
    // Commands.run(() -> drivetrain.drive(-1.0, 0.0, 0.0, false, false),
    // drivetrain)),
    // Commands.deadline(
    // Commands.waitSeconds(0.5),
    // Commands.run(() -> drivetrain.drive(-3.0, 0.0, 0.0, false, false),
    // drivetrain)),
    // Commands.deadline(
    // Commands.waitSeconds(2.0),
    // Commands.run(
    // () -> drivetrain.drive(-1.0, 0.0, 0.0, false, false), drivetrain)))));

    /************
     * Swerve Rotation Tuning ************
     *
     * useful for tuning the swerve module rotation PID controller
     *
     */
    // autoChooser.addOption(
    // "Swerve Rotation Tuning",
    // Commands.sequence(
    // Commands.runOnce(drivetrain::enableFieldRelative, drivetrain),
    // Commands.repeatingSequence(
    // Commands.deadline(
    // Commands.waitSeconds(0.5),
    // Commands.run(() -> drivetrain.drive(0.1, 0.1, 0.0, true, false),
    // drivetrain)),
    // Commands.deadline(
    // Commands.waitSeconds(0.5),
    // Commands.run(() -> drivetrain.drive(-0.1, 0.1, 0.0, true, false),
    // drivetrain)),
    // Commands.deadline(
    // Commands.waitSeconds(0.5),
    // Commands.run(() -> drivetrain.drive(-0.1, -0.1, 0.0, true, false),
    // drivetrain)),
    // Commands.deadline(
    // Commands.waitSeconds(0.5),
    // Commands.run(
    // () -> drivetrain.drive(0.1, -0.1, 0.0, true, false), drivetrain)))));

    Shuffleboard.getTab("MAIN").add(autoChooser.getSendableChooser());
  }

  private void configureDrivetrainCommands() {
    /*-
     * Set up the default command for the drivetrain.
     * The joysticks' values map to percentage of the maximum velocities.
     * The velocities may be specified from either the robot's or field's frame of
     * reference.
     * Robot-centric: +x is forward, +y is left, +theta is CCW
     * Field-centric: origin is down-right, 0deg is up, +x is forward, +y is left,
     * +theta is CCW
     * direction.
     *      ___________
     *      |    |    | ^
     * (0,0).____|____| y, x-> 0->
     */
    drivetrain.setDefaultCommand(
        new TeleopSwerve(drivetrain, oi::getTranslateX, oi::getTranslateY, oi::getRotate));

    // @reference code
    // lock rotation to the nearest 180° while driving
    // oi.getLock180Button()
    // .onTrue(
    // new RotateToAngle(
    // drivetrain,
    // oi::getTranslateX,
    // oi::getTranslateY,
    // () ->
    // (drivetrain.getPose().getRotation().getDegrees() > -90
    // && drivetrain.getPose().getRotation().getDegrees() < 90)
    // ? 0.0
    // : 180.0));

    // field-relative toggle
    oi.fieldCentricButton()
        .toggleOnTrue(
            Commands.either(
                Commands.runOnce(drivetrain::disableFieldRelative, drivetrain),
                Commands.runOnce(drivetrain::enableFieldRelative, drivetrain),
                drivetrain::getFieldRelative));

    // slow-mode toggle
    oi.slowModeSwitch()
        .onTrue(
            Commands.runOnce(
                () -> {
                  drivetrain.enableTranslationSlowMode();
                  drivetrain.enableRotationSlowMode();
                },
                drivetrain));
    oi.slowModeSwitch()
        .onFalse(
            Commands.runOnce(
                () -> {
                  drivetrain.disableTranslationSlowMode();
                  drivetrain.disableRotationSlowMode();
                },
                drivetrain));

    // reset gyro to 0 degrees
    oi.resetGyroButton()
        .onTrue(
            Commands.runOnce(
                    () ->
                        drivetrain.resetPose(
                            new Pose2d(
                                drivetrain.getPose().getTranslation(),
                                Rotation2d.fromDegrees(lastAlliance == Alliance.Blue ? 0 : 180))),
                    drivetrain)
                // .andThen(Commands.runOnce(drivetrain::zeroGyroscope, drivetrain)));
                .andThen(
                    Commands.runOnce(
                        () -> drivetrain.setGyroOffset(lastAlliance == Alliance.Blue ? 0 : 180),
                        drivetrain)));

    // @reference code
    // reset pose based on vision
    // oi.resetPoseToVisionButton()
    // .onTrue(
    // Commands.runOnce(() -> drivetrain.resetPoseToVision(() ->
    // vision.getBestRobotPose())));
  }

  public void disablePeriodic() {
    shooterPose.resetToAbsoluteEncoder();
  }

  private void configureSubsystemCommands() {
    // FIXME: add commands for the subsystem
  }

  // private void configureVisionCommands() {
  // // enable/disable vision
  // oi.getVisionIsEnabledSwitch().onTrue(Commands.runOnce(() ->
  // vision.enable(true)));
  // oi.getVisionIsEnabledSwitch()
  // .onFalse(
  // Commands.parallel(
  // Commands.runOnce(() -> vision.enable(false), vision),
  // Commands.runOnce(drivetrain::resetPoseRotationToGyro)));
  // }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }

  /**
   * Check if the alliance color has changed; if so, update the vision subsystem and Field2d
   * singleton.
   */
  public void checkAllianceColor() {
    Optional<Alliance> alliance = DriverStation.getAlliance();
    if (alliance.isPresent() && alliance.get() != lastAlliance) {
      this.lastAlliance = alliance.get();
      this.drivetrain.updateAlliance(this.lastAlliance);
    }
  }

  public DriverStation.Alliance getAllianceColor() {
    return lastAlliance;
  }
}
