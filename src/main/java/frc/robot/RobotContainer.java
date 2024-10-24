// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.team3061.RobotConfig;
import frc.lib.team3061.drivetrain.Drivetrain;
import frc.lib.team3061.drivetrain.DrivetrainIOCTRE;
import frc.lib.team3061.drivetrain.swerve.SwerveModuleIO;
import frc.lib.team3061.drivetrain.swerve.SwerveModuleIOTalonFXPhoenix6;
import frc.lib.team3061.gyro.GyroIO;
import frc.lib.team3061.gyro.GyroIOPigeon2Phoenix6;
import frc.lib.team3061.leds.LEDs;
import frc.robot.commands.FeedForwardCharacterization;
import frc.robot.commands.FeedForwardCharacterization.FeedForwardCharacterizationData;
import frc.robot.commands.RotateToAngle;
import frc.robot.commands.TeleopSwerve;
import frc.robot.commands.feederCommands.BrakeFeeder;
import frc.robot.commands.feederCommands.FeedShooterManual;
import frc.robot.commands.intakeCommands.IntakeNote;
import frc.robot.commands.intakeCommands.IntakeSourceNote;
import frc.robot.commands.shooterCommands.RunShooterFast;
import frc.robot.commands.shooterCommands.RunShooterSlow;
import frc.robot.commands.shooterCommands.SetShooterDistanceContinuous;
import frc.robot.commands.shooterCommands.SetShooterPose;
import frc.robot.commands.shooterCommands.StopShooter;
import frc.robot.configs.DefaultRobotConfig;
import frc.robot.limelightVision.VisionSubsystem;
import frc.robot.operator_interface.OISelector;
import frc.robot.operator_interface.OperatorInterface;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooterPose.Pose;
import frc.robot.subsystems.shooterPose.ShooterPose;
import frc.robot.subsystems.shooterWheels.ShooterWheels;
import java.util.Optional;
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
  private Alliance lastAlliance = DriverStation.Alliance.Red;
  private VisionSubsystem visionSubsystem;
  public Intake intake;
  public Feeder feeder;
  public ShooterWheels shooterWheels;
  public ShooterPose shooterPose;

  public enum ScoringMode {
    AMP,
    SPEAKER
  }

  public ScoringMode scoringMode = ScoringMode.AMP;

  // use AdvantageKit's LoggedDashboardChooser instead of SendableChooser to ensure accurate logging
  private final LoggedDashboardChooser<Command> autoChooser =
      new LoggedDashboardChooser<>("Auto Routine");

  private final LoggedDashboardNumber endgameAlert1 =
      new LoggedDashboardNumber("Endgame Alert #1", 20.0);
  private final LoggedDashboardNumber endgameAlert2 =
      new LoggedDashboardNumber("Endgame Alert #2", 10.0);

  // RobotContainer singleton
  private static RobotContainer robotContainer = new RobotContainer();

  /**
   * Create the container for the robot. Contains subsystems, operator interface (OI) devices, and
   * commands.
   */
  public RobotContainer() {
    /*
     * IMPORTANT: The RobotConfig subclass object *must* be created before any other objects
     * that use it directly or indirectly. If this isn't done, a null pointer exception will result.
     */
    createRobotConfig();

    LEDs.getInstance();

    createSubsystems();

    // disable all telemetry in the LiveWindow to reduce the processing during each iteration
    LiveWindow.disableAllTelemetry();

    constructField();

    updateOI();

    configureAutoCommands();

    NamedCommands.registerCommand("SpinShooter", new PrintCommand("Spin Shooter Command"));
    NamedCommands.registerCommand("ShootNote", new PrintCommand("Shoot Note Command"));
    NamedCommands.registerCommand("IntakeNote", new PrintCommand("Intake Note Command"));
    NamedCommands.registerCommand(
        "SetShooterDistance115", new PrintCommand("Set Shooter Distance 115 Command"));
    NamedCommands.registerCommand(
        "SetShooterDistance125", new PrintCommand("Set Shooter Distance 125 Command"));
    NamedCommands.registerCommand(
        "SetShooterDistance150", new PrintCommand("Set Shooter Distance 150 Command"));
  }

  /**
   * The RobotConfig subclass object *must* be created before any other objects that use it directly
   * or indirectly. If this isn't done, a null pointer exception will result.
   */
  private void createRobotConfig() {
    config = new DefaultRobotConfig();
  }

  private void createSubsystems() {
    int[] driveMotorCANIDs = config.getSwerveDriveMotorCANIDs();
    int[] steerMotorCANDIDs = config.getSwerveSteerMotorCANIDs();
    int[] steerEncoderCANDIDs = config.getSwerveSteerEncoderCANIDs();
    double[] steerOffsets = config.getSwerveSteerOffsets();
    SwerveModuleIO flModule =
        new SwerveModuleIOTalonFXPhoenix6(
            0, driveMotorCANIDs[0], steerMotorCANDIDs[0], steerEncoderCANDIDs[0], steerOffsets[0]);

    SwerveModuleIO frModule =
        new SwerveModuleIOTalonFXPhoenix6(
            1, driveMotorCANIDs[1], steerMotorCANDIDs[1], steerEncoderCANDIDs[1], steerOffsets[1]);

    SwerveModuleIO blModule =
        new SwerveModuleIOTalonFXPhoenix6(
            2, driveMotorCANIDs[2], steerMotorCANDIDs[2], steerEncoderCANDIDs[2], steerOffsets[2]);

    SwerveModuleIO brModule =
        new SwerveModuleIOTalonFXPhoenix6(
            3, driveMotorCANIDs[3], steerMotorCANDIDs[3], steerEncoderCANDIDs[3], steerOffsets[3]);

    GyroIO gyro = new GyroIOPigeon2Phoenix6(config.getGyroCANID());
    // DrivetrainIO drivetrainIO =
    //     new DrivetrainIOGeneric(gyro, flModule, frModule, blModule, brModule);
    DrivetrainIOCTRE drivetrainIO = new DrivetrainIOCTRE();
    drivetrain = new Drivetrain(drivetrainIO);

    intake = new Intake();
    feeder = new Feeder();
    shooterWheels = new ShooterWheels();
    shooterPose = new ShooterPose();

    visionSubsystem = new VisionSubsystem();

    //   String[] cameraNames = config.getCameraNames(); //TODO: Uncomment Camera stuff
    //   Transform3d[] robotToCameraTransforms = config.getRobotToCameraTransforms();
    //   VisionIO[] visionIOs = new VisionIO[cameraNames.length];
    //   AprilTagFieldLayout layout;
    //   try {
    //     layout = new AprilTagFieldLayout(VisionConstants.APRILTAG_FIELD_LAYOUT_PATH);
    //   } catch (IOException e) {
    //     layout = new AprilTagFieldLayout(new ArrayList<>(), 16.4592, 8.2296);
    //   }
    //   for (int i = 0; i < visionIOs.length; i++) {
    //     visionIOs[i] = new VisionIOPhotonVision(cameraNames[i], layout,
    // robotToCameraTransforms[i]);
    //   }
    //   vision = new Vision(visionIOs);
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
    oi.aimOrSourceButton()
        .whileTrue(
            new ConditionalCommand(
                new ConditionalCommand(
                    // Has note AND is in AMP scoring mode
                    new RunShooterSlow(shooterWheels)
                        .andThen(
                            new SetShooterPose(shooterPose, Pose.AMP)
                                .asProxy()
                                .alongWith(
                                    new RotateToAngle(
                                            drivetrain,
                                            oi::getTranslateX,
                                            oi::getTranslateY,
                                            oi::getRotate,
                                            () -> this.lastAlliance == Alliance.Blue ? 90 : -90,
                                            () -> false)
                                        .asProxy())),
                    // Has note AND is in SPEAKER scoring mode
                    new RunShooterFast(shooterWheels)
                        .andThen(
                            new SetShooterDistanceContinuous(
                                    shooterPose,
                                    () ->
                                        visionSubsystem.hasTarget()
                                            ? visionSubsystem.getDistanceToTarget()
                                            : 105)
                                .asProxy()
                                .alongWith(
                                    new BrakeFeeder(feeder, shooterWheels).asProxy(),
                                    new RotateToAngle(
                                            drivetrain,
                                            oi::getTranslateX,
                                            oi::getTranslateY,
                                            oi::getRotate,
                                            () ->
                                                drivetrain.getPose().getRotation().getDegrees()
                                                    + visionSubsystem.getTX(),
                                            () -> !visionSubsystem.hasTarget())
                                        .asProxy())),
                    // Check ScoringMode
                    () -> scoringMode == ScoringMode.AMP),
                // Does NOT have note
                new IntakeSourceNote(feeder, shooterPose).asProxy(),
                // Check hasNote
                feeder::hasNote));

    oi.aimOrSourceButton()
        .onFalse(
            new StopShooter(shooterWheels).andThen(new SetShooterPose(shooterPose, Pose.HANDOFF)));

    oi.IntakeOrScoreButton()
        .whileTrue(
            new ConditionalCommand(
                new FeedShooterManual(feeder).asProxy(),
                new IntakeNote(intake, feeder, shooterPose).asProxy(),
                feeder::hasNote));

    oi.ampModeButton().onTrue(new InstantCommand(() -> scoringMode = ScoringMode.AMP));

    oi.speakerModeButton().onTrue(new InstantCommand(() -> scoringMode = ScoringMode.SPEAKER));

    // oi.groundIntakeButton().whileTrue(new IntakeNote(intake, feeder, shooterPose));

    // oi.sourceLoadButton().whileTrue(new IntakeSourceNote(feeder, shooterPose));

    // oi.ampScoreButton()
    //     .whileTrue(
    //         new RunShooterSlow(shooterWheels).andThen(new SetShooterPose(shooterPose,
    // Pose.AMP)));
    // oi.ampScoreButton()
    //     .onFalse(
    //         new StopShooter(shooterWheels).andThen(new SetShooterPose(shooterPose,
    // Pose.HANDOFF)));

    // oi.aimSpeakerButton()
    //     .whileTrue(
    //         new RunShooterFast(shooterWheels)
    //             .andThen(
    //                 new SetShooterDistanceContinuous(
    //                         shooterPose,
    //                         () ->
    //                             visionSubsystem.hasTarget()
    //                                 ? visionSubsystem.getDistanceToTarget()
    //                                 : 105)
    //                     .alongWith(
    //                         new BrakeFeeder(feeder, shooterWheels),
    //                         new RotateToAngle(
    //                             drivetrain,
    //                             oi::getTranslateX,
    //                             oi::getTranslateY,
    //                             oi::getRotate,
    //                             () ->
    //                                 drivetrain.getPose().getRotation().getDegrees()
    //                                     + visionSubsystem.getTX(),
    //                             () -> !visionSubsystem.hasTarget()))));

    // oi.aimSpeakerButton()
    //     .onFalse(
    //         new StopShooter(shooterWheels).andThen(new SetShooterPose(shooterPose,
    // Pose.HANDOFF)));

    // oi.smartFeedButton().whileTrue(new FeedShooterManual(feeder));

    // oi.manualFeedButton().whileTrue(new FeedShooterManual(feeder));

    // oi.ejectButton().onTrue(new Eject(feeder, intake, shooterWheels));

    oi.feedThroughButton().whileTrue(new FeedShooterManual(feeder));

    configureDrivetrainCommands();

    configureSubsystemCommands();

    // Endgame alerts
    new Trigger(
            () ->
                DriverStation.isTeleopEnabled()
                    && DriverStation.getMatchTime() > 0.0
                    && DriverStation.getMatchTime() <= Math.round(endgameAlert1.get()))
        .onTrue(
            Commands.run(() -> LEDs.getInstance().setEndgameAlert(true))
                .withTimeout(1.5)
                .andThen(
                    Commands.run(() -> LEDs.getInstance().setEndgameAlert(false))
                        .withTimeout(1.0)));
    new Trigger(
            () ->
                DriverStation.isTeleopEnabled()
                    && DriverStation.getMatchTime() > 0.0
                    && DriverStation.getMatchTime() <= Math.round(endgameAlert2.get()))
        .onTrue(
            Commands.sequence(
                Commands.run(() -> LEDs.getInstance().setEndgameAlert(true)).withTimeout(0.5),
                Commands.run(() -> LEDs.getInstance().setEndgameAlert(false)).withTimeout(0.5),
                Commands.run(() -> LEDs.getInstance().setEndgameAlert(true)).withTimeout(0.5),
                Commands.run(() -> LEDs.getInstance().setEndgameAlert(false)).withTimeout(1.0)));
  }

  /** Use this method to define your commands for autonomous mode. */
  private void configureAutoCommands() {
    // Waypoints
    NamedCommands.registerCommand("command1", Commands.print("passed marker 1"));
    NamedCommands.registerCommand("command2", Commands.print("passed marker 2"));
    NamedCommands.registerCommand(
        "enableXStance", Commands.runOnce(drivetrain::enableXstance, drivetrain));
    NamedCommands.registerCommand(
        "disableXStance", Commands.runOnce(drivetrain::disableXstance, drivetrain));
    NamedCommands.registerCommand("wait5Seconds", Commands.waitSeconds(5.0));

    // build auto path commands

    // add commands to the auto chooser
    autoChooser.addDefaultOption("Do Nothing", new InstantCommand());

    /************ Test Path ************
     *
     * demonstration of PathPlanner path group with event markers
     *
     */
    Command autoTest = new PathPlannerAuto("TestAuto");
    Command testLine = new PathPlannerAuto("DistanceTest");
    autoChooser.addOption("Test Auto", autoTest);
    autoChooser.addOption("Distance Test", testLine);

    /************ Start Point ************
     *
     * useful for initializing the pose of the robot to a known location
     *
     */

    Command startPoint =
        Commands.runOnce(
            () ->
                drivetrain.resetPose(
                    PathPlannerPath.fromPathFile("StartPoint").getPreviewStartingHolonomicPose()),
            drivetrain);
    autoChooser.addOption("Start Point", startPoint);

    /************ Drive Characterization ************
     *
     * useful for characterizing the swerve modules for driving (i.e, determining kS and kV)
     *
     */
    autoChooser.addOption(
        "Swerve Drive Characterization",
        new FeedForwardCharacterization(
            drivetrain,
            true,
            new FeedForwardCharacterizationData("drive"),
            drivetrain::runDriveCharacterizationVolts,
            drivetrain::getDriveCharacterizationVelocity,
            drivetrain::getDriveCharacterizationAcceleration));

    /************ Swerve Rotate Characterization ************
     *
     * useful for characterizing the swerve modules for rotating (i.e, determining kS and kV)
     *
     */
    autoChooser.addOption(
        "Swerve Rotate Characterization",
        new FeedForwardCharacterization(
            drivetrain,
            true,
            new FeedForwardCharacterizationData("rotate"),
            drivetrain::runRotateCharacterizationVolts,
            drivetrain::getRotateCharacterizationVelocity,
            drivetrain::getRotateCharacterizationAcceleration));

    /************ Distance Test ************
     *
     * used for empirically determining the wheel diameter
     *
     */
    Command distanceTestPathCommand = new PathPlannerAuto("DistanceTest");
    autoChooser.addOption("Distance Path", distanceTestPathCommand);

    /************ Auto Tuning ************
     *
     * useful for tuning the autonomous PID controllers
     *
     */
    Command tuningCommand = new PathPlannerAuto("Tuning");
    autoChooser.addOption("Auto Tuning", tuningCommand);

    /************ Drive Velocity Tuning ************
     *
     * useful for tuning the drive velocity PID controller
     *
     */
    autoChooser.addOption(
        "Drive Velocity Tuning",
        Commands.sequence(
            Commands.runOnce(drivetrain::disableFieldRelative, drivetrain),
            Commands.repeatingSequence(
                Commands.deadline(
                    Commands.waitSeconds(1.0),
                    Commands.run(() -> drivetrain.drive(2.0, 0.0, 0.0, false, false), drivetrain)),
                Commands.deadline(
                    Commands.waitSeconds(1.0),
                    Commands.run(() -> drivetrain.drive(-0.5, 0.0, 0.0, false, false), drivetrain)),
                Commands.deadline(
                    Commands.waitSeconds(1.0),
                    Commands.run(() -> drivetrain.drive(1.0, 0.0, 0.0, false, false), drivetrain)),
                Commands.deadline(
                    Commands.waitSeconds(0.5),
                    Commands.run(() -> drivetrain.drive(3.0, 0.0, 0.0, false, false), drivetrain)),
                Commands.deadline(
                    Commands.waitSeconds(2.0),
                    Commands.run(() -> drivetrain.drive(1.0, 0.0, 0.0, false, false), drivetrain)),
                Commands.deadline(
                    Commands.waitSeconds(2.0),
                    Commands.run(() -> drivetrain.drive(-1.0, 0.0, 0.0, false, false), drivetrain)),
                Commands.deadline(
                    Commands.waitSeconds(0.5),
                    Commands.run(() -> drivetrain.drive(-3.0, 0.0, 0.0, false, false), drivetrain)),
                Commands.deadline(
                    Commands.waitSeconds(2.0),
                    Commands.run(
                        () -> drivetrain.drive(-1.0, 0.0, 0.0, false, false), drivetrain)))));

    /************ Swerve Rotation Tuning ************
     *
     * useful for tuning the swerve module rotation PID controller
     *
     */
    autoChooser.addOption(
        "Swerve Rotation Tuning",
        Commands.sequence(
            Commands.runOnce(drivetrain::enableFieldRelative, drivetrain),
            Commands.repeatingSequence(
                Commands.deadline(
                    Commands.waitSeconds(0.5),
                    Commands.run(() -> drivetrain.drive(0.1, 0.1, 0.0, true, false), drivetrain)),
                Commands.deadline(
                    Commands.waitSeconds(0.5),
                    Commands.run(() -> drivetrain.drive(-0.1, 0.1, 0.0, true, false), drivetrain)),
                Commands.deadline(
                    Commands.waitSeconds(0.5),
                    Commands.run(() -> drivetrain.drive(-0.1, -0.1, 0.0, true, false), drivetrain)),
                Commands.deadline(
                    Commands.waitSeconds(0.5),
                    Commands.run(
                        () -> drivetrain.drive(0.1, -0.1, 0.0, true, false), drivetrain)))));

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
    //     .onTrue(
    //         new RotateToAngle(
    //             drivetrain,
    //             oi::getTranslateX,
    //             oi::getTranslateY,
    //             () ->
    //                 (drivetrain.getPose().getRotation().getDegrees() > -90
    //                         && drivetrain.getPose().getRotation().getDegrees() < 90)
    //                     ? 0.0
    //                     : 180.0));

    // field-relative toggle
    oi.fieldCentricButton()
        .toggleOnTrue(
            Commands.either(
                Commands.runOnce(drivetrain::disableFieldRelative, drivetrain),
                Commands.runOnce(drivetrain::enableFieldRelative, drivetrain),
                drivetrain::getFieldRelative));

    // slow-mode toggle
    oi.slowModeSwitch().onTrue(Commands.runOnce(drivetrain::enableTranslationSlowMode, drivetrain));
    oi.slowModeSwitch()
        .onFalse(Commands.runOnce(drivetrain::disableTranslationSlowMode, drivetrain));

    // reset gyro to 0 degrees
    oi.resetGyroButton().onTrue(Commands.runOnce(drivetrain::zeroGyroscope, drivetrain));

    // @reference code
    // reset pose based on vision
    // oi.resetPoseToVisionButton()
    //     .onTrue(
    //         Commands.runOnce(() -> drivetrain.resetPoseToVision(() ->
    // vision.getBestRobotPose())));
  }

  private void configureSubsystemCommands() {
    // FIXME: add commands for the subsystem
  }

  // private void configureVisionCommands() {
  //   // enable/disable vision
  //   oi.getVisionIsEnabledSwitch().onTrue(Commands.runOnce(() -> vision.enable(true)));
  //   oi.getVisionIsEnabledSwitch()
  //       .onFalse(
  //           Commands.parallel(
  //               Commands.runOnce(() -> vision.enable(false), vision),
  //               Commands.runOnce(drivetrain::resetPoseRotationToGyro)));
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
}
