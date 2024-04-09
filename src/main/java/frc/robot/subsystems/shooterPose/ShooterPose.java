package frc.robot.subsystems.shooterPose;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkLimitSwitch;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.Map;
import java.util.NavigableMap;
import java.util.TreeMap;
import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;

public class ShooterPose extends SubsystemBase {

  @AutoLog
  public static class ShooterPoseTiltIOInput {
    double Angle = 0.0;
    double AngleVelocity = 0.0;
    double AbsoluteAngle = 0.0;
    double AngleGoal = 0.0;
    double AngleSetpoint = 0.0;
    double AngleSpeed = 0.0;
    double AngleFeedforward = 0.0;
    boolean AbsoluteIsConnected = false;
  }

  @AutoLog
  public static class ShooterPoseHeightIOInput {
    double Height = 0.0;
    double HeightVelocity = 0.0;
    double HeightGoal = 0.0;
    double HeightSetpoint = 0.0;
    double HeightSpeed = 0.0;
    double HeightFeedforward = 0.0;
  }

  @AutoLog
  public static class ShooterPoseIOInput {
    Pose pose = Pose.HANDOFF;
  }

  private final ShooterPoseTiltIOInputAutoLogged loggedTiltIO =
      new ShooterPoseTiltIOInputAutoLogged();
  private final ShooterPoseHeightIOInputAutoLogged loggedHeightIO =
      new ShooterPoseHeightIOInputAutoLogged();
  private final ShooterPoseIOInputAutoLogged loggedPoseIO = new ShooterPoseIOInputAutoLogged();

  private CANSparkMax shooterHeightLeftMotor;
  private CANSparkMax shooterHeightRightMotor;
  private SparkLimitSwitch shooterHeightLimitSwitch;
  private RelativeEncoder shooterHeightEncoder;

  private CANSparkMax shooterTiltMotor;
  private RelativeEncoder shooterTiltEncoder;
  private DutyCycleEncoder shooterTiltAbsoluteEncoder;

  private ProfiledPIDController shooterHeightPID;
  private ElevatorFeedforward shooterHeightFeedforward;
  private ProfiledPIDController shooterTiltPID;
  private ArmFeedforward shooterTiltFeedforward;

  private ShuffleboardTab shooterPoseTab;

  private GenericEntry goalEntry;

  private GenericEntry maxVelocityEntry, maxAccelerationEntry;
  private GenericEntry shooterHeightP, shooterHeightI, shooterHeightD;
  private GenericEntry shooterTiltP, shooterTiltI, shooterTiltD;

  private double shooterDistance;

  private int limitSwitchCounter;
  private boolean elevatorPIDOverride;
  private boolean encoderReset;

  private NavigableMap<Double, Double> angleLookupTable;
  private NavigableMap<Double, Double> popShotAngleLookupTable;

  // private final TunableNumber shooterHeightP =
  // new TunableNumber("Elevator Height P",
  // ShooterPoseConstants.SHOOTER_HEIGHT_KP);
  // private final TunableNumber shooterHeightI =
  // new TunableNumber("Elevator Height I",
  // ShooterPoseConstants.SHOOTER_HEIGHT_KI);
  // private final TunableNumber shooterHeightD =
  // new TunableNumber("Elevator Height D",
  // ShooterPoseConstants.SHOOTER_HEIGHT_KD);

  // private final TunableNumber shooterTiltP =
  // new TunableNumber("Shooter Tilt P", ShooterPoseConstants.SHOOTER_TILT_KP);
  // private final TunableNumber shooterTiltP =
  // new TunableNumber("Shooter Tilt I", ShooterPoseConstants.SHOOTER_TILT_KI);
  // private final TunableNumber shooterTiltD =
  // new TunableNumber("Shooter Tilt D", ShooterPoseConstants.SHOOTER_TILT_KD);

  public ShooterPose() {
    angleLookupTable = new TreeMap<>();
    angleLookupTable.put(10.0, -47.0); // (distance, optimal angle)
    angleLookupTable.put(31.0, -47.0); // subwoofer, min angle
    angleLookupTable.put(50.0, -43.0);
    angleLookupTable.put(60.0, -39.0);
    angleLookupTable.put(70.0, -33.5);
    angleLookupTable.put(80.0, -30.5);
    angleLookupTable.put(90.0, -29.5);
    angleLookupTable.put(100.0, -28.0);
    angleLookupTable.put(110.0, -27.5);
    angleLookupTable.put(120.0, -26.25);
    angleLookupTable.put(130.0, -25.5);
    angleLookupTable.put(140.0, -25.0);
    angleLookupTable.put(150.0, -24.5);
    angleLookupTable.put(250.0, -20.5);

    popShotAngleLookupTable = new TreeMap<>();
    popShotAngleLookupTable.put(10.0, -43.0); // (distance, optimal angle)
    popShotAngleLookupTable.put(31.0, -43.0); // subwoofer, min angle
    popShotAngleLookupTable.put(50.0, -37.0);
    popShotAngleLookupTable.put(60.0, -29.0);
    popShotAngleLookupTable.put(70.0, -27.5);
    popShotAngleLookupTable.put(80.0, -26.5);
    popShotAngleLookupTable.put(90.0, -25.5);
    popShotAngleLookupTable.put(100.0, -24.5);
    popShotAngleLookupTable.put(110.0, -23.5);
    popShotAngleLookupTable.put(120.0, -23.0);
    popShotAngleLookupTable.put(130.0, -22.5);
    popShotAngleLookupTable.put(140.0, -22.0);
    popShotAngleLookupTable.put(150.0, -21.5);
    popShotAngleLookupTable.put(250.0, -20.5);

    shooterHeightLeftMotor =
        new CANSparkMax(
            ShooterPoseConstants.SHOOTER_HEIGHT_LEFT_MOTOR_CAN_ID, MotorType.kBrushless);
    shooterHeightRightMotor =
        new CANSparkMax(
            ShooterPoseConstants.SHOOTER_HEIGHT_RIGHT_MOTOR_CAN_ID, MotorType.kBrushless);

    shooterHeightRightMotor.restoreFactoryDefaults();
    shooterHeightLeftMotor.restoreFactoryDefaults();

    // leader
    shooterHeightRightMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 20);
    shooterHeightRightMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 50);
    shooterHeightRightMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 200);
    shooterHeightRightMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 300);

    // follower
    shooterHeightLeftMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 100);
    shooterHeightLeftMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 200);
    shooterHeightLeftMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 500);
    shooterHeightLeftMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 700);

    Timer.delay(0.050);

    shooterHeightRightMotor.setInverted(true);

    shooterHeightRightMotor.enableVoltageCompensation(12);
    shooterHeightLeftMotor.enableVoltageCompensation(12);

    shooterHeightRightMotor.setIdleMode(IdleMode.kCoast);
    shooterHeightLeftMotor.setIdleMode(IdleMode.kCoast);

    shooterHeightLeftMotor.follow(shooterHeightRightMotor, true);

    shooterHeightRightMotor.enableSoftLimit(SoftLimitDirection.kForward, true);
    shooterHeightRightMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);
    shooterHeightRightMotor.setSoftLimit(
        SoftLimitDirection.kForward,
        (float) angleModulusDeg(ShooterPoseConstants.MAX_SHOOTER_HEIGHT_INCHES));
    shooterHeightRightMotor.setSoftLimit(
        SoftLimitDirection.kReverse,
        (float) angleModulusDeg(ShooterPoseConstants.MIN_SHOOTER_HEIGHT_INCHES));

    shooterHeightLimitSwitch =
        shooterHeightRightMotor.getReverseLimitSwitch(SparkLimitSwitch.Type.kNormallyOpen);
    shooterHeightLimitSwitch.enableLimitSwitch(true);

    shooterHeightEncoder = shooterHeightRightMotor.getEncoder();
    shooterHeightEncoder.setPositionConversionFactor(
        ShooterPoseConstants.SHOOTER_HEIGHT_INCHES_PER_ROTATION);
    shooterHeightEncoder.setVelocityConversionFactor(
        ShooterPoseConstants.SHOOTER_HEIGHT_RPM_TO_INCHES_PER_SECOND);
    shooterHeightEncoder.setPosition(0);

    shooterHeightPID =
        new ProfiledPIDController(
            ShooterPoseConstants.SHOOTER_HEIGHT_KP,
            ShooterPoseConstants.SHOOTER_HEIGHT_KI,
            ShooterPoseConstants.SHOOTER_HEIGHT_KD,
            new TrapezoidProfile.Constraints(
                ShooterPoseConstants.SHOOTER_HEIGHT_MAX_VELOCITY,
                ShooterPoseConstants.SHOOTER_HEIGHT_MAX_ACCELERATION),
            ShooterPoseConstants.SHOOTER_PID_PERIOD_SEC);
    shooterHeightPID.setTolerance(ShooterPoseConstants.SHOOTER_HEIGHT_GOAL_TOLERANCE_INCHES);
    shooterHeightPID.reset(shooterHeightEncoder.getPosition());
    shooterHeightPID.setGoal(0);

    shooterHeightFeedforward =
        new ElevatorFeedforward(
            ShooterPoseConstants.SHOOTER_HEIGHT_KS,
            ShooterPoseConstants.SHOOTER_HEIGHT_KG,
            ShooterPoseConstants.SHOOTER_HEIGHT_KV,
            ShooterPoseConstants.SHOOTER_HEIGHT_KA);

    shooterTiltMotor =
        new CANSparkMax(ShooterPoseConstants.SHOOTER_TILT_MOTOR_CAN_ID, MotorType.kBrushless);

    shooterTiltMotor.restoreFactoryDefaults();

    shooterTiltMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 20);
    shooterTiltMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 20);
    shooterTiltMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 50);
    shooterTiltMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 100);

    Timer.delay(0.050);
    shooterTiltMotor.setInverted(false);
    shooterTiltMotor.enableVoltageCompensation(12);
    shooterTiltMotor.setIdleMode(IdleMode.kCoast);

    shooterTiltMotor.enableSoftLimit(SoftLimitDirection.kForward, true);
    shooterTiltMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);
    shooterTiltMotor.setSoftLimit(
        SoftLimitDirection.kForward, (float) ShooterPoseConstants.MAX_SHOOTER_TILT_DEGREES);
    shooterTiltMotor.setSoftLimit(
        SoftLimitDirection.kReverse, (float) ShooterPoseConstants.MIN_SHOOTER_TILT_DEGREES);

    shooterTiltAbsoluteEncoder = new DutyCycleEncoder(8);

    shooterTiltEncoder = shooterTiltMotor.getEncoder();
    shooterTiltEncoder.setPositionConversionFactor(
        ShooterPoseConstants.SHOOTER_TILT_DEGREES_PER_ROTATION);
    shooterTiltEncoder.setVelocityConversionFactor(
        ShooterPoseConstants.SHOOTER_TILT_RPM_TO_DEGREES_PER_SECOND);
    shooterTiltEncoder.setPosition(angleModulusDeg(-44.44));

    shooterTiltPID =
        new ProfiledPIDController(
            ShooterPoseConstants.SHOOTER_TILT_KP,
            ShooterPoseConstants.SHOOTER_TILT_KI,
            ShooterPoseConstants.SHOOTER_TILT_KD,
            new TrapezoidProfile.Constraints(
                ShooterPoseConstants.SHOOTER_TILT_MAX_VELOCITY,
                ShooterPoseConstants.SHOOTER_TILT_MAX_ACCELERATION),
            ShooterPoseConstants.SHOOTER_PID_PERIOD_SEC);
    shooterTiltPID.setTolerance(ShooterPoseConstants.SHOOTER_TILT_GOAL_TOLERANCE_DEGREES);
    shooterTiltPID.reset(shooterTiltEncoder.getPosition());
    shooterTiltPID.setGoal(ShooterPoseConstants.SHOOTER_TILT_HANDOFF_SETPOINT);

    shooterTiltFeedforward =
        new ArmFeedforward(
            ShooterPoseConstants.SHOOTER_TILT_KS,
            ShooterPoseConstants.SHOOTER_TILT_KG,
            ShooterPoseConstants.SHOOTER_TILT_KV,
            ShooterPoseConstants.SHOOTER_TILT_KA);

    encoderReset = false;

    shooterDistance = 0;

    if (ShooterPoseConstants.SHOOTER_POSE_TESTING) {
      setUpShuffleboard();
    }

    setUpShuffleboard();

    shooterHeightRightMotor.stopMotor();
    shooterTiltMotor.stopMotor();

    /*Timer.delay(0.25);
    shooterHeightRightMotor.burnFlash();
    Timer.delay(0.25);
    shooterHeightLeftMotor.burnFlash();
    Timer.delay(0.25);
    shooterTiltMotor.burnFlash();
    Timer.delay(0.25);*/
  }

  public void setPopShotDistance(double distanceInches) {
    if (distanceInches <= 0) {
      return;
    }

    shooterDistance = distanceInches;
    double speakerHeight = 83, shooterHeight = 27 + 14.75;
    double basicAngle =
        -1 * Math.toDegrees(Math.atan((speakerHeight - shooterHeight) / distanceInches));

    // Find the closest distances in the lookup table
    Double lowerDistance = popShotAngleLookupTable.floorKey(distanceInches);
    Double higherDistance = popShotAngleLookupTable.ceilingKey(distanceInches);

    double targetAngle;

    // If exact match or only one boundary exists, use it directly
    if (lowerDistance == null || higherDistance == null || lowerDistance.equals(higherDistance)) {
      targetAngle = popShotAngleLookupTable.getOrDefault(distanceInches, basicAngle);
    } else {

      // Interpolate between the two angles for a more accurate angle
      double lowerAngle = popShotAngleLookupTable.get(lowerDistance);
      double higherAngle = popShotAngleLookupTable.get(higherDistance);
      targetAngle =
          interpolate(distanceInches, lowerDistance, higherDistance, lowerAngle, higherAngle);
    }
    shooterHeightPID.setGoal(ShooterPoseConstants.SHOOTER_HEIGHT_TRAP_SETPOINT);

    shooterTiltPID.setGoal(
        MathUtil.clamp(
            angleModulusDeg(targetAngle),
            ShooterPoseConstants.MIN_SHOOTER_TILT_DEGREES,
            ShooterPoseConstants.MAX_SHOOTER_TILT_DEGREES));
  }

  public void setShooterDistance(double distanceInches) {
    if (distanceInches <= 0) {
      return;
    }

    shooterDistance = distanceInches;
    double speakerHeight = 83, shooterHeight = 27;
    double basicAngle =
        -1 * Math.toDegrees(Math.atan((speakerHeight - shooterHeight) / distanceInches));

    // Find the closest distances in the lookup table
    Double lowerDistance = angleLookupTable.floorKey(distanceInches);
    Double higherDistance = angleLookupTable.ceilingKey(distanceInches);

    double targetAngle;

    // If exact match or only one boundary exists, use it directly
    if (lowerDistance == null || higherDistance == null || lowerDistance.equals(higherDistance)) {
      targetAngle = angleLookupTable.getOrDefault(distanceInches, basicAngle);
    } else {

      // Interpolate between the two angles for a more accurate angle
      double lowerAngle = angleLookupTable.get(lowerDistance);
      double higherAngle = angleLookupTable.get(higherDistance);
      targetAngle =
          interpolate(distanceInches, lowerDistance, higherDistance, lowerAngle, higherAngle);
    }

    // @TODO check targetAngle, raise elevator if it is too low
    if (targetAngle < ShooterPoseConstants.MIN_SHOOTER_TILT_DEGREES + 9) {
      shooterHeightPID.setGoal(ShooterPoseConstants.SHOOTER_HEIGHT_HANDOFF_SETPOINT + 1);
    } else {
      shooterHeightPID.setGoal(ShooterPoseConstants.SHOOTER_HEIGHT_HANDOFF_SETPOINT);
    }

    shooterTiltPID.setGoal(
        MathUtil.clamp(
            angleModulusDeg(targetAngle),
            ShooterPoseConstants.MIN_SHOOTER_TILT_DEGREES,
            ShooterPoseConstants.MAX_SHOOTER_TILT_DEGREES));
  }

  private static double interpolate(double x, double x0, double x1, double y0, double y1) {
    return y0 + (x - x0) * (y1 - y0) / (x1 - x0);
  }

  public void setShooterPose(Pose pose) {
    // shooterTiltPID.reset(shooterTiltEncoder.getPosition());
    // shooterHeightPID.reset(shooterHeightEncoder.getPosition());
    loggedPoseIO.pose = pose;

    switch (pose) {
      case HANDOFF:
        shooterHeightPID.setGoal(ShooterPoseConstants.SHOOTER_HEIGHT_HANDOFF_SETPOINT);
        shooterTiltPID.setGoal(angleModulusDeg(ShooterPoseConstants.SHOOTER_TILT_HANDOFF_SETPOINT));
        break;

      case SUBWOOFER:
        shooterHeightPID.setGoal(ShooterPoseConstants.SHOOTER_HEIGHT_SUBWOOFER_SETPOINT);
        shooterTiltPID.setGoal(
            angleModulusDeg(ShooterPoseConstants.SHOOTER_TILT_SUBWOOFER_SETPOINT));
        break;

      case AMP:
        shooterHeightPID.setGoal(ShooterPoseConstants.SHOOTER_HEIGHT_AMP_SETPOINT);
        shooterTiltPID.setGoal(angleModulusDeg(ShooterPoseConstants.SHOOTER_TILT_AMP_SETPOINT));
        break;

      case TRAP:
        shooterHeightPID.setGoal(ShooterPoseConstants.SHOOTER_HEIGHT_TRAP_SETPOINT);
        shooterTiltPID.setGoal(angleModulusDeg(ShooterPoseConstants.SHOOTER_TILT_TRAP_SETPOINT));
        break;

      case TRAP_PREP:
        shooterHeightPID.setGoal(ShooterPoseConstants.SHOOTER_HEIGHT_TRAP_PREP_SETPOINT);
        shooterTiltPID.setGoal(
            angleModulusDeg(ShooterPoseConstants.SHOOTER_TILT_TRAP_PREP_SETPOINT));
        break;

      case SOURCE:
        shooterHeightPID.setGoal(ShooterPoseConstants.SHOOTER_HEIGHT_SOURCE_SETPOINT);
        shooterTiltPID.setGoal(angleModulusDeg(ShooterPoseConstants.SHOOTER_TILT_SOURCE_SETPOINT));
        break;
      case CLIMBER:
        shooterHeightPID.setGoal(ShooterPoseConstants.SHOOTER_HEIGHT_SOURCE_SETPOINT);
        shooterTiltPID.setGoal(angleModulusDeg(ShooterPoseConstants.SHOOTER_TILT_SOURCE_SETPOINT));
        break;
      default:
        break;
    }
  }

  public boolean isAtGoal() {
    return shooterHeightPID.atGoal() && shooterTiltPID.atGoal();
  }

  private double getShooterDistance() {
    return shooterDistance;
  }

  private void setUpShuffleboard() {
    shooterPoseTab = Shuffleboard.getTab("ShooterPose");

    shooterPoseTab.addDouble("Shooter Distance", () -> getShooterDistance());
    if (ShooterPoseConstants.SHOOTER_POSE_TESTING) {
      shooterPoseTab.addDouble("Tilt Absolute Angle", () -> getAbsolutePosition());

      shooterPoseTab.addDouble("Tilt Angle", () -> shooterTiltEncoder.getPosition());
      shooterPoseTab.addDouble("Tilt Angle Velocity", () -> shooterTiltEncoder.getVelocity());

      // shooterPoseTab.addDouble("Elevator Height", () ->
      // shooterHeightEncoder.getPosition());
      // shooterPoseTab.addDouble("Elevator Velocity", () ->
      // shooterHeightEncoder.getVelocity());

      // shooterPoseTab.addBoolean("Lower Limit Switch", () ->
      // shooterHeightLimitSwitch.isPressed());

      shooterPoseTab.addDouble(
          "Feed Forward",
          () ->
              shooterTiltFeedforward.calculate(
                  Math.toRadians(
                      shooterTiltEncoder.getPosition()
                          + ShooterPoseConstants.SHOOTER_TILT_COG_OFFSET),
                  shooterTiltEncoder.getVelocity()));

      // shooterHeightP = shooterPoseTab.add("Shooter Height P", 0).getEntry();
      // shooterHeightI = shooterPoseTab.add("Shooter Height I", 0).getEntry();
      // shooterHeightD = shooterPoseTab.add("Shooter Height D", 0).getEntry();
      shooterTiltP =
          shooterPoseTab.add("Shooter Tilt P", ShooterPoseConstants.SHOOTER_TILT_KP).getEntry();
      shooterTiltI = shooterPoseTab.add("Shooter Tilt I", 0).getEntry();
      shooterTiltD = shooterPoseTab.add("Shooter Tilt D", 0).getEntry();

      goalEntry =
          shooterPoseTab
              .add("Goal", -32)
              .withWidget(BuiltInWidgets.kNumberSlider)
              .withProperties(
                  Map.of(
                      "min",
                      ShooterPoseConstants.MIN_SHOOTER_TILT_DEGREES,
                      "max",
                      ShooterPoseConstants.MAX_SHOOTER_TILT_DEGREES))
              .getEntry();
      maxVelocityEntry =
          shooterPoseTab
              .add("MaxVelocity", ShooterPoseConstants.SHOOTER_TILT_MAX_VELOCITY)
              .getEntry();
      maxAccelerationEntry =
          shooterPoseTab
              .add("MaxAcceleration", ShooterPoseConstants.SHOOTER_TILT_MAX_ACCELERATION)
              .getEntry();
    }
  }

  public void resetToAbsoluteEncoder() {
    if (shooterTiltAbsoluteEncoder.isConnected()) {
      shooterTiltEncoder.setPosition(getAbsolutePosition());
    }
  }

  public void periodic() {
    if (DriverStation.isDisabled()) {
      shooterHeightPID.reset(shooterHeightEncoder.getPosition());
      shooterTiltPID.reset(shooterTiltEncoder.getPosition());
    }

    // filter out false positives
    if (shooterHeightLimitSwitch.isPressed()) {
      limitSwitchCounter++;
    } else {
      limitSwitchCounter = 0;
    }

    // turn off elevator when limit switch is pressed, leave it off if goal isn't changed
    if (shooterHeightPID.getGoal().position != 0) {
      elevatorPIDOverride = false;
    } else if (limitSwitchCounter > 10) {
      elevatorPIDOverride = true;
    }

    if (elevatorPIDOverride) {
      shooterHeightRightMotor.stopMotor();
      shooterHeightEncoder.setPosition(0);
    } else {
      shooterHeightRightMotor.set(
          shooterHeightPID.calculate(shooterHeightEncoder.getPosition())
              + shooterHeightFeedforward.calculate(shooterHeightEncoder.getVelocity()));
    }

    shooterTiltMotor.set(
        MathUtil.clamp(shooterTiltPID.calculate(shooterTiltEncoder.getPosition()), -0.5, 0.5)
            + shooterTiltFeedforward.calculate(
                Math.toRadians(
                    shooterTiltEncoder.getPosition()
                        + ShooterPoseConstants.SHOOTER_TILT_COG_OFFSET),
                shooterTiltEncoder.getVelocity()));

    if (ShooterPoseConstants.SHOOTER_POSE_TESTING) {
      // shooterHeightPID.setP(shooterHeightP.getDouble(0));
      // shooterHeightPID.setI(shooterHeightI.getDouble(0));
      // shooterHeightPID.setD(shooterHeightD.getDouble(0));
      shooterTiltPID.setConstraints(
          new TrapezoidProfile.Constraints(
              maxVelocityEntry.getDouble(0), maxAccelerationEntry.getDouble(0)));
      shooterTiltPID.setP(shooterTiltP.getDouble(0));
      shooterTiltPID.setI(shooterTiltI.getDouble(0));
      shooterTiltPID.setD(shooterTiltD.getDouble(0));
      shooterTiltPID.setGoal(goalEntry.getDouble(-32));
    }
    if (ShooterPoseConstants.SHOOTER_POSE_LOGGING) {
      updateLoggedIO();
    }
  }

  public void updateLoggedIO() {
    loggedTiltIO.Angle = shooterTiltEncoder.getPosition();
    loggedTiltIO.AngleVelocity = shooterTiltEncoder.getVelocity();
    loggedTiltIO.AbsoluteAngle = getAbsolutePosition();
    loggedTiltIO.AngleGoal = shooterTiltPID.getGoal().position;
    loggedTiltIO.AngleSetpoint = shooterTiltPID.getSetpoint().position;
    loggedTiltIO.AngleSpeed = shooterTiltMotor.get();
    loggedTiltIO.AngleFeedforward =
        shooterTiltFeedforward.calculate(
            Math.toRadians(
                shooterTiltEncoder.getPosition() + ShooterPoseConstants.SHOOTER_TILT_COG_OFFSET),
            shooterTiltEncoder.getVelocity());
    loggedTiltIO.AbsoluteIsConnected = shooterTiltAbsoluteEncoder.isConnected();

    loggedHeightIO.Height = shooterHeightEncoder.getPosition();
    loggedHeightIO.HeightVelocity = shooterHeightEncoder.getVelocity();
    loggedHeightIO.HeightGoal = shooterHeightPID.getGoal().position;
    loggedHeightIO.HeightSetpoint = shooterHeightPID.getSetpoint().position;
    loggedHeightIO.HeightSpeed = shooterHeightRightMotor.get();
    loggedHeightIO.HeightFeedforward =
        shooterHeightFeedforward.calculate(shooterHeightEncoder.getVelocity());

    Logger.processInputs("Shooter Pose/Tilt", loggedTiltIO);
    Logger.processInputs("Shooter Pose/Height", loggedHeightIO);
    Logger.processInputs("Shooter Pose", loggedPoseIO);
  }

  private double getAbsolutePosition() {
    return angleModulusDeg(
        shooterTiltAbsoluteEncoder.getAbsolutePosition() * -360
            + ShooterPoseConstants.SHOOTER_TILT_ABSOLUTE_OFFSET);
  }

  private double angleModulusDeg(double angleDeg) {
    return Math.toDegrees(MathUtil.angleModulus(Math.toRadians(angleDeg)));
  }

  public boolean hasClearence() {
    if (shooterHeightEncoder.getPosition() <= 0.25
        && shooterTiltEncoder.getPosition()
            <= ShooterPoseConstants.SHOOTER_TILT_HANDOFF_SETPOINT + 2) {
      return true;
    } else {
      return false;
    }
  }
}
