package frc.robot.subsystems.shooterPose;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkLimitSwitch;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.AutoLog;

public class ShooterPose extends SubsystemBase {

  @AutoLog
  public static class ShooterPoseIOInput {
    double Angle = 0.0;
    double AbsoluteAngle = 0.0;
    double AngleGoal = 0.0;
    double AngleSpeed = 0.0;
    double AngleFeedforward = 0.0;
    double Height = 0.0;
    double HeightGoal = 0.0;
    double HeightSpeed = 0.0;
    double HeightFeedforward = 0.0;
  }

  private final ShooterPoseIOInput loggedIO = new ShooterPoseIOInputAutoLogged();

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

  private GenericEntry shooterHeightP, shooterHeightI, shooterHeightD;
  private GenericEntry shooterTiltP, shooterTiltI, shooterTiltD;

  // private final TunableNumber shooterHeightP =
  //     new TunableNumber("Elevator Height P", ShooterPoseConstants.SHOOTER_HEIGHT_KP);
  // private final TunableNumber shooterHeightI =
  //     new TunableNumber("Elevator Height I", ShooterPoseConstants.SHOOTER_HEIGHT_KI);
  // private final TunableNumber shooterHeightD =
  //     new TunableNumber("Elevator Height D", ShooterPoseConstants.SHOOTER_HEIGHT_KD);

  // private final TunableNumber shooterTiltP =
  //     new TunableNumber("Shooter Tilt P", ShooterPoseConstants.SHOOTER_TILT_KP);
  // private final TunableNumber shooterTiltP =
  //     new TunableNumber("Shooter Tilt I", ShooterPoseConstants.SHOOTER_TILT_KI);
  // private final TunableNumber shooterTiltD =
  //     new TunableNumber("Shooter Tilt D", ShooterPoseConstants.SHOOTER_TILT_KD);

  public ShooterPose() {
    shooterHeightLeftMotor =
        new CANSparkMax(
            ShooterPoseConstants.SHOOTER_HEIGHT_LEFT_MOTOR_CAN_ID, MotorType.kBrushless);
    shooterHeightRightMotor =
        new CANSparkMax(
            ShooterPoseConstants.SHOOTER_HEIGHT_RIGHT_MOTOR_CAN_ID, MotorType.kBrushless);

    shooterHeightRightMotor.restoreFactoryDefaults();
    shooterHeightLeftMotor.restoreFactoryDefaults();

    shooterHeightRightMotor.setInverted(true);

    shooterHeightRightMotor.enableVoltageCompensation(12);
    shooterHeightLeftMotor.enableVoltageCompensation(12);

    // shooterHeightRightMotor.setIdleMode(IdleMode.kBrake);
    // shooterHeightLeftMotor.setIdleMode(IdleMode.kBrake);

    // testing only
    shooterHeightRightMotor.setIdleMode(IdleMode.kCoast);
    shooterHeightLeftMotor.setIdleMode(IdleMode.kCoast);

    shooterHeightLeftMotor.follow(shooterHeightRightMotor, true);

    shooterHeightRightMotor.enableSoftLimit(SoftLimitDirection.kForward, true);
    shooterHeightRightMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);
    shooterHeightRightMotor.setSoftLimit(
        // SoftLimitDirection.kForward, (float) ShooterPoseConstants.MAX_SHOOTER_HEIGHT_INCHES);
        SoftLimitDirection.kForward, 4);
    shooterHeightRightMotor.setSoftLimit(
        // SoftLimitDirection.kReverse, (float) ShooterPoseConstants.MIN_SHOOTER_HEIGHT_INCHES);
        SoftLimitDirection.kReverse, -4);

    shooterHeightEncoder = shooterHeightRightMotor.getEncoder();
    shooterHeightEncoder.setPositionConversionFactor(
        ShooterPoseConstants.SHOOTER_HEIGHT_INCHES_PER_ROTATION);
    shooterHeightEncoder.setVelocityConversionFactor(
        ShooterPoseConstants.SHOOTER_HEIGHT_RPM_TO_INCHES_PER_SECOND);
    shooterHeightEncoder.setPosition(0);

    shooterTiltMotor =
        new CANSparkMax(ShooterPoseConstants.SHOOTER_TILT_MOTOR_CAN_ID, MotorType.kBrushless);

    shooterTiltMotor.restoreFactoryDefaults();
    shooterTiltMotor.setInverted(true);
    shooterTiltMotor.enableVoltageCompensation(12);
    // shooterTiltMotor.setIdleMode(IdleMode.kBrake);
    shooterTiltMotor.setIdleMode(IdleMode.kCoast);

    shooterTiltMotor.enableSoftLimit(SoftLimitDirection.kForward, true);
    shooterTiltMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);
    shooterTiltMotor.setSoftLimit(
        SoftLimitDirection.kForward, (float) ShooterPoseConstants.MAX_SHOOTER_TILT_DEGREES);
    shooterTiltMotor.setSoftLimit(
        SoftLimitDirection.kReverse, (float) ShooterPoseConstants.MIN_SHOOTER_TILT_DEGREES);

    shooterTiltAbsoluteEncoder = new DutyCycleEncoder(9);
    shooterTiltAbsoluteEncoder.setDistancePerRotation(-360);
    shooterTiltAbsoluteEncoder.setPositionOffset(ShooterPoseConstants.SHOOTER_TILT_ABSOLUTE_OFFSET/360);

    shooterTiltEncoder = shooterTiltMotor.getEncoder();
    shooterTiltEncoder.setPositionConversionFactor(
        ShooterPoseConstants.SHOOTER_TILT_DEGREES_PER_ROTATION);
    shooterTiltEncoder.setVelocityConversionFactor(
        ShooterPoseConstants.SHOOTER_TILT_RPM_TO_DEGREES_PER_SECOND);
    shooterTiltEncoder.setPosition(shooterTiltAbsoluteEncoder.getDistance());

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

    shooterTiltFeedforward =
        new ArmFeedforward(
            ShooterPoseConstants.SHOOTER_TILT_KS,
            ShooterPoseConstants.SHOOTER_TILT_KG,
            ShooterPoseConstants.SHOOTER_TILT_KV,
            ShooterPoseConstants.SHOOTER_TILT_KA);

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

    shooterHeightFeedforward =
        new ElevatorFeedforward(
            ShooterPoseConstants.SHOOTER_HEIGHT_KS,
            ShooterPoseConstants.SHOOTER_HEIGHT_KG,
            ShooterPoseConstants.SHOOTER_HEIGHT_KV,
            ShooterPoseConstants.SHOOTER_HEIGHT_KA);

    if (ShooterPoseConstants.SHOOTER_POSE_TESTING) {
      setUpShuffleboard();
    }

    shooterHeightRightMotor.stopMotor();
    shooterTiltMotor.stopMotor();
  }

  public void setShooterDistance(double distanceInches) {
    if (distanceInches == 0) {
      return;
    }
    // @TODO Determine if we need to reset PID controllers here
    shooterHeightPID.setGoal(ShooterPoseConstants.SHOOTER_POSE_HANDOFF_SETPOINT);
    double speakerHeight = 83, shooterHeight = 27;
    double targetAngle =
        -1 * Math.toDegrees(Math.atan((speakerHeight - shooterHeight) / distanceInches));
    shooterTiltPID.setGoal(targetAngle);
  }

  public void setShooterPose(Pose pose) {
    shooterTiltPID.reset(shooterTiltEncoder.getPosition());
    shooterHeightPID.reset(shooterHeightEncoder.getPosition());

    switch (pose) {
      case HANDOFF:
        shooterHeightPID.setGoal(ShooterPoseConstants.SHOOTER_POSE_HANDOFF_SETPOINT);
        shooterTiltPID.setGoal(ShooterPoseConstants.SHOOTER_TILT_HANDOFF_SETPOINT);
        break;

      case SUBWOOFER:
        shooterHeightPID.setGoal(ShooterPoseConstants.SHOOTER_POSE_SUBWOOFER_SETPOINT);
        shooterTiltPID.setGoal(ShooterPoseConstants.SHOOTER_TILT_SUBWOOFER_SETPOINT);
        break;

      case AMP:
        shooterHeightPID.setGoal(ShooterPoseConstants.SHOOTER_POSE_AMP_SETPOINT);
        shooterTiltPID.setGoal(ShooterPoseConstants.SHOOTER_TILT_AMP_SETPOINT);
        break;

      case TRAP:
        shooterHeightPID.setGoal(ShooterPoseConstants.SHOOTER_POSE_TRAP_SETPOINT);
        shooterTiltPID.setGoal(ShooterPoseConstants.SHOOTER_TILT_TRAP_SETPOINT);
        break;

      case SOURCE:
        shooterHeightPID.setGoal(ShooterPoseConstants.SHOOTER_POSE_SOURCE_SETPOINT);
        shooterTiltPID.setGoal(ShooterPoseConstants.SHOOTER_TILT_SOURCE_SETPOINT);
        break;
    }
  }

  private void setUpShuffleboard() {
    shooterPoseTab = Shuffleboard.getTab("Elevator");

    shooterPoseTab.addDouble("Tilt Absolute Angle", () -> shooterTiltAbsoluteEncoder.getDistance());

    shooterPoseTab.addDouble("Tilt Angle", () -> shooterTiltEncoder.getPosition());
    shooterPoseTab.addDouble("Tilt Angle Velocity", () -> shooterTiltEncoder.getVelocity());

    shooterPoseTab.addDouble("Elevator Height", () -> shooterHeightEncoder.getPosition());
    shooterPoseTab.addDouble("Elevator Velocity", () -> shooterHeightEncoder.getVelocity());

    shooterHeightP = shooterPoseTab.add("Shooter Height P", 0).getEntry();
    shooterHeightI = shooterPoseTab.add("Shooter Height I", 0).getEntry();
    shooterHeightD = shooterPoseTab.add("Shooter Height D", 0).getEntry();
    shooterTiltP = shooterPoseTab.add("Shooter Tilt P", 0).getEntry();
    shooterTiltI = shooterPoseTab.add("Shooter Tilt I", 0).getEntry();
    shooterTiltD = shooterPoseTab.add("Shooter Tilt D", 0).getEntry();

    goalEntry = shooterPoseTab.add("Goal", 0).getEntry();
  }

  public void periodic() {
    // if (shooterHeightLimitSwitch.isPressed()) {
    //   shooterHeightEncoder.setPosition(0);
    // } else if (shooterHeightPID.atGoal() && shooterHeightPID.getGoal().position == 0) {
    //   // move elevator manually
    // }

    // shooterTiltMotor.set(
    //     shooterTiltPID.calculate(shooterTiltEncoder.getPosition())
    //         + shooterTiltFeedforward.calculate(
    //             shooterTiltEncoder.getPosition(), shooterTiltEncoder.getVelocity()));

    shooterHeightRightMotor.set(
        shooterHeightPID.calculate(shooterHeightEncoder.getPosition(), goalEntry.getDouble(0))
            + shooterHeightFeedforward.calculate(
                shooterHeightEncoder.getPosition(), shooterHeightEncoder.getVelocity()));

    if (ShooterPoseConstants.SHOOTER_POSE_TESTING) {
      shooterHeightPID.setP(shooterHeightP.getDouble(0));
      shooterHeightPID.setI(shooterHeightI.getDouble(0));
      shooterHeightPID.setD(shooterHeightD.getDouble(0));
      shooterTiltPID.setP(shooterTiltP.getDouble(0));
      shooterTiltPID.setI(shooterTiltI.getDouble(0));
      shooterTiltPID.setD(shooterTiltD.getDouble(0));
    }

    updateLoggedIO();
  }

  public void updateLoggedIO() {
    loggedIO.Angle = shooterTiltEncoder.getPosition();
    loggedIO.AbsoluteAngle = shooterTiltAbsoluteEncoder.getDistance();
    loggedIO.AngleGoal = shooterTiltPID.getGoal().position;
    loggedIO.AngleSpeed = shooterTiltMotor.get();
    loggedIO.AngleFeedforward =
        shooterTiltFeedforward.calculate(
            shooterTiltEncoder.getPosition(), shooterTiltEncoder.getVelocity());
    loggedIO.Height = shooterHeightEncoder.getPosition();
    loggedIO.HeightGoal = shooterHeightPID.getGoal().position;
    loggedIO.HeightSpeed = shooterHeightRightMotor.get();
    loggedIO.HeightFeedforward =
        shooterHeightFeedforward.calculate(
            shooterHeightEncoder.getPosition(), shooterHeightEncoder.getVelocity());
  }
}
