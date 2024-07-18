package frc.lib.team3061.drivetrain;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.ClosedLoopOutputType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants.SteerFeedbackType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstantsFactory;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import frc.lib.team3061.RobotConfig;
import frc.lib.team3061.drivetrain.swerve.Conversions;
import frc.lib.team3061.drivetrain.swerve.SwerveConstants;
import frc.lib.team3061.gyro.GyroIO.GyroIOInputs;
import frc.lib.team3061.util.RobotOdometry;
import frc.lib.team6328.util.TunableNumber;
import frc.robot.Constants;

public class DrivetrainIOCTRE extends SwerveDrivetrain implements DrivetrainIO {

  static class CustomSlotGains extends Slot0Configs {
    public CustomSlotGains(double kP, double kI, double kD, double kA, double kV, double kS) {
      this.kP = kP;
      this.kI = kI;
      this.kD = kD;
      this.kA = kA;
      this.kV = kV;
      this.kS = kS;
    }
  }

  static class SwerveModuleSignals {
    public SwerveModuleSignals(TalonFX driveMotor, TalonFX steerMotor) {
      this.steerVelocityStatusSignal = steerMotor.getVelocity().clone();
      this.steerAccelerationStatusSignal = steerMotor.getAcceleration().clone();
      this.steerPositionErrorStatusSignal = steerMotor.getClosedLoopError().clone();
      this.steerPositionReferenceStatusSignal = steerMotor.getClosedLoopReference().clone();
      this.drivePositionStatusSignal = driveMotor.getPosition().clone();
      this.driveVelocityErrorStatusSignal = driveMotor.getClosedLoopError().clone();
      this.driveVelocityReferenceStatusSignal = driveMotor.getClosedLoopReference().clone();
      this.driveAccelerationStatusSignal = driveMotor.getAcceleration().clone();
    }

    StatusSignal<Double> steerVelocityStatusSignal;
    StatusSignal<Double> steerAccelerationStatusSignal;
    StatusSignal<Double> steerPositionErrorStatusSignal;
    StatusSignal<Double> steerPositionReferenceStatusSignal;
    StatusSignal<Double> drivePositionStatusSignal;
    StatusSignal<Double> driveVelocityErrorStatusSignal;
    StatusSignal<Double> driveVelocityReferenceStatusSignal;
    StatusSignal<Double> driveAccelerationStatusSignal;
  }

  private final TunableNumber driveKp =
      new TunableNumber("Drive/DriveKp", RobotConfig.getInstance().getSwerveDriveKP());
  private final TunableNumber driveKi =
      new TunableNumber("Drive/DriveKi", RobotConfig.getInstance().getSwerveDriveKI());
  private final TunableNumber driveKd =
      new TunableNumber("Drive/DriveKd", RobotConfig.getInstance().getSwerveDriveKD());
  private final TunableNumber steerKp =
      new TunableNumber("Drive/TurnKp", RobotConfig.getInstance().getSwerveAngleKP());
  private final TunableNumber steerKi =
      new TunableNumber("Drive/TurnKi", RobotConfig.getInstance().getSwerveAngleKI());
  private final TunableNumber steerKd =
      new TunableNumber("Drive/TurnKd", RobotConfig.getInstance().getSwerveAngleKD());

  private static final double COUPLE_RATIO = 0.0;
  private static final double STEER_INERTIA = 0.00001;
  private static final double DRIVE_INERTIA = 0.001;
  // The steer motor uses any SwerveModule.SteerRequestType control request with the
  // output type specified by SwerveModuleConstants.SteerMotorClosedLoopOutput
  private static final Slot0Configs steerGains =
      new Slot0Configs().withKP(100).withKI(0).withKD(0.2).withKS(0).withKV(1.5).withKA(0);
  // When using closed-loop control, the drive motor uses the control
  // output type specified by SwerveModuleConstants.DriveMotorClosedLoopOutput
  private static final Slot0Configs driveGains =
      new Slot0Configs().withKP(3).withKI(0).withKD(0).withKS(0).withKV(0).withKA(0);

  // The closed-loop output type to use for the steer motors;
  // This affects the PID/FF gains for the steer motors
  private static final ClosedLoopOutputType steerClosedLoopOutput = ClosedLoopOutputType.Voltage;
  // The closed-loop output type to use for the drive motors;
  // This affects the PID/FF gains for the drive motors
  private static final ClosedLoopOutputType driveClosedLoopOutput = ClosedLoopOutputType.Voltage;

  // The stator current at which the wheels start to slip;
  // This needs to be tuned to your individual robot
  private static final double kSlipCurrentA = 150.0;

  // Initial configs for the drive and steer motors and the CANcoder; these cannot be null.
  // Some configs will be overwritten; check the `with*InitialConfigs()` API documentation.
  private static final TalonFXConfiguration driveInitialConfigs = new TalonFXConfiguration();
  private static final TalonFXConfiguration steerInitialConfigs =
      new TalonFXConfiguration()
          .withCurrentLimits(
              new CurrentLimitsConfigs()
                  // Swerve azimuth does not require much torque output, so we can set a relatively
                  // low
                  // stator current limit to help avoid brownouts without impacting performance.
                  .withStatorCurrentLimit(60)
                  .withStatorCurrentLimitEnable(true));
  private static final CANcoderConfiguration cancoderInitialConfigs = new CANcoderConfiguration();
  // Configs for the Pigeon 2; leave this null to skip applying Pigeon 2 configs
  private static final Pigeon2Configuration pigeonConfigs = null;

  // Theoretical free speed (m/s) at 12v applied output;
  // This needs to be tuned to your individual robot
  public static final double kSpeedAt12VoltsMps = 5.96;

  // Every 1 rotation of the azimuth results in kCoupleRatio drive motor turns;
  // This may need to be tuned to your individual robot
  private static final double kCoupleRatio = 3.125;

  private static final double kDriveGearRatio = 5.357142857142857;
  private static final double kSteerGearRatio = 21.428571428571427;
  private static final double kWheelRadiusInches = 2;

  private static final boolean kInvertLeftSide = false;
  private static final boolean kInvertRightSide = true;

  private static final String kCANbusName = "Monke";
  private static final int kPigeonId = 7;

  // These are only used for simulation
  private static final double kSteerInertia = 0.00001;
  private static final double kDriveInertia = 0.001;
  // Simulated voltage necessary to overcome friction
  private static final double kSteerFrictionVoltage = 0.25;
  private static final double kDriveFrictionVoltage = 0.25;

  private static final SwerveDrivetrainConstants drivetrainConstants =
      new SwerveDrivetrainConstants()
          .withCANbusName(kCANbusName)
          .withPigeon2Id(kPigeonId)
          .withPigeon2Configs(pigeonConfigs);

  private static final SwerveModuleConstantsFactory ConstantCreator =
      new SwerveModuleConstantsFactory()
          .withDriveMotorGearRatio(kDriveGearRatio)
          .withSteerMotorGearRatio(kSteerGearRatio)
          .withWheelRadius(kWheelRadiusInches)
          .withSlipCurrent(kSlipCurrentA)
          .withSteerMotorGains(steerGains)
          .withDriveMotorGains(driveGains)
          .withSteerMotorClosedLoopOutput(steerClosedLoopOutput)
          .withDriveMotorClosedLoopOutput(driveClosedLoopOutput)
          .withSpeedAt12VoltsMps(kSpeedAt12VoltsMps)
          .withSteerInertia(kSteerInertia)
          .withDriveInertia(kDriveInertia)
          .withSteerFrictionVoltage(kSteerFrictionVoltage)
          .withDriveFrictionVoltage(kDriveFrictionVoltage)
          .withFeedbackSource(SteerFeedbackType.FusedCANcoder)
          .withCouplingGearRatio(kCoupleRatio)
          .withDriveMotorInitialConfigs(driveInitialConfigs)
          .withSteerMotorInitialConfigs(steerInitialConfigs)
          .withCANcoderInitialConfigs(cancoderInitialConfigs);

  // Front Left
  private static final int kFrontLeftDriveMotorId = 20;
  private static final int kFrontLeftSteerMotorId = 21;
  private static final int kFrontLeftEncoderId = 22;
  private static final double kFrontLeftEncoderOffset = 0.21435546875;
  private static final boolean kFrontLeftSteerInvert = false;

  private static final double kFrontLeftXPosInches = 11.25;
  private static final double kFrontLeftYPosInches = 11.25;

  // Front Right
  private static final int kFrontRightDriveMotorId = 30;
  private static final int kFrontRightSteerMotorId = 31;
  private static final int kFrontRightEncoderId = 32;
  private static final double kFrontRightEncoderOffset = -0.435546875;
  private static final boolean kFrontRightSteerInvert = false;

  private static final double kFrontRightXPosInches = 11.25;
  private static final double kFrontRightYPosInches = -11.25;

  // Back Left
  private static final int kBackLeftDriveMotorId = 10;
  private static final int kBackLeftSteerMotorId = 11;
  private static final int kBackLeftEncoderId = 12;
  private static final double kBackLeftEncoderOffset = 0.205322265625;
  private static final boolean kBackLeftSteerInvert = false;

  private static final double kBackLeftXPosInches = -11.25;
  private static final double kBackLeftYPosInches = 11.25;

  // Back Right
  private static final int kBackRightDriveMotorId = 40;
  private static final int kBackRightSteerMotorId = 41;
  private static final int kBackRightEncoderId = 42;
  private static final double kBackRightEncoderOffset = 0.025634765625;
  private static final boolean kBackRightSteerInvert = false;

  private static final double kBackRightXPosInches = -11.25;
  private static final double kBackRightYPosInches = -11.25;

  private static final SwerveModuleConstants frontLeft =
      ConstantCreator.createModuleConstants(
              kFrontLeftSteerMotorId,
              kFrontLeftDriveMotorId,
              kFrontLeftEncoderId,
              kFrontLeftEncoderOffset,
              Units.inchesToMeters(kFrontLeftXPosInches),
              Units.inchesToMeters(kFrontLeftYPosInches),
              kInvertLeftSide)
          .withSteerMotorInverted(kFrontLeftSteerInvert);
  private static final SwerveModuleConstants frontRight =
      ConstantCreator.createModuleConstants(
              kFrontRightSteerMotorId,
              kFrontRightDriveMotorId,
              kFrontRightEncoderId,
              kFrontRightEncoderOffset,
              Units.inchesToMeters(kFrontRightXPosInches),
              Units.inchesToMeters(kFrontRightYPosInches),
              kInvertRightSide)
          .withSteerMotorInverted(kFrontRightSteerInvert);
  private static final SwerveModuleConstants backLeft =
      ConstantCreator.createModuleConstants(
              kBackLeftSteerMotorId,
              kBackLeftDriveMotorId,
              kBackLeftEncoderId,
              kBackLeftEncoderOffset,
              Units.inchesToMeters(kBackLeftXPosInches),
              Units.inchesToMeters(kBackLeftYPosInches),
              kInvertLeftSide)
          .withSteerMotorInverted(kBackLeftSteerInvert);
  private static final SwerveModuleConstants backRight =
      ConstantCreator.createModuleConstants(
              kBackRightSteerMotorId,
              kBackRightDriveMotorId,
              kBackRightEncoderId,
              kBackRightEncoderOffset,
              Units.inchesToMeters(kBackRightXPosInches),
              Units.inchesToMeters(kBackRightYPosInches),
              kInvertRightSide)
          .withSteerMotorInverted(kBackRightSteerInvert);

  // gyro signals
  private final StatusSignal<Double> pitchStatusSignal;
  private final StatusSignal<Double> rollStatusSignal;
  private final StatusSignal<Double> angularVelocityXStatusSignal;
  private final StatusSignal<Double> angularVelocityYStatusSignal;

  // swerve module signals
  SwerveModuleSignals[] swerveModulesSignals = new SwerveModuleSignals[4];

  private Translation2d centerOfRotation;
  private ChassisSpeeds targetChassisSpeeds;

  private SwerveRequest.Idle idleRequest = new SwerveRequest.Idle();
  private SwerveRequest.RobotCentric driveRobotCentricRequest = new SwerveRequest.RobotCentric();
  private SwerveRequest.FieldCentric driveFieldCentricRequest = new SwerveRequest.FieldCentric();
  private SwerveRequest.FieldCentricFacingAngle driveFacingAngleRequest =
      new SwerveRequest.FieldCentricFacingAngle();
  private SwerveRequest.SwerveDriveBrake brakeRequest = new SwerveRequest.SwerveDriveBrake();
  private SwerveRequest.PointWheelsAt pointRequest = new SwerveRequest.PointWheelsAt();
  private SwerveRequest.ApplyChassisSpeeds applyChassisSpeedsRequest =
      new SwerveRequest.ApplyChassisSpeeds();

  // only used for TorqueCurrentFOC characterization
  private TorqueCurrentFOC[] driveCurrentRequests = new TorqueCurrentFOC[4];
  private TorqueCurrentFOC[] steerCurrentRequests = new TorqueCurrentFOC[4];

  /**
   * Creates a new Drivetrain subsystem.
   *
   * @param gyroIO the abstracted interface for the gyro for the drivetrain
   * @param flModule the front left swerve module
   * @param frModule the front right swerve module
   * @param blModule the back left swerve module
   * @param brModule the back right swerve module
   */
  public DrivetrainIOCTRE() {
    super(
        drivetrainConstants,
        RobotConfig.getInstance().getOdometryUpdateFrequency(),
        frontLeft,
        frontRight,
        backLeft,
        backRight);

    // configure current limits
    for (SwerveModule swerveModule : this.Modules) {
      CurrentLimitsConfigs currentLimits = new CurrentLimitsConfigs();
      swerveModule.getDriveMotor().getConfigurator().refresh(currentLimits);
      currentLimits.SupplyCurrentLimit = SwerveConstants.DRIVE_CONTINUOUS_CURRENT_LIMIT;
      currentLimits.SupplyCurrentThreshold = SwerveConstants.DRIVE_PEAK_CURRENT_LIMIT;
      currentLimits.SupplyTimeThreshold = SwerveConstants.DRIVE_PEAK_CURRENT_DURATION;
      currentLimits.SupplyCurrentLimitEnable = SwerveConstants.DRIVE_ENABLE_CURRENT_LIMIT;
      swerveModule.getDriveMotor().getConfigurator().apply(currentLimits);

      currentLimits = new CurrentLimitsConfigs();
      swerveModule.getSteerMotor().getConfigurator().refresh(currentLimits);
      currentLimits.SupplyCurrentLimit = SwerveConstants.ANGLE_CONTINUOUS_CURRENT_LIMIT;
      currentLimits.SupplyCurrentThreshold = SwerveConstants.ANGLE_PEAK_CURRENT_LIMIT;
      currentLimits.SupplyTimeThreshold = SwerveConstants.ANGLE_PEAK_CURRENT_DURATION;
      currentLimits.SupplyCurrentLimitEnable = SwerveConstants.ANGLE_ENABLE_CURRENT_LIMIT;
      swerveModule.getSteerMotor().getConfigurator().apply(currentLimits);
    }

    this.pitchStatusSignal = this.m_pigeon2.getPitch().clone();
    this.pitchStatusSignal.setUpdateFrequency(100);
    this.rollStatusSignal = this.m_pigeon2.getRoll().clone();
    this.rollStatusSignal.setUpdateFrequency(100);
    this.angularVelocityXStatusSignal = this.m_pigeon2.getAngularVelocityXWorld().clone();
    this.angularVelocityXStatusSignal.setUpdateFrequency(100);
    this.angularVelocityYStatusSignal = this.m_pigeon2.getAngularVelocityYWorld().clone();
    this.angularVelocityYStatusSignal.setUpdateFrequency(100);

    for (int i = 0; i < swerveModulesSignals.length; i++) {
      swerveModulesSignals[i] =
          new SwerveModuleSignals(this.Modules[i].getSteerMotor(), this.Modules[i].getDriveMotor());
    }

    this.centerOfRotation = new Translation2d(); // default to (0,0)

    this.targetChassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);

    // specify that we will be using CTRE's custom odometry instead of 3061 lib's default
    RobotOdometry.getInstance().setCustomOdometry(this);

    for (int i = 0; i < driveCurrentRequests.length; i++) {
      this.driveCurrentRequests[i] = new TorqueCurrentFOC(0.0);
      this.steerCurrentRequests[i] = new TorqueCurrentFOC(0.0);
    }
  }

  @Override
  public void updateInputs(DrivetrainIOInputsCollection inputs) {

    // update and log gyro inputs
    this.updateGyroInputs(inputs.gyro);

    // update and log the swerve modules inputs
    for (int i = 0; i < swerveModulesSignals.length; i++) {
      this.updateSwerveModuleInputs(inputs.swerve[i], this.Modules[i], swerveModulesSignals[i]);
    }

    inputs.drivetrain.swerveMeasuredStates = this.getState().ModuleStates;
    inputs.drivetrain.swerveReferenceStates = this.getState().ModuleTargets;

    // log poses, 3D geometry, and swerve module states, gyro offset
    inputs.drivetrain.robotPose =
        new Pose2d(this.getState().Pose.getTranslation(), this.getState().Pose.getRotation());
    inputs.drivetrain.robotPose3D = new Pose3d(inputs.drivetrain.robotPose);

    inputs.drivetrain.targetVXMetersPerSec = this.targetChassisSpeeds.vxMetersPerSecond;
    inputs.drivetrain.targetVYMetersPerSec = this.targetChassisSpeeds.vyMetersPerSecond;
    inputs.drivetrain.targetAngularVelocityRadPerSec =
        this.targetChassisSpeeds.omegaRadiansPerSecond;

    ChassisSpeeds measuredChassisSpeeds =
        m_kinematics.toChassisSpeeds(this.getState().ModuleStates);
    inputs.drivetrain.measuredVXMetersPerSec = measuredChassisSpeeds.vxMetersPerSecond;
    inputs.drivetrain.measuredVYMetersPerSec = measuredChassisSpeeds.vyMetersPerSecond;
    inputs.drivetrain.measuredAngularVelocityRadPerSec =
        measuredChassisSpeeds.omegaRadiansPerSecond;

    inputs.drivetrain.averageDriveCurrent = this.getAverageDriveCurrent(inputs);

    inputs.drivetrain.rotation = this.getState().Pose.getRotation();

    // update tunables
    if (driveKp.hasChanged()
        || driveKi.hasChanged()
        || driveKd.hasChanged()
        || steerKp.hasChanged()
        || steerKi.hasChanged()
        || steerKd.hasChanged()) {
      for (SwerveModule swerveModule : this.Modules) {
        Slot0Configs driveSlot0 = new Slot0Configs();
        swerveModule.getDriveMotor().getConfigurator().refresh(driveSlot0);
        driveSlot0.kP = driveKp.get();
        driveSlot0.kI = driveKi.get();
        driveSlot0.kD = driveKd.get();
        swerveModule.getDriveMotor().getConfigurator().apply(driveSlot0);

        Slot0Configs steerSlot0 = new Slot0Configs();
        swerveModule.getSteerMotor().getConfigurator().refresh(steerSlot0);
        steerSlot0.kP = steerKp.get();
        steerSlot0.kI = steerKi.get();
        steerSlot0.kD = steerKd.get();
        swerveModule.getSteerMotor().getConfigurator().apply(steerSlot0);
      }
    }
  }

  private void updateGyroInputs(GyroIOInputs inputs) {
    BaseStatusSignal.refreshAll(
        this.pitchStatusSignal,
        this.rollStatusSignal,
        this.angularVelocityXStatusSignal,
        this.angularVelocityYStatusSignal);

    inputs.connected = (this.m_yawGetter.getStatus() == StatusCode.OK);
    inputs.yawDeg =
        BaseStatusSignal.getLatencyCompensatedValue(this.m_yawGetter, this.m_angularVelocity);
    inputs.pitchDeg =
        BaseStatusSignal.getLatencyCompensatedValue(
            this.pitchStatusSignal, this.angularVelocityYStatusSignal);
    inputs.rollDeg =
        BaseStatusSignal.getLatencyCompensatedValue(
            this.rollStatusSignal, this.angularVelocityXStatusSignal);
    inputs.rollDegPerSec = this.angularVelocityXStatusSignal.getValue();
    inputs.pitchDegPerSec = this.angularVelocityYStatusSignal.getValue();
    inputs.yawDegPerSec = this.m_angularVelocity.getValue();
  }

  private void updateSwerveModuleInputs(
      SwerveIOInputs inputs, SwerveModule module, SwerveModuleSignals signals) {

    BaseStatusSignal.refreshAll(
        signals.steerVelocityStatusSignal,
        signals.steerAccelerationStatusSignal,
        signals.steerPositionErrorStatusSignal,
        signals.steerPositionReferenceStatusSignal,
        signals.drivePositionStatusSignal,
        signals.driveVelocityErrorStatusSignal,
        signals.driveVelocityReferenceStatusSignal,
        signals.driveAccelerationStatusSignal);

    SwerveModulePosition position = module.getPosition(false);
    SwerveModuleState state = module.getCurrentState();

    inputs.driveDistanceMeters = position.distanceMeters;
    inputs.driveVelocityMetersPerSec = state.speedMetersPerSecond;
    inputs.driveVelocityReferenceMetersPerSec =
        Conversions.falconRPSToMechanismMPS(
            signals.driveVelocityReferenceStatusSignal.getValue(),
            SwerveConstants.MK4I_L2_WHEEL_CIRCUMFERENCE,
            SwerveConstants.MK4I_L2_DRIVE_GEAR_RATIO);
    inputs.driveVelocityErrorMetersPerSec =
        Conversions.falconRPSToMechanismMPS(
            signals.driveVelocityErrorStatusSignal.getValue(),
            SwerveConstants.MK4I_L2_WHEEL_CIRCUMFERENCE,
            SwerveConstants.MK4I_L2_DRIVE_GEAR_RATIO);
    inputs.driveAccelerationMetersPerSecPerSec =
        Conversions.falconRPSToMechanismMPS(
            signals.driveAccelerationStatusSignal.getValue(),
            SwerveConstants.MK4I_L2_WHEEL_CIRCUMFERENCE,
            SwerveConstants.MK4I_L2_DRIVE_GEAR_RATIO);
    inputs.driveAppliedVolts = module.getDriveMotor().getMotorVoltage().getValue();
    inputs.driveStatorCurrentAmps = module.getDriveMotor().getStatorCurrent().getValue();
    inputs.driveSupplyCurrentAmps = module.getDriveMotor().getSupplyCurrent().getValue();
    inputs.driveTempCelsius = module.getDriveMotor().getDeviceTemp().getValue();

    inputs.steerAbsolutePositionDeg = module.getCANcoder().getAbsolutePosition().getValue() * 360.0;

    // since we are using the FusedCANcoder feature, the position and velocity signal for the angle
    // motor accounts for the gear ratio; so, pass a gear ratio of 1 to just convert from rotations
    // to degrees.
    inputs.steerPositionDeg = position.angle.getDegrees();
    inputs.steerPositionReferenceDeg =
        Conversions.falconRotationsToMechanismDegrees(
            signals.steerPositionReferenceStatusSignal.getValue(), 1);
    inputs.steerPositionErrorDeg =
        Conversions.falconRotationsToMechanismDegrees(
            signals.steerPositionErrorStatusSignal.getValue(), 1);
    inputs.steerVelocityRevPerMin =
        Conversions.falconRPSToMechanismRPM(signals.steerVelocityStatusSignal.getValue(), 1);
    inputs.steerAccelerationMetersPerSecPerSec =
        Conversions.falconRPSToMechanismRPM(signals.steerAccelerationStatusSignal.getValue(), 1);

    inputs.steerAppliedVolts = module.getSteerMotor().getMotorVoltage().getValue();
    inputs.steerStatorCurrentAmps = module.getSteerMotor().getStatorCurrent().getValue();
    inputs.steerSupplyCurrentAmps = module.getSteerMotor().getSupplyCurrent().getValue();
    inputs.steerTempCelsius = module.getSteerMotor().getDeviceTemp().getValue();
  }

  @Override
  public void holdXStance() {
    this.targetChassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);
    this.setControl(this.brakeRequest);
  }

  @Override
  public void driveFieldRelative(
      double xVelocity, double yVelocity, double rotationalVelocity, boolean isOpenLoop) {

    this.targetChassisSpeeds =
        ChassisSpeeds.discretize(
            ChassisSpeeds.fromFieldRelativeSpeeds(
                xVelocity, yVelocity, rotationalVelocity, this.getState().Pose.getRotation()),
            Constants.LOOP_PERIOD_SECS);

    if (isOpenLoop) {
      this.setControl(
          this.driveFieldCentricRequest
              .withDriveRequestType(SwerveModule.DriveRequestType.OpenLoopVoltage)
              .withSteerRequestType(SwerveModule.SteerRequestType.MotionMagicExpo)
              .withVelocityX(xVelocity)
              .withVelocityY(yVelocity)
              .withRotationalRate(rotationalVelocity));
    } else {
      this.setControl(
          this.driveFieldCentricRequest
              .withDriveRequestType(SwerveModule.DriveRequestType.Velocity)
              .withSteerRequestType(SwerveModule.SteerRequestType.MotionMagicExpo)
              .withVelocityX(xVelocity)
              .withVelocityY(yVelocity)
              .withRotationalRate(rotationalVelocity));
    }
  }

  @Override
  public void driveFieldRelativeFacingAngle(
      double xVelocity, double yVelocity, Rotation2d targetDirection, boolean isOpenLoop) {
    this.targetChassisSpeeds =
        ChassisSpeeds.discretize(
            ChassisSpeeds.fromFieldRelativeSpeeds(
                xVelocity, yVelocity, 0.0, getState().Pose.getRotation()),
            Constants.LOOP_PERIOD_SECS);

    if (isOpenLoop) {
      this.setControl(
          this.driveFacingAngleRequest
              .withDriveRequestType(SwerveModule.DriveRequestType.OpenLoopVoltage)
              .withSteerRequestType(SwerveModule.SteerRequestType.MotionMagicExpo)
              .withVelocityX(xVelocity)
              .withVelocityY(yVelocity)
              .withTargetDirection(targetDirection));
    } else {
      this.setControl(
          this.driveFacingAngleRequest
              .withDriveRequestType(SwerveModule.DriveRequestType.Velocity)
              .withSteerRequestType(SwerveModule.SteerRequestType.MotionMagicExpo)
              .withVelocityX(xVelocity)
              .withVelocityY(yVelocity)
              .withTargetDirection(targetDirection));
    }
  }

  @Override
  public void pointWheelsAt(Rotation2d targetDirection) {
    this.targetChassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);

    this.setControl(this.pointRequest.withModuleDirection(targetDirection));
  }

  @Override
  public void driveRobotRelative(
      double xVelocity, double yVelocity, double rotationalVelocity, boolean isOpenLoop) {
    this.targetChassisSpeeds =
        ChassisSpeeds.discretize(
            new ChassisSpeeds(xVelocity, yVelocity, rotationalVelocity),
            Constants.LOOP_PERIOD_SECS);

    if (isOpenLoop) {
      this.setControl(
          this.driveRobotCentricRequest
              .withDriveRequestType(SwerveModule.DriveRequestType.OpenLoopVoltage)
              .withSteerRequestType(SwerveModule.SteerRequestType.MotionMagicExpo)
              .withVelocityX(xVelocity)
              .withVelocityY(yVelocity)
              .withRotationalRate(rotationalVelocity));
    } else {
      this.setControl(
          this.driveRobotCentricRequest
              .withDriveRequestType(SwerveModule.DriveRequestType.Velocity)
              .withSteerRequestType(SwerveModule.SteerRequestType.MotionMagicExpo)
              .withVelocityX(xVelocity)
              .withVelocityY(yVelocity)
              .withRotationalRate(rotationalVelocity));
    }
  }

  @Override
  public void setChassisSpeeds(ChassisSpeeds speeds, boolean isOpenLoop) {
    this.targetChassisSpeeds.omegaRadiansPerSecond = speeds.omegaRadiansPerSecond;
    this.targetChassisSpeeds.vxMetersPerSecond = speeds.vxMetersPerSecond;
    this.targetChassisSpeeds.vyMetersPerSecond = speeds.vyMetersPerSecond;

    if (isOpenLoop) {
      this.setControl(
          this.applyChassisSpeedsRequest
              .withDriveRequestType(SwerveModule.DriveRequestType.OpenLoopVoltage)
              .withSteerRequestType(SwerveModule.SteerRequestType.MotionMagicExpo)
              .withSpeeds(speeds)
              .withCenterOfRotation(this.centerOfRotation));
    } else {
      this.setControl(
          this.applyChassisSpeedsRequest
              .withDriveRequestType(SwerveModule.DriveRequestType.Velocity)
              .withSteerRequestType(SwerveModule.SteerRequestType.MotionMagicExpo)
              .withSpeeds(speeds)
              .withCenterOfRotation(this.centerOfRotation));
    }
  }

  @Override
  public void setGyroOffset(double expectedYaw) {
    try {
      m_stateLock.writeLock().lock();

      m_fieldRelativeOffset =
          getState().Pose.getRotation().plus(Rotation2d.fromDegrees(expectedYaw));
    } finally {
      m_stateLock.writeLock().unlock();
    }
  }

  @Override
  public void setCenterOfRotation(Translation2d centerOfRotation) {
    this.centerOfRotation = centerOfRotation;
  }

  @Override
  public void resetPose(Pose2d pose) {
    this.seedFieldRelative(pose);
  }

  @Override
  public void setDriveMotorCurrent(double amps) {
    // ensure that the SwerveDrivetrain class doesn't control either motor
    this.setControl(idleRequest);

    for (int i = 0; i < this.Modules.length; i++) {
      this.Modules[i].getDriveMotor().setControl(driveCurrentRequests[i].withOutput(amps));
    }
  }

  @Override
  public void setSteerMotorCurrent(double amps) {
    // ensure that the SwerveDrivetrain class doesn't control either motor
    this.setControl(idleRequest);

    for (int i = 0; i < this.Modules.length; i++) {
      this.Modules[i].getSteerMotor().setControl(steerCurrentRequests[i].withOutput(amps));
    }
  }

  @Override
  public void setBrakeMode(boolean enable) {
    for (SwerveModule swerveModule : this.Modules) {
      MotorOutputConfigs config = new MotorOutputConfigs();
      swerveModule.getDriveMotor().getConfigurator().refresh(config);
      config.NeutralMode = enable ? NeutralModeValue.Brake : NeutralModeValue.Coast;
      swerveModule.getDriveMotor().getConfigurator().apply(config);
    }
  }

  /**
   * Returns the average current of the swerve module drive motors in amps.
   *
   * @return the average current of the swerve module drive motors in amps
   */
  private double getAverageDriveCurrent(DrivetrainIOInputsCollection inputs) {
    double totalCurrent = 0.0;
    for (SwerveIOInputs swerveInputs : inputs.swerve) {
      totalCurrent += swerveInputs.driveStatorCurrentAmps;
    }
    return totalCurrent / inputs.swerve.length;
  }

  public Pose2d getEstimatedPosition() {
    return this.m_odometry.getEstimatedPosition();
  }

  public void resetPosition(
      Rotation2d gyroAngle, SwerveModulePosition[] modulePositions, Pose2d poseMeters) {
    this.m_odometry.resetPosition(gyroAngle, modulePositions, poseMeters);
  }

  public Pose2d updateWithTime(
      double currentTimeSeconds, Rotation2d gyroAngle, SwerveModulePosition[] modulePositions) {
    return this.m_odometry.updateWithTime(currentTimeSeconds, gyroAngle, modulePositions);
  }

  private static ClosedLoopOutputType getSteerClosedLoopOutputType() {
    if (RobotConfig.getInstance().getSwerveSteerControlMode()
        == RobotConfig.SWERVE_CONTROL_MODE.TORQUE_CURRENT_FOC) {
      return ClosedLoopOutputType.TorqueCurrentFOC;
    } else {
      return ClosedLoopOutputType.Voltage;
    }
  }

  private static ClosedLoopOutputType getDriveClosedLoopOutputType() {
    if (RobotConfig.getInstance().getSwerveDriveControlMode()
        == RobotConfig.SWERVE_CONTROL_MODE.TORQUE_CURRENT_FOC) {
      return ClosedLoopOutputType.TorqueCurrentFOC;
    } else {
      return ClosedLoopOutputType.Voltage;
    }
  }
}
