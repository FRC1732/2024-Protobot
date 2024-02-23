package frc.robot.configs;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.lib.team3061.RobotConfig;
import frc.lib.team3061.drivetrain.swerve.SwerveConstants.SwerveType;

/*
 * Refer to the README for how to represent your robot's configuration. For more information on
 * these methods, refer to the documentation in the RobotConfig class.
 */
public class DefaultRobotConfig extends RobotConfig {

  // REMINDER: offsets are in rotations
  private static final int FRONT_LEFT_MODULE_DRIVE_MOTOR = 10;
  private static final int FRONT_LEFT_MODULE_STEER_MOTOR = 11;
  private static final int FRONT_LEFT_MODULE_STEER_ENCODER = 12;
  private static final double FRONT_LEFT_MODULE_STEER_OFFSET_ROT =
      0.456787109375 + .5 + (0) / 360 * .01; // input degree values inside the parenthesis

  private static final int FRONT_RIGHT_MODULE_DRIVE_MOTOR = 40;
  private static final int FRONT_RIGHT_MODULE_STEER_MOTOR = 41;
  private static final int FRONT_RIGHT_MODULE_STEER_ENCODER = 42;
  private static final double FRONT_RIGHT_MODULE_STEER_OFFSET_ROT =
      -0.22607421875 + (0) / 360 * .01; // input degree values inside the parenthesis

  private static final int BACK_LEFT_MODULE_DRIVE_MOTOR = 20;
  private static final int BACK_LEFT_MODULE_STEER_MOTOR = 21;
  private static final int BACK_LEFT_MODULE_STEER_ENCODER = 22;
  private static final double BACK_LEFT_MODULE_STEER_OFFSET_ROT =
      -0.031494140625 + .5 + (0) / 360 * .01; // input degree values inside the parenthesis

  private static final int BACK_RIGHT_MODULE_DRIVE_MOTOR = 30;
  private static final int BACK_RIGHT_MODULE_STEER_MOTOR = 31;
  private static final int BACK_RIGHT_MODULE_STEER_ENCODER = 32;
  private static final double BACK_RIGHT_MODULE_STEER_OFFSET_ROT =
      -0.186767578125 + (0) / 360 * .01; // input degree values inside the parenthesis

  private static final int GYRO_ID = 7;

  private static final double TRACKWIDTH_METERS = 0.55245; // 21.75 inches
  private static final double WHEELBASE_METERS = 0.55245; // 21.75 inches
  private static final double ROBOT_WIDTH_WITH_BUMPERS = 0.8636; // 34 inches
  private static final double ROBOT_LENGTH_WITH_BUMPERS = 0.9779; // 38.5 inches

  // FIXME: tune PID values for the angle and drive motors for the swerve modules

  /* Angle Motor PID Values */
  private static final double ANGLE_KP = 100; // 0.6 last year
  private static final double ANGLE_KI = 0.0;
  private static final double ANGLE_KD = 0.2; // 12 last year

  /* Drive Motor PID Values */
  private static final double DRIVE_KP = 3; // 0.1 last year
  private static final double DRIVE_KI = 0.0;
  private static final double DRIVE_KD = 0.0;

  // FIXME: characterize the drivetrain and update these constants
  private static final double DRIVE_KS = 0.55493; // divide by 12 last year
  private static final double DRIVE_KV = 2.3014; // divide by 12 last year
  private static final double DRIVE_KA = 0.12872; // divide by 12 last year

  // FIXME: specify the type of swerve module (MK4 and MK4i are supported)
  private static final SwerveType SWERVE_TYPE = SwerveType.MK4I;

  // FIXME: determine maximum velocities empirically
  private static final double MAX_VELOCITY_METERS_PER_SECOND = 4.25; // 4.96824 last year
  private static final double MAX_COAST_VELOCITY_METERS_PER_SECOND = 0.05;

  // FIXME: specify the name of the CANivore CAN FD bus as appropriate (an empty string uses the
  // default CAN bus)
  private static final String CAN_BUS_NAME = "Monke";

  // FIXME: specify the name of the camera used for detecting AprilTags
  private static final String CAMERA_NAME = "";

  // FIXME: update this with the actual transform from the robot to the camera
  private static final Transform3d ROBOT_TO_CAMERA =
      new Transform3d(new Translation3d(0, 0, 0), new Rotation3d(0, 0, 0));

  // FIXME: specify the configuration for pneumatics
  private static final int PNEUMATICS_HUB_ID = 3;
  private static final int FLOW_SENSOR_CHANNEL = 0;
  private static final int REV_HIGH_PRESSURE_SENSOR_CHANNEL = 0;
  private static final int REV_LOW_PRESSURE_SENSOR_CHANNEL = 1;

  // FIXME: specify maximum velocity and acceleration and tune PID values for auto paths

  private static final double AUTO_MAX_SPEED_METERS_PER_SECOND =
      2.0; // last year was max velocity * 0.625
  private static final double AUTO_MAX_ACCELERATION_METERS_PER_SECOND_SQUARED =
      2.0; // last year was 1.9
  private static final double AUTO_DRIVE_P_CONTROLLER = 0.2402346041055719; // last year was 1.0
  private static final double AUTO_DRIVE_I_CONTROLLER = 0.0;
  private static final double AUTO_DRIVE_D_CONTROLLER = 0.0;
  private static final double AUTO_TURN_P_CONTROLLER = 14.414076246334309; // last year was 7
  private static final double AUTO_TURN_I_CONTROLLER = 0.0;
  private static final double AUTO_TURN_D_CONTROLLER = 0.28828152492668624; // last year was 0

  @Override
  public double getSwerveAngleKP() {
    return ANGLE_KP;
  }

  @Override
  public double getSwerveAngleKI() {
    return ANGLE_KI;
  }

  @Override
  public double getSwerveAngleKD() {
    return ANGLE_KD;
  }

  @Override
  public double getSwerveDriveKP() {
    return DRIVE_KP;
  }

  @Override
  public double getSwerveDriveKI() {
    return DRIVE_KI;
  }

  @Override
  public double getSwerveDriveKD() {
    return DRIVE_KD;
  }

  @Override
  public double getDriveKS() {
    return DRIVE_KS;
  }

  @Override
  public double getDriveKV() {
    return DRIVE_KV;
  }

  @Override
  public double getDriveKA() {
    return DRIVE_KA;
  }

  @Override
  public SwerveType getSwerveType() {
    return SWERVE_TYPE;
  }

  @Override
  public int[] getSwerveDriveMotorCANIDs() {
    return new int[] {
      FRONT_LEFT_MODULE_DRIVE_MOTOR,
      FRONT_RIGHT_MODULE_DRIVE_MOTOR,
      BACK_LEFT_MODULE_DRIVE_MOTOR,
      BACK_RIGHT_MODULE_DRIVE_MOTOR
    };
  }

  @Override
  public int[] getSwerveSteerMotorCANIDs() {
    return new int[] {
      FRONT_LEFT_MODULE_STEER_MOTOR,
      FRONT_RIGHT_MODULE_STEER_MOTOR,
      BACK_LEFT_MODULE_STEER_MOTOR,
      BACK_RIGHT_MODULE_STEER_MOTOR
    };
  }

  @Override
  public int[] getSwerveSteerEncoderCANIDs() {
    return new int[] {
      FRONT_LEFT_MODULE_STEER_ENCODER,
      FRONT_RIGHT_MODULE_STEER_ENCODER,
      BACK_LEFT_MODULE_STEER_ENCODER,
      BACK_RIGHT_MODULE_STEER_ENCODER
    };
  }

  @Override
  public double[] getSwerveSteerOffsets() {
    return new double[] {
      FRONT_LEFT_MODULE_STEER_OFFSET_ROT,
      FRONT_RIGHT_MODULE_STEER_OFFSET_ROT,
      BACK_LEFT_MODULE_STEER_OFFSET_ROT,
      BACK_RIGHT_MODULE_STEER_OFFSET_ROT
    };
  }

  @Override
  public int getGyroCANID() {
    return GYRO_ID;
  }

  @Override
  public double getTrackwidth() {
    return TRACKWIDTH_METERS;
  }

  @Override
  public double getWheelbase() {
    return WHEELBASE_METERS;
  }

  @Override
  public double getRobotWidthWithBumpers() {
    return ROBOT_WIDTH_WITH_BUMPERS;
  }

  @Override
  public double getRobotLengthWithBumpers() {
    return ROBOT_LENGTH_WITH_BUMPERS;
  }

  @Override
  public Transform3d[] getRobotToCameraTransforms() {
    return new Transform3d[] {ROBOT_TO_CAMERA};
  }

  @Override
  public double getRobotMaxVelocity() {
    return MAX_VELOCITY_METERS_PER_SECOND;
  }

  @Override
  public double getRobotMaxCoastVelocity() {
    return MAX_COAST_VELOCITY_METERS_PER_SECOND;
  }

  @Override
  public double getAutoMaxSpeed() {
    return AUTO_MAX_SPEED_METERS_PER_SECOND;
  }

  @Override
  public double getAutoMaxAcceleration() {
    return AUTO_MAX_ACCELERATION_METERS_PER_SECOND_SQUARED;
  }

  @Override
  public double getAutoDriveKP() {
    return AUTO_DRIVE_P_CONTROLLER;
  }

  @Override
  public double getAutoDriveKI() {
    return AUTO_DRIVE_I_CONTROLLER;
  }

  @Override
  public double getAutoDriveKD() {
    return AUTO_DRIVE_D_CONTROLLER;
  }

  @Override
  public double getAutoTurnKP() {
    return AUTO_TURN_P_CONTROLLER;
  }

  @Override
  public double getAutoTurnKI() {
    return AUTO_TURN_I_CONTROLLER;
  }

  @Override
  public double getAutoTurnKD() {
    return AUTO_TURN_D_CONTROLLER;
  }

  @Override
  public String getCANBusName() {
    return CAN_BUS_NAME;
  }

  @Override
  public String[] getCameraNames() {
    return new String[] {CAMERA_NAME};
  }

  @Override
  public int getPneumaticsHubCANID() {
    return PNEUMATICS_HUB_ID;
  }

  @Override
  public int getFlowSensorChannel() {
    return FLOW_SENSOR_CHANNEL;
  }

  @Override
  public int getRevHighPressureSensorChannel() {
    return REV_HIGH_PRESSURE_SENSOR_CHANNEL;
  }

  @Override
  public int getRevLowPressureSensorChannel() {
    return REV_LOW_PRESSURE_SENSOR_CHANNEL;
  }
}
