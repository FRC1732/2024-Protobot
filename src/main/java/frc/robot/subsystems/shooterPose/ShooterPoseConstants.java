package frc.robot.subsystems.shooterPose;

public class ShooterPoseConstants {

  public static final boolean SHOOTER_POSE_TESTING = false;

  public static final int SHOOTER_HEIGHT_LEFT_MOTOR_CAN_ID = 60; // TODO: assign correct CAN IDs
  public static final int SHOOTER_HEIGHT_RIGHT_MOTOR_CAN_ID = 61; // TODO: assign correct CAN IDs
  public static final int SHOOTER_TILT_MOTOR_CAN_ID = 62; // TODO: assign correct CAN IDs

  public static final double SHOOTER_HEIGHT_MAX_VELOCITY = 35.17; // in/s
  public static final double SHOOTER_HEIGHT_MAX_ACCELERATION = 100; // in/s^2 838.75 calculated max
  public static final double SHOOTER_HEIGHT_KP = 0;
  public static final double SHOOTER_HEIGHT_KI = 0;
  public static final double SHOOTER_HEIGHT_KD = 0;
  public static final double SHOOTER_HEIGHT_KG = 0.23; // V
  public static final double SHOOTER_HEIGHT_KV = 0.334010; // V*s/in
  public static final double SHOOTER_HEIGHT_KA = 0.000762; // V*s^2/in
  public static final double SHOOTER_HEIGHT_KS = 0;

  public static final double SHOOTER_TILT_MAX_VELOCITY = 506.25; // deg/s
  public static final double SHOOTER_TILT_MAX_ACCELERATION = 1000; // deg/s^2 25000 calculated max
  public static final double SHOOTER_TILT_KP = 0;
  public static final double SHOOTER_TILT_KI = 0;
  public static final double SHOOTER_TILT_KD = 0;
  public static final double SHOOTER_TILT_KG = 0.37; // V
  public static final double SHOOTER_TILT_KV = 0.021777; // V*s/deg
  public static final double SHOOTER_TILT_KA = 0.000083; // V*s^2/deg
  public static final double SHOOTER_TILT_KS = 0;

  public static final double SHOOTER_TILT_CONVERSION_FACTOR_DPR =
      5.625; // degrees per motor revolution (360 / reduction = 360 / 64)
  public static final double SHOOTER_HEIGHT_CONVERSION_FACTOR_IPR =
      0.366519; // inches per motor revolution (spool diam * 3.14159 / reduction = 1.75 * 3.14159 /
  // 15)

  public static final double SHOOTER_TILT_NEUTRAL_SETPOINT = 0 * SHOOTER_TILT_CONVERSION_FACTOR_DPR;
  public static final double SHOOTER_TILT_SPEAKER_115_SETPOINT =
      30 * SHOOTER_TILT_CONVERSION_FACTOR_DPR;
  public static final double SHOOTER_TILT_SPEAKER_125_SETPOINT =
      0 * SHOOTER_TILT_CONVERSION_FACTOR_DPR;
  public static final double SHOOTER_TILT_SPEAKER_150_SETPOINT =
      50 * SHOOTER_TILT_CONVERSION_FACTOR_DPR;
  public static final double SHOOTER_TILT_AMP_SETPOINT = 0 * SHOOTER_TILT_CONVERSION_FACTOR_DPR;
  public static final double SHOOTER_TILT_TRAP_SETPOINT = 0 * SHOOTER_TILT_CONVERSION_FACTOR_DPR;
  public static final double SHOOTER_TILT_SOURCE_SETPOINT = 0 * SHOOTER_TILT_CONVERSION_FACTOR_DPR;
  public static final double SHOOTER_TILT_FEEDING_SETPOINT = 0 * SHOOTER_TILT_CONVERSION_FACTOR_DPR;

  public static final double SHOOTER_POSE_NEUTRAL_SETPOINT =
      0 * SHOOTER_HEIGHT_CONVERSION_FACTOR_IPR;
  public static final double SHOOTER_POSE_SPEAKER_SETPOINT =
      0 * SHOOTER_HEIGHT_CONVERSION_FACTOR_IPR;
  public static final double SHOOTER_POSE_AMP_SETPOINT = 0 * SHOOTER_HEIGHT_CONVERSION_FACTOR_IPR;
  public static final double SHOOTER_POSE_TRAP_SETPOINT = 0 * SHOOTER_HEIGHT_CONVERSION_FACTOR_IPR;
  public static final double SHOOTER_POSE_SOURCE_SETPOINT =
      0 * SHOOTER_HEIGHT_CONVERSION_FACTOR_IPR;
  public static final double SHOOTER_POSE_FEEDING_SETPOINT =
      0 * SHOOTER_HEIGHT_CONVERSION_FACTOR_IPR;
}
