package frc.robot.subsystems.shooterPose;

public class ShooterPoseConstants {

  public static final boolean SHOOTER_POSE_TESTING = false;

  public static final int SHOOTER_POSE_LEFT_MOTOR_CAN_ID = 1; // TODO: assign correct CAN IDs
  public static final int SHOOTER_POSE_RIGHT_MOTOR_CAN_ID = 2; // TODO: assign correct CAN IDs
  public static final int SHOOTER_TILT_MOTOR_CAN_ID = 3; // TODO: assign correct CAN IDs

  public static final double SHOOTER_POSE_RIGHT_MOTOR_P = 0;
  public static final double SHOOTER_POSE_RIGHT_MOTOR_I = 0;
  public static final double SHOOTER_POSE_RIGHT_MOTOR_D = 0;

  public static final double SHOOTER_POSE_SPEAKER_SETPOINT = 0;
  public static final double SHOOTER_POSE_AMP_SETPOINT = 0;
  public static final double SHOOTER_POSE_TRAP_SETPOINT = 0;

  public static final double SHOOTER_TILT_P = 0;
  public static final double SHOOTER_TILT_I = 0;
  public static final double SHOOTER_TILT_D = 0;

  public static final double SHOOTER_TILT_CONVERSION_FACTOR_DPR = 5.625; //degrees per revolution
  public static final double SHOOTER_POSE_CONVERSION_FACTOR_IPR = 0.36673; //inches per revolution

  public static final double SHOOTER_TILT_SPEAKER_115_SETPOINT = 0;
  public static final double SHOOTER_TILT_SPEAKER_125_SETPOINT = 0;
  public static final double SHOOTER_TILT_SPEAKER_150_SETPOINT = 0;
  public static final double SHOOTER_TILT_AMP_SETPOINT = 0;
  public static final double SHOOTER_TILT_TRAP_SETPOINT = 0;
}
