package frc.robot.subsystems.shooterPose;

public class ShooterPoseConstants {

  public static final boolean SHOOTER_POSE_TESTING = true;
  public static final boolean SHOOTER_POSE_LOGGING = true;

  public static final double SHOOTER_PID_PERIOD_SEC = 0.02;

  public static final int SHOOTER_HEIGHT_LEFT_MOTOR_CAN_ID = 60;
  public static final int SHOOTER_HEIGHT_RIGHT_MOTOR_CAN_ID = 61;
  public static final int SHOOTER_TILT_MOTOR_CAN_ID = 62;

  public static final double MIN_SHOOTER_HEIGHT_INCHES = 0;
  public static final double MAX_SHOOTER_HEIGHT_INCHES = 16;
  public static final double SHOOTER_HEIGHT_GOAL_TOLERANCE_INCHES = 1;

  public static final double MIN_SHOOTER_TILT_DEGREES = -32.5;
  public static final double MAX_SHOOTER_TILT_DEGREES = 102.5;
  public static final double SHOOTER_TILT_ABSOLUTE_OFFSET =
      60.5 - 36.366 + 4.428 + 291.062 - 90
          + 20; // 0 is a flat shot, CCW is positive, shooter facing left
  public static final double SHOOTER_TILT_GOAL_TOLERANCE_DEGREES = 2;
  public static final double SHOOTER_TILT_MAX_ABOLUTE_RELATIVE_ERROR_DEG = 10;
  public static final double SHOOTER_TILT_COG_OFFSET = 37.79;

  public static final double SHOOTER_HEIGHT_MAX_VELOCITY = 35.17; // in/s
  public static final double SHOOTER_HEIGHT_MAX_ACCELERATION = 100; // in/s^2 838.75 calculated max
  public static final double SHOOTER_HEIGHT_KP = 0.1;
  public static final double SHOOTER_HEIGHT_KI = 0;
  public static final double SHOOTER_HEIGHT_KD = 0;
  public static final double SHOOTER_HEIGHT_KG = 0.23 / 12 * .8; // V
  public static final double SHOOTER_HEIGHT_KV = 0; // 0.334010 / 12 * .8; // V*s/in
  public static final double SHOOTER_HEIGHT_KA = 0; // 0.000762 / 12 * .8; // V*s^2/in
  public static final double SHOOTER_HEIGHT_KS = 0;

  // public static final double SHOOTER_TILT_MAX_VELOCITY = 100; // deg/s
  // public static final double SHOOTER_TILT_MAX_ACCELERATION = 200; // deg/s^2 4500 calculated max
  public static final double SHOOTER_TILT_MAX_VELOCITY = 300; // deg/s
  public static final double SHOOTER_TILT_MAX_ACCELERATION = 600; // deg/s^2 4500 calculated max
  //public static final double SHOOTER_TILT_KP = .029;
  public static final double SHOOTER_TILT_KP = .040;
  public static final double SHOOTER_TILT_KI = 0;
  public static final double SHOOTER_TILT_KD = 0;
  public static final double SHOOTER_TILT_KG = 0.77 / 12 * .45; // V
  public static final double SHOOTER_TILT_KV = 0; // 0.021778; // V*s/deg
  public static final double SHOOTER_TILT_KA = 0; // 0.000361; // V*s^2/deg
  public static final double SHOOTER_TILT_KS = 0;

  public static final double SHOOTER_TILT_DEGREES_PER_ROTATION =
      5.625; // degrees per motor revolution (360 / reduction = 360 / 64)
  public static final double SHOOTER_TILT_RPM_TO_DEGREES_PER_SECOND =
      0.09375; // RPM to deg/sec (360 / reduction / 60 = 360 / 64 / 60)
  public static final double SHOOTER_HEIGHT_INCHES_PER_ROTATION =
      0.366519; // inches per motor revolution (spool diam * 3.14 / reduction = 1.75 * 3.14 / 15)
  public static final double SHOOTER_HEIGHT_RPM_TO_INCHES_PER_SECOND =
      0.006109; // RPM to inch/sec (spool diam * 3.14 / reduction / 60 = 1.75 * 3.14 / 15 / 60)

  public static final double SHOOTER_TILT_HANDOFF_SETPOINT = -33;
  public static final double SHOOTER_TILT_SUBWOOFER_SETPOINT = -60;
  public static final double SHOOTER_TILT_AMP_SETPOINT = 40 - 5;
  public static final double SHOOTER_TILT_TRAP_SETPOINT = 30;
  public static final double SHOOTER_TILT_TRAP_PREP_SETPOINT = -28.7;

  public static final double SHOOTER_TILT_SOURCE_SETPOINT = 100;
  public static final double SHOOTER_TILT_CLIMBER_SETPOINT = 35;

  public static final double SHOOTER_HEIGHT_HANDOFF_SETPOINT = 0;
  public static final double SHOOTER_HEIGHT_SUBWOOFER_SETPOINT = 0;
  public static final double SHOOTER_HEIGHT_AMP_SETPOINT = 8 + 1;
  public static final double SHOOTER_HEIGHT_TRAP_SETPOINT = 16;
  public static final double SHOOTER_HEIGHT_TRAP_PREP_SETPOINT = 16;

  public static final double SHOOTER_HEIGHT_SOURCE_SETPOINT = 4;
  public static final double SHOOTER_HEIGHT_CLIMBER_SETPOINT = 2;
}
