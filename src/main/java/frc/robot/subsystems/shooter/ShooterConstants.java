package frc.robot.subsystems.shooter;

public class ShooterConstants {

  private static final String CONSTRUCTOR_EXCEPTION = "constant class";

  private ShooterConstants() {
    throw new IllegalStateException(CONSTRUCTOR_EXCEPTION);
  }

  public static final boolean DEBUGGING = false;
  public static final boolean TESTING = false;
  public static final String SUBSYSTEM_NAME = "Shooter";

  public static final int SHOOTER_MOTOR_HIGH_CAN_ID = 61;
  public static final int SHOOTER_MOTOR_LOW_CAN_ID = 62;

  public static final boolean SHOOTER_MOTOR_HIGH_INVERTED = false;
  public static final boolean SHOOTER_MOTOR_LOW_INVERTED = true;
  public static final boolean SHOOTER_MOTOR_TILT_INVERTED = false;
  public static final boolean SHOOTER_MOTOR_LEFT_INVERTED = false;
  public static final boolean SHOOTER_MOTOR_RIGHT_INVERTED = false;

  public static final double SHOOTER_TILT_P = 0;
  public static final double SHOOTER_TILT_I = 0;
  public static final double SHOOTER_TILT_D = 0;

  public static final double SHOOTER_TILT_SPEAKER_SETPOINT = 0;
  public static final double SHOOTER_TILT_AMP_SETPOINT = 0;
  public static final double SHOOTER_TILT_TRAP_SETPOINT = 0;

  public static final int SHOOTER_TILT_MOTOR_CAN_ID = 1;
}
