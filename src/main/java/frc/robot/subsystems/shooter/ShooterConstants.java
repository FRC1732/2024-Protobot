package frc.robot.subsystems.shooter;

public class ShooterConstants {

  private static final String CONSTRUCTOR_EXCEPTION = "constant class";

  private ShooterConstants() {
    throw new IllegalStateException(CONSTRUCTOR_EXCEPTION);
  }

  public static final boolean TESTING = false;
  public static final String SUBSYSTEM_NAME = "Shooter";

  public static final int SHOOTER_TILT_MOTOR_CAN_ID = 1; //TODO: set to comp bot values
  public static final int SHOOTER_HIGH_MOTOR_CAN_ID = 2; //TODO: set to comp bot values
  public static final int SHOOTER_LOW_MOTOR_CAN_ID = 3; //TODO: set to comp bot values

  public static final boolean SHOOTER_HIGH_MOTOR_INVERTED = true;

  public static final double SHOOTER_SPEED_115 = 0.0;
  public static final double SHOOTER_SPEED_125 = 0.0;
  public static final double SHOOTER_SPEED_150 = 0.0;

  public static final double SHOOTER_TILT_P = 0;
  public static final double SHOOTER_TILT_I = 0;
  public static final double SHOOTER_TILT_D = 0;

  public static final double SHOOTER_TILT_SPEAKER_115_SETPOINT = 0;
  public static final double SHOOTER_TILT_SPEAKER_125_SETPOINT = 0;
  public static final double SHOOTER_TILT_SPEAKER_150_SETPOINT = 0;
  public static final double SHOOTER_TILT_AMP_SETPOINT = 0;
  public static final double SHOOTER_TILT_TRAP_SETPOINT = 0;
}
