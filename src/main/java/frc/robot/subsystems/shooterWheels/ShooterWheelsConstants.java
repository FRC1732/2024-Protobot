package frc.robot.subsystems.shooterWheels;

public class ShooterWheelsConstants {

  private static final String CONSTRUCTOR_EXCEPTION = "constant class";

  private ShooterWheelsConstants() {
    throw new IllegalStateException(CONSTRUCTOR_EXCEPTION);
  }

  public static final boolean SHOOTER_WHEELS_TESTING = true;
  public static final boolean SHOOTER_WHEELS_LOGGING = true;
  public static final String SUBSYSTEM_NAME = "Shooter";

  public static final int SHOOTER_HIGH_MOTOR_CAN_ID = 58; // TODO: set to comp bot values
  public static final int SHOOTER_LOW_MOTOR_CAN_ID = 59; // TODO: set to comp bot values

  public static final double SHOOTER_SPEED_FAST = 0.85;
  public static final double SHOOTER_SPEED_MEDIUM = 0.48;
  public static final double SHOOTER_SPEED_SLOW = 0.25;
  public static final double SHOOTER_SPEED_SPOIL = 0.16;
  public static final double SHOOTER_SPEED_BACKWARDS = -0.40;
  public static final double SHOOTER_SPEED_STOPPED = 0.0;

  public static final double SHOOTER_SPEED_P = 0.0015;
  public static final double SHOOTER_SPEED_I = 0.0000008;
  public static final double SHOOTER_SPEED_D = 0;
}
