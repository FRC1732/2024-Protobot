package frc.robot.subsystems.shooterWheels;

public class ShooterWheelsConstants {

  private static final String CONSTRUCTOR_EXCEPTION = "constant class";

  private ShooterWheelsConstants() {
    throw new IllegalStateException(CONSTRUCTOR_EXCEPTION);
  }

  public static final boolean TESTING = false;
  public static final String SUBSYSTEM_NAME = "Shooter";

  public static final int SHOOTER_TILT_MOTOR_CAN_ID = 1; // TODO: set to comp bot values
  public static final int SHOOTER_HIGH_MOTOR_CAN_ID = 2; // TODO: set to comp bot values
  public static final int SHOOTER_LOW_MOTOR_CAN_ID = 3; // TODO: set to comp bot values

  public static final boolean SHOOTER_HIGH_MOTOR_INVERTED = true;

  public static final double SHOOTER_SPEED_AMP = 0.0;
  public static final double SHOOTER_SPEED_TRAP = 0.0;
  public static final double SHOOTER_SPEED_115 = 0.0;
  public static final double SHOOTER_SPEED_125 = 0.0;
  public static final double SHOOTER_SPEED_150 = 0.0;
  public static final double SHOOTER_SPEED_BACKWARDS = 0.0;
}
