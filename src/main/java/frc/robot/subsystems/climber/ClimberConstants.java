package frc.robot.subsystems.climber;

public class ClimberConstants {

  public static final int CLIMBER_LEFT_MOTOR_CAN_ID = 13;
  public static final int CLIMBER_RIGHT_MOTOR_CAN_ID = 29;

  public static final double CLIMBER_LEFT_P = 0;
  public static final double CLIMBER_LEFT_I = 0;
  public static final double CLIMBER_LEFT_D = 0;

  public static final double CLIMBER_RIGHT_P = 0;
  public static final double CLIMBER_RIGHT_I = 0;
  public static final double CLIMBER_RIGHT_D = 0;

  public static final double MAX_SETPOINT_INCHES = 22.6;
  public static final double MIN_SETPOINT_INCHES = 0.5;

  public static final double CLIMBER_CONVERSION_FACTOR = 1.0 / 28.3 * 1.432 * Math.PI; //

  public static final boolean CLIMBER_TESTING = true;
  public static final boolean CLIMBER_LOGGING = true;
}
