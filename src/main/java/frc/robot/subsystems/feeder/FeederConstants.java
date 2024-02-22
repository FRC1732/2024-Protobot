package frc.robot.subsystems.feeder;

public class FeederConstants {

  private static final String CONSTRUCTOR_EXCEPTION = "constant class";

  private FeederConstants() {
    throw new IllegalStateException(CONSTRUCTOR_EXCEPTION);
  }

  public static final boolean DEBUGGING = false;
  public static final boolean TESTING = false;
  public static final String SUBSYSTEM_NAME = "Feeder";

  public static final int FEEDER_MOTOR_CAN_ID = 50;
  public static final boolean SHOOTER_MOTOR_LEFT_INVERTED = true;
  public static final boolean SHOOTER_MOTOR_RIGHT_INVERTED = false;

  public static final int ANALOG_INPUT_LOCATION = 1;

  public static final Double FEEDER_MOTOR_SPEED = .4;
  public static final Double FEEDER_BRAKE_SPEED = 0.0;//-.04;

  public static final boolean FEEDER_LOGGING = true;
}
