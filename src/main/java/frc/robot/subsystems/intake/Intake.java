package frc.robot.subsystems.intake;

import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
  private CANSparkMax intakeLCanSparkMax;
  private CANSparkMax intakeRCanSparkMax;

  public static Double intakeMotorLSpeed = 0.40;
  public static Double intakeMotorRSpeed = 0.40;

  /** Creates a new Intake. */
  public Intake() {

    intakeLCanSparkMax =
        new CANSparkMax(IntakeConstants.INTAKE_MOTOR_L_CAN_ID, CANSparkMax.MotorType.kBrushless);
    intakeRCanSparkMax =
        new CANSparkMax(IntakeConstants.INTAKE_MOTOR_R_CAN_ID, CANSparkMax.MotorType.kBrushless);

    intakeLCanSparkMax.setInverted(IntakeConstants.INTAKE_MOTOR_L_INVERTED);
    intakeRCanSparkMax.setInverted(IntakeConstants.INTAKE_MOTOR_R_INVERTED);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void runIntake() {
    intakeLCanSparkMax.set(intakeMotorLSpeed);
    intakeRCanSparkMax.set(intakeMotorRSpeed);
  }

  public void stopIntake() {
    intakeLCanSparkMax.set(0);
    intakeRCanSparkMax.set(0);
  }
}
