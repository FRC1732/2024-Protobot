package frc.robot.subsystems.intake;

import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
  private CANSparkMax intakeCanSparkMax;
  private CANSparkMax centererCanSparkMax;

  public static Double intakeMotorLSpeed = 0.80;
  public static Double intakeMotorRSpeed = 0.80;

  /** Creates a new Intake. */
  public Intake() {

    intakeCanSparkMax =
        new CANSparkMax(IntakeConstants.INTAKE_MOTOR_L_CAN_ID, CANSparkMax.MotorType.kBrushless);
    centererCanSparkMax =
        new CANSparkMax(IntakeConstants.INTAKE_MOTOR_R_CAN_ID, CANSparkMax.MotorType.kBrushless);

    intakeCanSparkMax.setInverted(IntakeConstants.INTAKE_MOTOR_L_INVERTED);
    centererCanSparkMax.setInverted(IntakeConstants.INTAKE_MOTOR_R_INVERTED);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void runIntake() {
    intakeCanSparkMax.set(intakeMotorLSpeed);
    centererCanSparkMax.set(intakeMotorRSpeed);
  }

  public void runIntakeOut() {
    intakeCanSparkMax.set(intakeMotorLSpeed * -1);
    centererCanSparkMax.set(intakeMotorRSpeed * -1);
  }

  public void stopIntake() {
    intakeCanSparkMax.set(0);
    centererCanSparkMax.set(0);
  }
}
