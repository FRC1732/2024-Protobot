package frc.robot.subsystems.intake;

import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {

  @AutoLog
  public static class IntakeIOInput {
    double intakeMainMotorSpeed = 0.0;
    double intakeCentererMotorSpeed = 0.0;
  }

  private IntakeIOInputAutoLogged inputs = new IntakeIOInputAutoLogged();

  private CANSparkMax intakeMainMotor;
  private CANSparkMax intakeCentererMotor;

  public static Double intakeMainMotorSpeed = 0.80;
  public static Double intakeCentererMotorSpeed = 0.80;

  /** Creates a new Intake. */
  public Intake() {

    intakeMainMotor =
        new CANSparkMax(
            IntakeConstants.INTAKE_CENTERER_MOTOR_CAN_ID, CANSparkMax.MotorType.kBrushless);
    intakeCentererMotor =
        new CANSparkMax(IntakeConstants.INTAKE_MAIN_MOTOR_CAN_ID, CANSparkMax.MotorType.kBrushless);

    intakeCentererMotor.setInverted(IntakeConstants.INTAKE_MAIN_MOTOR_INVERTED);
  }

  public void runIntake() {
    intakeMainMotor.set(intakeMainMotorSpeed);
    intakeCentererMotor.set(intakeCentererMotorSpeed);
  }

  public void runIntakeOut() {
    intakeMainMotor.set(intakeMainMotorSpeed * -1);
    intakeCentererMotor.set(intakeCentererMotorSpeed * -1);
  }

  public void stopIntake() {
    intakeMainMotor.set(0);
    intakeCentererMotor.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (IntakeConstants.INTAKE_LOGGING) {
      updateInputs();
    }
  }

  private void updateInputs() {
    inputs.intakeMainMotorSpeed = intakeMainMotor.get();
    inputs.intakeCentererMotorSpeed = intakeCentererMotor.get();

    Logger.processInputs("Intake", inputs);
  }
}
