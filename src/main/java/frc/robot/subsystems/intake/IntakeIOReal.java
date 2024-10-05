package frc.robot.subsystems.intake;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants;
import frc.robot.Constants.CAN;
import frc.robot.Constants.IntakeConstants;

public class IntakeIOReal implements IntakeIO {
  private DigitalInput breamBreak;
  private TalonFX firstIntake;
  private TalonFX secondIntake;

  public IntakeIOReal() {
    System.out.println("[Init] Creating IntakeIOReal");

    breamBreak = new DigitalInput(IntakeConstants.kBeamBreakChannel);
    firstIntake = new TalonFX(CAN.kFirstIntakeCanId);
    secondIntake = new TalonFX(CAN.kSecondIntakeCanId);

    firstIntake.clearStickyFaults();
    firstIntake.setInverted(true);
    firstIntake.setNeutralMode(NeutralModeValue.Coast);

    secondIntake.clearStickyFaults();
    secondIntake.setInverted(true);
    secondIntake.setNeutralMode(NeutralModeValue.Coast);
    // motor.setPositionConversionFactor(2.0 * Math.PI);
    // encoder.setVelocityConversionFactor(2.0 * Math.PI / 60.0 / 5.0);
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    inputs.firstIntakeVelocityRadPerSec = firstIntake.getVelocity().getValueAsDouble() * 
        Constants.IntakeConstants.firstStageGearRatio;
    inputs.secondIntakeVelocityRadPerSec = secondIntake.getVelocity().getValueAsDouble() * 
        Constants.IntakeConstants.secondStageGearRatio;

    inputs.firstIntakeAppliedVolts =
        firstIntake.getMotorVoltage().getValueAsDouble() * firstIntake.getSupplyVoltage().getValueAsDouble();
    inputs.secondIntakeAppliedVolts =
        secondIntake.getMotorVoltage().getValueAsDouble() * secondIntake.getSupplyVoltage().getValueAsDouble();

    inputs.firstIntakeCurrAmps = firstIntake.getSupplyCurrent().getValueAsDouble();
    inputs.secondIntakeAppliedVolts = secondIntake.getSupplyCurrent().getValueAsDouble();

    inputs.hasNote = !breamBreak.get();
  }

  @Override
  public void setVoltage(double voltage) {
    firstIntake.setVoltage(voltage);
    secondIntake.setVoltage(voltage);
  }

  @Override
  public void setBrakeMode(boolean brake) {
    firstIntake.setNeutralMode(brake ? NeutralModeValue.Brake : NeutralModeValue.Coast);
    secondIntake.setNeutralMode(brake ? NeutralModeValue.Brake : NeutralModeValue.Coast);
  }
}
