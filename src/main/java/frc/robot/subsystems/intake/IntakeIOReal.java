package frc.robot.subsystems.intake;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants.CAN;

public class IntakeIOReal implements IntakeIO {
  // private DigitalInput breamBreak;
  private TalonFX motor;
  private DigitalInput beamBreak;

  public IntakeIOReal() {
    System.out.println("[Init] Creating IntakeIOReal");

    // breamBreak = new DigitalInput(IntakeConstants.kBeamBreakChannel);
    motor = new TalonFX(CAN.kIntakeCanId);
    motor.clearStickyFaults();
    motor.setInverted(true);
    motor.setNeutralMode(NeutralModeValue.Coast);

    beamBreak = new DigitalInput(CAN.kBeamBreakChannel);
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    inputs.velocityRadPerSec = motor.getVelocity().getValueAsDouble() * 2.0 * Math.PI / 60.0 / 5.0;
    inputs.appliedVolts =
        motor.getMotorVoltage().getValueAsDouble() * motor.getSupplyVoltage().getValueAsDouble();
    inputs.currentAmps = motor.getStatorCurrent().getValueAsDouble();
    inputs.hasNote = false;
  }

  @Override
  public void setVoltage(double voltage) {
    motor.setVoltage(voltage);
  }

  @Override
  public void setBrakeMode(boolean brake) {
    motor.setNeutralMode(brake ? NeutralModeValue.Brake : NeutralModeValue.Coast);
  }
}
