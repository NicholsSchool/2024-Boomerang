package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLogOutput;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.robot.Constants.CAN;
import frc.robot.util.RevDistanceSensor.Rev2mDistanceSensor;
import frc.robot.util.RevDistanceSensor.Rev2mDistanceSensor.Port;
import frc.robot.util.RevDistanceSensor.Rev2mDistanceSensor.RangeProfile;
import frc.robot.util.RevDistanceSensor.Rev2mDistanceSensor.Unit;

public class IntakeIOReal implements IntakeIO {
  // private DigitalInput breamBreak;
  private TalonFX motor;
  private Rev2mDistanceSensor distSensor;

  public IntakeIOReal() {
    System.out.println("[Init] Creating IntakeIOReal");

    // breamBreak = new DigitalInput(IntakeConstants.kBeamBreakChannel);
    motor = new TalonFX(CAN.kIntakeCanId);
    motor.clearStickyFaults();
    motor.setInverted(true);
    motor.setNeutralMode(NeutralModeValue.Coast);

    distSensor = new Rev2mDistanceSensor(Port.kOnboard); //i2c port
    distSensor.setDistanceUnits(Unit.kInches);
    distSensor.setRangeProfile(RangeProfile.kDefault);
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

  @AutoLogOutput
  public double getDistSensorRangeInches() {
    return distSensor.getRange();
  }
}
