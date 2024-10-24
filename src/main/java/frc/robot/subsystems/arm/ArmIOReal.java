package frc.robot.subsystems.arm;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.AnalogInput;
import frc.robot.Constants;
import frc.robot.Constants.CAN;

public class ArmIOReal implements ArmIO {

  private TalonFX lShoulder;
  private TalonFX rShoulder;
  private AnalogInput absoluteEncoder;

  public ArmIOReal() {
    lShoulder = new TalonFX(CAN.kShoulderL);
    rShoulder = new TalonFX(CAN.kShoulderR);

    var config = new TalonFXConfiguration();
    config.CurrentLimits.StatorCurrentLimit = Constants.ArmConstants.ARM_CURRENT_LIMIT;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    lShoulder.getConfigurator().apply(config);
    rShoulder.setControl(new Follower(CAN.kShoulderL, true));

    absoluteEncoder = new AnalogInput(Constants.ArmConstants.kThroughBoreChannel);
  }

  @Override
  public void updateInputs(ArmIOInputs inputs) {
    inputs.angleDegs = this.getArmAngleDeg();
    inputs.angleRads = Math.toRadians(this.getArmAngleDeg());
    inputs.velocityRadsPerSec =
        new double[] {
          lShoulder.getVelocity().getValueAsDouble(), rShoulder.getVelocity().getValueAsDouble()
        };
    inputs.appliedOutput =
        new double[] {
          lShoulder.getMotorVoltage().getValueAsDouble(),
          rShoulder.getMotorVoltage().getValueAsDouble()
        };
    inputs.busVoltage =
        new double[] {
          lShoulder.getSupplyVoltage().getValueAsDouble(),
          rShoulder.getSupplyVoltage().getValueAsDouble()
        };
    inputs.appliedVolts =
        new double[] {
          inputs.busVoltage[0] * inputs.appliedOutput[0],
          inputs.busVoltage[1] * inputs.appliedOutput[1]
        };
    inputs.currentAmps =
        new double[] {
          lShoulder.getStatorCurrent().getValueAsDouble(),
          rShoulder.getStatorCurrent().getValueAsDouble()
        };
  }

  @Override
  public void setVoltage(double voltage) {
    lShoulder.setVoltage(voltage);
  }

  private double getArmAngleDeg() {
    return absoluteEncoder.getVoltage() / 12.0 * 360.0;
  }
}
