package frc.robot.subsystems.indexer;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.robot.Constants;
import frc.robot.Constants.CAN;

public class IndexerIOReal implements IndexerIO {
  public TalonFX indexerMotor;

  public IndexerIOReal() {
    indexerMotor = new TalonFX(CAN.kIndexer);
    var config = new TalonFXConfiguration();
    config.CurrentLimits.StatorCurrentLimit = Constants.IndexerConstants.INDEX_CURRENT_LIMIT;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive; // TODO: check
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    indexerMotor.getConfigurator().apply(config);
  }

  @Override
  public void updateInputs(IndexerIOInputs inputs) {
    inputs.velocityRadPerSec = indexerMotor.getVelocity().getValueAsDouble();
    inputs.currentAmps = indexerMotor.getStatorCurrent().getValueAsDouble();
    inputs.appliedVolts = indexerMotor.getSupplyVoltage().getValueAsDouble();
  }

  public void setVoltage(double voltage) {
    indexerMotor.setVoltage(voltage);
  }
}
