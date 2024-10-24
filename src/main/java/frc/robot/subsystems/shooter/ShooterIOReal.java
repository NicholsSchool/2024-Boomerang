package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.robot.Constants;
import frc.robot.Constants.ShooterConstants;

public class ShooterIOReal implements ShooterIO {
  private final TalonFX motorBottom;
  private final TalonFX motorTop;

  public ShooterIOReal() {
    System.out.println("[Init] Creating OuttakeIOReal");

    motorTop = new TalonFX(Constants.CAN.kShooterTop);
    motorBottom = new TalonFX(Constants.CAN.kShooterBottom);
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.CurrentLimits.StatorCurrentLimit = ShooterConstants.kCurrentLimit;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    motorBottom.getConfigurator().apply(config);
    motorTop.setControl(new Follower(Constants.CAN.kShooterBottom, false));
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    inputs.velocityRPMs =
        (motorBottom.getVelocity().getValueAsDouble()
                / Constants.ShooterConstants.kSHOOTER_GEAR_RATIO)
            * 60.0;
    inputs.appliedVolts = motorTop.getMotorVoltage().getValueAsDouble();
    inputs.currentAmps = motorTop.getTorqueCurrent().getValueAsDouble();
  }

  @Override
  public void setVoltage(double voltage) {
    motorTop.setVoltage(-voltage);
    motorBottom.setVoltage(-voltage);
  }

  @Override
  public void setBrakeMode(boolean brake) {
    motorTop.setNeutralMode(brake ? NeutralModeValue.Brake : NeutralModeValue.Coast);
  }

  @Override
  public void setDirection(boolean forward) {
    motorTop.setInverted(!forward);
  }

  @Override
  public void stop() {
    motorTop.stopMotor();
  }
}
