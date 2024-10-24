package frc.robot.subsystems.shooter;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.Constants;

public class ShooterIOSim implements ShooterIO {
  private FlywheelSim sim =
      new FlywheelSim(
          DCMotor.getFalcon500(1), Constants.ShooterConstants.kSHOOTER_GEAR_RATIO, 0.004);
  private double appliedVolts = 0.0;

  public ShooterIOSim() {
    System.out.println("[Init] Creating OuttakeIOSim");
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {

    sim.setInputVoltage(appliedVolts);
    sim.update(Constants.loopPeriodSecs);

    inputs.velocityRPMs = sim.getAngularVelocityRPM();
    inputs.appliedVolts = appliedVolts;
    inputs.currentAmps = sim.getCurrentDrawAmps();
  }

  @Override
  public void setVoltage(double voltage) {
    appliedVolts = voltage;
    sim.setInputVoltage(appliedVolts);
  }

  @Override
  public void setBrakeMode(boolean brake) {}

  @Override
  public void setDirection(boolean forward) {}

  @Override
  public void stop() {
    appliedVolts = 0;
    sim.setInputVoltage(appliedVolts);
  }
}
