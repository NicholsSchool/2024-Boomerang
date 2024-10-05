package frc.robot.subsystems.intake;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.Constants;
import java.util.Random;

public class IntakeIOSim implements IntakeIO {
  private FlywheelSim firstSim =
      new FlywheelSim(
          DCMotor.getFalcon500(1), 1 / Constants.IntakeConstants.secondStageGearRatio, 0.004);
  private FlywheelSim secondSim =
      new FlywheelSim(
          DCMotor.getFalcon500(1), 1 / Constants.IntakeConstants.firstStageGearRatio, 0.004);

  private double appliedVolts = 0.0;
  private LinearFilter velocityFilter;
  private boolean isIntaking = false;
  private double minVelocityRadPerSec = 10.0;
  private Timer timer = new Timer();
  private boolean isTimerRunning = false;
  private boolean hasNote = false;
  Random rand = new Random();

  public IntakeIOSim() {
    System.out.println("[Init] Creating IntakeIOSim");
    // Create filter of velocity to determine which direction motor is going.
    // this is used to simulate a note retrieval
    velocityFilter = LinearFilter.singlePoleIIR(0.1, Constants.loopPeriodSecs);
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    // TODO: do fancy pretending for hasNote

    firstSim.setInputVoltage(appliedVolts);
    firstSim.update(Constants.loopPeriodSecs);

    secondSim.setInputVoltage(appliedVolts);
    secondSim.update(Constants.loopPeriodSecs);

    inputs.firstIntakeVelocityRadPerSec = firstSim.getAngularVelocityRadPerSec();
    inputs.secondIntakeVelocityRadPerSec = secondSim.getAngularVelocityRadPerSec();

    inputs.firstIntakeAppliedVolts = appliedVolts;
    inputs.secondIntakeAppliedVolts = appliedVolts;

    inputs.firstIntakeCurrAmps = firstSim.getCurrentDrawAmps();
    inputs.secondIntakeAppliedVolts = secondSim.getCurrentDrawAmps();

    inputs.hasNote = false;

    simulateNote();
  }

  // simulate eating and vomiting a note
  private void simulateNote() {
    double filteredVelocity = velocityFilter.calculate(firstSim.getAngularVelocityRadPerSec());
    if (Math.abs(filteredVelocity) > minVelocityRadPerSec) {
      // assumes intaking is positive velocity)
      boolean intaking = (Math.signum(filteredVelocity) > 0);

      // if changed then start timer for intaking/outtaking
      if (isIntaking != intaking) {
        isIntaking = intaking;
        timer.restart();
        isTimerRunning = true;
      }
      // flip state after timeout time.
      if (isTimerRunning) {
        double timeoutSec = isIntaking ? 2 : 0.25; // up to 2 secs to intake, 0.25 sec to outtake
        if (timer.hasElapsed(timeoutSec)) {
          hasNote = !hasNote;
          timer.stop();
          isTimerRunning = false;
        }
      }
    }
  }

  @Override
  public void setVoltage(double voltage) {
    appliedVolts = voltage;
    firstSim.setInputVoltage(appliedVolts);
    secondSim.setInputVoltage(appliedVolts);
  }

  @Override
  public void setBrakeMode(boolean brake) {}
}
