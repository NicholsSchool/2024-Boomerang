package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
  @AutoLog
  public static class IntakeIOInputs {
    public double firstIntakeVelocityRadPerSec = 0.0;
    public double secondIntakeVelocityRadPerSec = 0.0;
    public boolean hasNote = false;
    public double firstIntakeAppliedVolts = 0.0;
    public double secondIntakeAppliedVolts = 0.0;
    public double firstIntakeCurrAmps = 0.0;
    public double secondIntakeCurrAmps = 0.0;
  }
  /** Updates the set of loggable inputs. */
  public default void updateInputs(IntakeIOInputs inputs) {}
  ;

  /** Set the motor voltage */
  public default void setVoltage(double volts) {}
  ;

  /** Enable or disable brake mode on the motors. */
  public default void setBrakeMode(boolean brake) {}
  ;
}
