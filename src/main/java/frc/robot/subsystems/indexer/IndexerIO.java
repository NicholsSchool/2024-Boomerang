package frc.robot.subsystems.indexer;

import org.littletonrobotics.junction.AutoLog;

public interface IndexerIO {

  @AutoLog
  public static class IndexerIOInputs {
    public double velocityRadPerSec = 0.0;
    public boolean hasNote = false;
    public double appliedVolts = 0.0;
    public double currentAmps = 0.0;
  }

  public default void updateInputs(IndexerIOInputs inputs) {}

  public default void setVoltage(double voltage) {}
}
