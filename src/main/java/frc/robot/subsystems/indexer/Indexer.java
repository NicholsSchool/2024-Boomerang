package frc.robot.subsystems.indexer;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.IndexerConstants;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.AutoLogOutput;

public class Indexer extends SubsystemBase {
  private IndexerIO io;
  private final IndexerIOInputsAutoLogged inputs = new IndexerIOInputsAutoLogged();

  private static final LoggedTunableNumber kP = new LoggedTunableNumber("kP", 0.1);
  private static final LoggedTunableNumber kI = new LoggedTunableNumber("kI", 0.01);

  private static final LoggedTunableNumber indexVelocity =
      new LoggedTunableNumber("index/indexVelocity");
  private static final LoggedTunableNumber reverseVelocity =
      new LoggedTunableNumber("index/reverseVelocity");

  private PIDController controller = new PIDController(0.0, 0.0, 0.0);
  private final SimpleMotorFeedforward ffModel;

  private static double setpointRPMs = 0.0;
  private static double setpointRadPerSec = 0.0;

  private static enum IndexMode {
    kIndexing,
    kReversing,
    kStopped
  };

  private IndexMode indexMode = IndexMode.kStopped;

  public Indexer(IndexerIO io) {
    this.io = io;

    controller.setPID(kP.get(), kI.get(), 0.0);

    indexVelocity.initDefault(IndexerConstants.kIndexRPM);
    reverseVelocity.initDefault(IndexerConstants.kReverseRPM);

    switch (Constants.getRobot()) {
      case ROBOT_REAL:
        ffModel = new SimpleMotorFeedforward(0.1, 0.12);
        break;
      case ROBOT_SIM:
      default:
        ffModel = new SimpleMotorFeedforward(0.0, 0.00);
        break;
    }
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);

    if (kP.hasChanged(hashCode()) || kI.hasChanged(hashCode())) {
      controller.setPID(kP.get(), kI.get(), 0.0);
      controller.reset();
    }

    if (DriverStation.isDisabled()) {
      io.setVoltage(0.0);
      controller.reset();
      indexMode = IndexMode.kStopped;
    } else {
      switch (indexMode) {
        case kIndexing:
          setpointRPMs = indexVelocity.get();
          break;
        case kReversing:
          setpointRPMs = reverseVelocity.get();
          break;
        case kStopped:
        default:
          setpointRPMs = 0.0;
      }
    }

    setpointRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(setpointRPMs);
    controller.setSetpoint(setpointRadPerSec);
    double voltage = ffModel.calculate(setpointRadPerSec);
    io.setVoltage(voltage);
  }

  public void index() {
    this.indexMode = IndexMode.kIndexing;
  }

  public void reverse() {
    this.indexMode = IndexMode.kReversing;
  }

  public void stop() {
    this.indexMode = IndexMode.kStopped;
  }

  @AutoLogOutput
  public double getSetpointRadians() {
    return setpointRadPerSec;
  }

  @AutoLogOutput
  public double getFF() {
    return ffModel.calculate(setpointRadPerSec);
  }

  @AutoLogOutput
  public IndexMode getState() {
    return indexMode;
  }

  @AutoLogOutput
  public double getSetPointRPMs() {
    return setpointRPMs;
  }

  @AutoLogOutput
  public double getVeloctiyRPMS() {
    return Units.radiansPerSecondToRotationsPerMinute(inputs.velocityRadPerSec);
  }
}
