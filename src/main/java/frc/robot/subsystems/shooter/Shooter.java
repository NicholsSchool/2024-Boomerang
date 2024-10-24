package frc.robot.subsystems.shooter;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {
  private double setpoint;
  private ShooterIO io;
  private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();
  public double voltageCommand;

  public static final LoggedTunableNumber shootVelocity =
      new LoggedTunableNumber("Outtake/AmpVelocityRPMs");
  public static final LoggedTunableNumber deliverVelocity =
      new LoggedTunableNumber("Outtake/SpeakerVelocityRPMs");
  public static final LoggedTunableNumber reverseVeloctiy =
      new LoggedTunableNumber("Outtake/TrapVelocityRPMs");
  public static final LoggedTunableNumber spinDurationSec =
      new LoggedTunableNumber("Outtake/SpinDurationSec");
  private static final LoggedTunableNumber kP = new LoggedTunableNumber("Outtake/kP");
  private static final LoggedTunableNumber kD = new LoggedTunableNumber("Outtake/kD");

  private PIDController controller = new PIDController(0.0, 0.0, 0.0);
  private final SimpleMotorFeedforward ffModel;

  private static enum OuttakeMode {
    kStopped,
    kShoot,
    kDeliver,
    kReverse
  };

  private OuttakeMode mode = OuttakeMode.kStopped;

  public Shooter(ShooterIO io) {
    System.out.println("[Init] Creating Outtake");
    this.io = io;
    io.setBrakeMode(false);

    shootVelocity.initDefault(Constants.ShooterConstants.kShooterRPM);
    reverseVeloctiy.initDefault(Constants.ShooterConstants.kReverseRPM);
    deliverVelocity.initDefault(Constants.ShooterConstants.kDeliverRPM);
    spinDurationSec.initDefault(1.5);
    kP.initDefault(Constants.ShooterConstants.kP);
    kD.initDefault(Constants.ShooterConstants.kD);

    // Switch constants based on mode (the physics simulator is treated as a
    // separate robot with different tuning)
    switch (Constants.getRobot()) {
      case ROBOT_REAL:
        ffModel = new SimpleMotorFeedforward(0.001, 0.0060);
        break;
      case ROBOT_SIM:
      default:
        ffModel = new SimpleMotorFeedforward(0.0, 0.0055);
        break;
    }
  }

  @Override
  public void periodic() {
    setpoint = 0.0;
    io.updateInputs(inputs);
    Logger.processInputs("Outtake", inputs);

    // Update tunable numbers
    if (kP.hasChanged(hashCode()) || kD.hasChanged(hashCode())) {
      controller.setP(kP.get());
      controller.setD(kD.get());
    }

    // Reset when disabled
    if (DriverStation.isDisabled()) {
      io.setVoltage(0.0);
      controller.reset();
      mode = OuttakeMode.kStopped;
    } else {
      switch (mode) {
        case kShoot:
          setpoint = shootVelocity.get();
          break;
        case kDeliver:
          setpoint = deliverVelocity.get();
          break;
        case kReverse:
          setpoint = reverseVeloctiy.get();
          break;
        case kStopped:
        default:
          setpoint = 0.0;
      }

      voltageCommand = ffModel.calculate(setpoint);
      io.setVoltage(MathUtil.clamp(voltageCommand, -12.0, 12.0));
    }
  }

  public void setDeliver() {
    mode = OuttakeMode.kDeliver;
  }

  public void setShoot() {
    mode = OuttakeMode.kShoot;
  }

  public void setReverse() {
    mode = OuttakeMode.kReverse;
  }

  public void stop() {
    mode = OuttakeMode.kStopped;
  }

  public void setVoltage(double voltage) {
    io.setVoltage(voltage);
  }

  @AutoLogOutput
  public double getSetpointVelocityRPMs() {
    return setpoint;
  }

  @AutoLogOutput
  public double getVoltageCommand() {
    return voltageCommand;
  }

  @AutoLogOutput
  public double getActualVelocityRPMs() {
    return inputs.velocityRPMs;
  }
}
