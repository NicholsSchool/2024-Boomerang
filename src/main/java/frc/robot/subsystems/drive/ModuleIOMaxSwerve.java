// Interface for a MAXSwerve Module:
//    Configured with two SPARKS MAX,
//    a NEO as the driving motor (build-in hall effect encoder),
//    a NEO 550 as the turning motor (azimuth),
//    and a REV Through Bore Encoder as the absolute turning encoder.
//

package frc.robot.subsystems.drive;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;

public class ModuleIOMaxSwerve implements ModuleIO {

  private final CANSparkMax drivingSparkMax;
  private final CANSparkMax turningSparkMax;

  private final RelativeEncoder drivingEncoder;
  private final AbsoluteEncoder turningEncoder;

  // The MAXSwerve module can be configured with one of three pinion gears: 12T, 13T, or 14T.
  // This changes the drive speed of the module (a pinion gear with more teeth will result in a
  // robot that drives faster).
  private static final int kDrivingMotorPinionTeeth = 12;
  // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15 teeth on the
  // bevel pinion
  private static final double kDrivingGearRatio = (45.0 * 22) / (kDrivingMotorPinionTeeth * 15);
  private static final double kTurningGearRatio = 1.0;

  private Rotation2d
      turnAngleOffset; // Angular offset of the module relative to the chassis in radians

  /**
   * Constructs a ModuleIOMaxSwerve and configures the driving and turning motor, encoder, and PID
   * controller. This configuration is specific to the REV MAXSwerve Module built with NEOs, SPARKS
   * MAX, and a Through Bore Encoder.
   */
  public ModuleIOMaxSwerve(int index) {

    if (index > 3 || index < 0) {
      throw new RuntimeException("Invalid module index");
    }

    // ModuleIO flModuleIO,
    // ModuleIO frModuleIO,
    // ModuleIO blModuleIO,
    // ModuleIO brModuleIO
    int[] driveIds = {
      0, 1, 2, 3 //TODO: DO NOT USE
    };
    int[] turnIds = {
      0, 1, 2, 3 //DO NOT USE
    }; 
    // Angular offsets of the modules relative to the chassis in radians
    double[] turnOffsets = {(-Math.PI / 2), 0, Math.PI, (Math.PI / 2)};

    drivingSparkMax = new CANSparkMax(driveIds[index], MotorType.kBrushless);
    turningSparkMax = new CANSparkMax(turnIds[index], MotorType.kBrushless);
    turnAngleOffset = new Rotation2d(turnOffsets[index]);

    // Factory reset, set the SPARKS MAX(s) to a known state before configuration.
    // This is useful in case a SPARK MAX is swapped out.
    drivingSparkMax.restoreFactoryDefaults();
    turningSparkMax.restoreFactoryDefaults();

    // Setup driving encoder (hall sensor on brushless motor connected to front port of spark max)
    drivingEncoder = drivingSparkMax.getEncoder();
    // Setup turning encoder (connected absolute encoder)
    turningEncoder = turningSparkMax.getAbsoluteEncoder(Type.kDutyCycle);

    // Invert the turning encoder, since the output shaft rotates in the opposite
    // direction of the steering motor in the REV MAXSwerve Module.
    turningEncoder.setInverted(true);

    // Set the idle mode (brake or coast).
    drivingSparkMax.setIdleMode(Constants.ModuleConstants.kDrivingMotorIdleMode);
    turningSparkMax.setIdleMode(Constants.ModuleConstants.kTurningMotorIdleMode);

    // Current Limiting
    drivingSparkMax.setSmartCurrentLimit(Constants.ModuleConstants.kDrivingMotorCurrentLimit);
    turningSparkMax.setSmartCurrentLimit(Constants.ModuleConstants.kTurningMotorCurrentLimit);

    // Save the SPARK MAX configurations.
    // Note: If a SPARK MAX browns out it will maintain these configurations.
    drivingSparkMax.burnFlash();
    turningSparkMax.burnFlash();

    drivingEncoder.setPosition(0);
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    inputs.drivePositionRad =
        Units.rotationsToRadians(drivingEncoder.getPosition()) / kDrivingGearRatio;
    inputs.driveVelocityRadPerSec =
        Units.rotationsPerMinuteToRadiansPerSecond(drivingEncoder.getVelocity())
            / kDrivingGearRatio;
    inputs.driveAppliedVolts = drivingSparkMax.getAppliedOutput() * drivingSparkMax.getBusVoltage();
    inputs.driveCurrentAmps = new double[] {drivingSparkMax.getOutputCurrent()};

    // turnAbsolutePosition used for voltage offset on relative encoder, but set both absolute and
    // position to same
    // for the turning absolute encoder.
    inputs.turnAbsolutePosition =
        Rotation2d.fromRotations(turningEncoder.getPosition() / kTurningGearRatio)
            .minus(turnAngleOffset);
    inputs.turnPosition = inputs.turnAbsolutePosition;
    inputs.turnVelocityRadPerSec =
        Units.rotationsPerMinuteToRadiansPerSecond(turningEncoder.getVelocity())
            / kTurningGearRatio;
    inputs.turnAppliedVolts = turningSparkMax.getAppliedOutput() * turningSparkMax.getBusVoltage();
    inputs.turnCurrentAmps = new double[] {turningSparkMax.getOutputCurrent()};
  }

  @Override
  public void setDriveVoltage(double volts) {
    drivingSparkMax.setVoltage(volts);
  }

  @Override
  public void setTurnVoltage(double volts) {
    turningSparkMax.setVoltage(volts);
  }

  @Override
  public void setDriveBrakeMode(boolean enable) {
    drivingSparkMax.setIdleMode(enable ? IdleMode.kBrake : IdleMode.kCoast);
  }

  @Override
  public void setTurnBrakeMode(boolean enable) {
    turningSparkMax.setIdleMode(enable ? IdleMode.kBrake : IdleMode.kCoast);
  }
}
