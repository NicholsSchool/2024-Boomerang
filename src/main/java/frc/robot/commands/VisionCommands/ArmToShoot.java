package frc.robot.commands.VisionCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.FieldConstants;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.util.GeomUtil;

public class ArmToShoot extends InstantCommand {
  private Arm arm;
  private Drive drive;

  public ArmToShoot(Arm arm, Drive drive) {
    this.arm = arm;
    this.drive = drive;
    hasRequirement(arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  @Override
  public void execute() {
    var currentPose = drive.getPose();
    double distance =
        GeomUtil.distance(
            currentPose,
            new Pose2d(
                new Translation2d(
                    AllianceFlipUtil.apply(FieldConstants.Speaker.centerSpeakerOpening.getX()),
                    FieldConstants.Speaker.centerSpeakerOpening.getY()),
                new Rotation2d()));
    arm.setTargetPos((-0.33 * Math.log(119.0 * (distance - 0.67)) + 2) * 180 / Math.PI);
  }
}
