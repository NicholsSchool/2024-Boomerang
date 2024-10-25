package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.arm.Arm;

public class ArmToPos extends InstantCommand {
  private Arm arm;
  double angleDeg;

  public ArmToPos(double angleDeg, Arm arm) {
    this.arm = arm;
    hasRequirement(arm);
    this.angleDeg = angleDeg;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  @Override
  public void execute() {
    arm.setTargetPos(angleDeg);
  }
}
