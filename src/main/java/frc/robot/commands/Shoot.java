package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;

public class Shoot extends InstantCommand {
  private Shooter shooter;
  private Intake intake;
  private Indexer indexer;

  public Shoot(Shooter shooter, Intake intake, Indexer indexer) {
    this.shooter = shooter;
    this.intake = intake;
    this.indexer = indexer;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  @Override
  public void execute() {
    new ParallelCommandGroup(
        new SequentialCommandGroup(
            new WaitCommand(ShooterConstants.shootRampUpTimeSecs),
            new ParallelCommandGroup(
                intake.runDigestCommand(), new InstantCommand(() -> indexer.index(), indexer))),
        new InstantCommand(() -> shooter.setShoot(), shooter));
  }
}
