package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;

public class ShootCommand extends Command {
  public ShootCommand(Intake intake, Shooter shooter) {
    new ParallelCommandGroup(
        new SequentialCommandGroup(
            new WaitCommand(ShooterConstants.shootRampUpTimeSecs), intake.runDigestCommand()),
        new InstantCommand(() -> shooter.setShoot(), shooter));
  }
}
