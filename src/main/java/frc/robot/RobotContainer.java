package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.RobotConstants;
import frc.robot.Constants.RobotType;
import frc.robot.Constants.ShooterConstants;
import frc.robot.commands.AutoCommands;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.FeedForwardCharacterization;
import frc.robot.commands.VisionCommands.ArmToShoot;
import frc.robot.commands.VoltageCommandRamp;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmIOReal;
import frc.robot.subsystems.arm.ArmIOSim;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIONAVX;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.indexer.IndexerIOReal;
import frc.robot.subsystems.indexer.IndexerIOSim;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIOReal;
import frc.robot.subsystems.intake.IntakeIOSim;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterIOReal;
import frc.robot.subsystems.shooter.ShooterIOSim;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private final Drive drive;
  private final Intake intake;
  private final Shooter shooter;
  private final Arm arm;
  private final Indexer indexer;
  private PowerDistribution pdh;

  // shuffleboard
  ShuffleboardTab boomerangTab;
  public static GenericEntry hasNote;

  // Controller
  public static CommandXboxController driveController = new CommandXboxController(0);
  public static CommandXboxController operatorController = new CommandXboxController(1);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  // Start position selections
  public static final LoggedTunableNumber startPositionIndex =
      new LoggedTunableNumber("start pos index: 0 or 1", 0.0);
  // Start Pos 0: Along line of the Amp.
  public static final LoggedTunableNumber startX0 =
      new LoggedTunableNumber("Start X0(m)", Units.inchesToMeters(0));
  public static final LoggedTunableNumber startY0 = new LoggedTunableNumber("Start Y0(m)", 4.05);
  public static final LoggedTunableNumber startTheta0 =
      new LoggedTunableNumber("Start Theta0(deg)", 0.0);
  // Start Pos 1: Next to human player side of Speaker.
  public static final LoggedTunableNumber startX1 =
      new LoggedTunableNumber(
          "Start X1(m)", Units.inchesToMeters(RobotConstants.robotSideLengthInches / 2));
  public static final LoggedTunableNumber startY1 = new LoggedTunableNumber("Start Y1(m)", 4.05);
  public static final LoggedTunableNumber startTheta1 =
      new LoggedTunableNumber("Start Theta1(deg)", 0.0);

  // Auto Commands
  private final AutoCommands autoCommands;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (Constants.getRobot()) {
      case ROBOT_REAL:
        // Real robot, instantiate hardware IO implementations
        pdh = new PowerDistribution(Constants.CAN.kPowerDistributionHub, ModuleType.kRev);
        drive =
            new Drive(
                new GyroIONAVX(),
                new ModuleIOTalonFX(0),
                new ModuleIOTalonFX(1),
                new ModuleIOTalonFX(2),
                new ModuleIOTalonFX(3));
        intake = new Intake(new IntakeIOReal());
        shooter = new Shooter(new ShooterIOReal());
        arm = new Arm(new ArmIOReal());
        indexer = new Indexer(new IndexerIOReal());
        break;

      case ROBOT_SIM:
        // Sim robot, instantiate physics sim IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim());
        intake = new Intake(new IntakeIOSim());
        shooter = new Shooter(new ShooterIOSim());
        arm = new Arm(new ArmIOSim());
        indexer = new Indexer(new IndexerIOSim());
        break;

      case ROBOT_FOOTBALL:
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim());
        intake = new Intake(new IntakeIOSim());
        shooter = new Shooter(new ShooterIOSim());
        arm = new Arm(new ArmIOSim());
        indexer = new Indexer(new IndexerIOSim());

        break;

      default:
        // case ROBOT_REPLAY:
        // Replayed robot, disable IO implementations since the replay
        // will supply the data.

        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});
        intake = new Intake(new IntakeIOSim());
        shooter = new Shooter(new ShooterIOSim());
        arm = new Arm(new ArmIOSim());
        indexer = new Indexer(new IndexerIOSim());

        break;
    }

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // Create auto commands
    autoCommands = new AutoCommands(drive);

    autoChooser.addOption("Wait 5 seconds", new WaitCommand(5.0));

    // add testing auto functions
    addTestingAutos();

    // initialize the shuffleboard outputs
    initShuffleboard();

    // Configure the button bindings
    configureButtonBindings();

    // set starting position of robot
    // setStartingPose();
  }

  private void initShuffleboard() {
    // Configure the Shuffleboard
    boomerangTab = Shuffleboard.getTab("Boomerang");
    // this is where display booleans will go
  }

  public void updateShuffleboard() {

    if (Constants.getRobot() == RobotType.ROBOT_REAL) {
      SmartDashboard.putNumber("PDH/Voltage", pdh.getVoltage());
      SmartDashboard.putNumber("PDH/Current", pdh.getTotalCurrent());
      SmartDashboard.putNumber("PDH/Power", pdh.getTotalPower());
      SmartDashboard.putNumber("PDH/Energy", pdh.getTotalEnergy());

      int numChannels = pdh.getNumChannels();
      for (int i = 0; i < numChannels; i++) {
        SmartDashboard.putNumber("PDH/Channel " + i, pdh.getCurrent(i));
      }
    }

    // resetPosWithDashboard();
  }

  // changes robot pose with dashboard tunables
  private void resetPosWithDashboard() {

    // update robot position only if robot is disabled, otherwise
    // robot could move in unexpected ways.
    if (DriverStation.isDisabled()) {
      if (startX0.hasChanged(hashCode())
          || startY0.hasChanged(hashCode())
          || startTheta0.hasChanged(hashCode())
          || startX1.hasChanged(hashCode())
          || startY1.hasChanged(hashCode())
          || startTheta1.hasChanged(hashCode())
          || startPositionIndex.hasChanged(hashCode())) {

        setStartingPose();
      }
    }
  }

  /**
   * Set the starting pose of the robot based on position index. This should be called only when
   * robot is disabled.
   */
  public void setStartingPose() {
    // Set starting position only if operating robot in field-relative control.
    // Otherwise, robot starts at 0, 0, 0.
    if (!Constants.driveRobotRelative) {
      Pose2d startPosition0 =
          new Pose2d(
              startX0.get(), startY0.get(), new Rotation2d(Math.toRadians(startTheta0.get())));
      Pose2d startPosition1 =
          new Pose2d(
              startX1.get(), startY1.get(), new Rotation2d(Math.toRadians(startTheta1.get())));

      drive.setPose(
          startPositionIndex.get() == 0
              ? AllianceFlipUtil.apply(startPosition0)
              : AllianceFlipUtil.apply(startPosition1));
    }
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -driveController.getLeftY() * Constants.DriveConstants.lowGearScaler,
            () -> -driveController.getLeftX() * Constants.DriveConstants.lowGearScaler,
            () -> -driveController.getRightX() * 0.55,
            () -> Constants.driveRobotRelative));
    driveController.start().onTrue(new InstantCommand(() -> drive.resetFieldHeading()));
    driveController
        .leftTrigger(0.8)
        .whileTrue(
            DriveCommands.joystickDrive(
                drive,
                () -> -driveController.getLeftY(),
                () -> -driveController.getLeftX(),
                () -> -driveController.getRightX(),
                () -> Constants.driveRobotRelative));

    driveController
        .a()
        .whileTrue(
            DriveCommands.joystickDriveWithAngle(
                drive,
                () -> -driveController.getLeftY() * Constants.DriveConstants.lowGearScaler,
                () -> -driveController.getLeftX() * Constants.DriveConstants.lowGearScaler,
                () -> 180,
                () -> drive.getYaw(),
                () -> Constants.driveRobotRelative));
    driveController
        .y()
        .whileTrue(
            DriveCommands.joystickDriveWithAngle(
                drive,
                () -> -driveController.getLeftY() * Constants.DriveConstants.lowGearScaler,
                () -> -driveController.getLeftX() * Constants.DriveConstants.lowGearScaler,
                () -> 0,
                () -> drive.getYaw(),
                () -> Constants.driveRobotRelative));
    driveController
        .x()
        .whileTrue(
            DriveCommands.joystickDriveWithAngle(
                drive,
                () -> -driveController.getLeftY() * Constants.DriveConstants.lowGearScaler,
                () -> -driveController.getLeftX() * Constants.DriveConstants.lowGearScaler,
                () -> 90,
                () -> drive.getYaw(),
                () -> Constants.driveRobotRelative));
    driveController
        .b()
        .whileTrue(
            DriveCommands.joystickDriveWithAngle(
                drive,
                () -> -driveController.getLeftY() * Constants.DriveConstants.lowGearScaler,
                () -> -driveController.getLeftX() * Constants.DriveConstants.lowGearScaler,
                () -> -90,
                () -> drive.getYaw(),
                () -> Constants.driveRobotRelative));
    driveController
        .rightStick()
        .toggleOnTrue(
            DriveCommands.joystickDriveFacingPoint(
                drive,
                () -> -driveController.getLeftY() * Constants.DriveConstants.lowGearScaler,
                () -> -driveController.getLeftX() * Constants.DriveConstants.lowGearScaler,
                new Translation2d(
                    FieldConstants.Speaker.centerSpeakerOpening.toTranslation2d().getX() - 0.5,
                    FieldConstants.Speaker.centerSpeakerOpening.toTranslation2d().getY()),
                () -> drive.getYaw()));
    driveController.rightStick().toggleOnTrue((new ArmToShoot(arm, drive)));
    // .repeatedly()
    // .until(driveController.leftTrigger())
    // .andThen(arm.runGoToPosCommand(40.0)));

    intake.setDefaultCommand(new InstantCommand(() -> intake.stop(), intake));
    shooter.setDefaultCommand(new InstantCommand(() -> shooter.stop(), shooter));
    indexer.setDefaultCommand(new InstantCommand(() -> indexer.stop(), indexer));
    // TODO: tune idle arm angle

    driveController.rightTrigger(0.9).whileTrue(intake.runEatCommand());
    driveController.rightBumper().whileTrue(intake.runVomitCommand());

    operatorController
        .a()
        .whileTrue(
            new ParallelCommandGroup(
                new SequentialCommandGroup(
                    new WaitCommand(ShooterConstants.shootRampUpTimeSecs),
                    new ParallelCommandGroup(
                        intake.runDigestCommand(),
                        new InstantCommand(() -> indexer.index(), indexer))),
                new InstantCommand(() -> shooter.setShoot(), shooter)));

    driveController.leftBumper().whileTrue(arm.runGoToPosCommand(55.0));
    driveController.leftBumper().whileFalse(arm.runGoToPosCommand(30.0));
    // driveController.rightTrigger(0.9).whileTrue(intake.runPoopCommand());
  }

  // /**
  //  * @return the command to run in autonomous
  //  */
  public Command getAutonomousCommand() {
    //   // var path = PathPlannerAuto.getStaringPoseFromAutoFile("TestAuto");
    //   // var path = PathPlannerPath.fromChoreoTrajectory("New Path");
    //   // NamedCommands.registerCommand("RunIntake", intake.runEatCommand().withTimeout(1.0));
    //   // NamedCommands.registerCommand("StopIntake", new InstantCommand(() -> intake.stop(),
    // intake));
    //   // NamedCommands.registerCommand(
    //   //     "Shoot", new InstantCommand(() -> new Shoot(shooter, intake,
    // indexer)).withTimeout(3));
    //   // drive.setPose(PathPlannerAuto.getStaringPoseFromAutoFile("New Auto"));
    //   // return new PathPlannerAuto("New Auto");
    //   // return AutoBuilder.followPath(PathPlannerPath.fromPathFile("TestPath"));
    return autoChooser.get();
  }

  private void addAutos() {}

  private void addTestingAutos() {
    // Pathplanner Auto Testing
    // Set up feedforward characterization
    autoChooser.addOption(
        "Drive FF Characterization",
        new FeedForwardCharacterization(
            drive,
            drive::runCharacterizationVolts,
            drive::getCharacterizationVelocity)); // todo change these for new robot

    autoChooser.addOption(
        "Module Drive Ramp Test",
        new VoltageCommandRamp(drive, drive::runDriveCommandRampVolts, 0.5, 5.0));

    autoChooser.addOption(
        "Module Turn Ramp Test",
        new VoltageCommandRamp(drive, drive::runTurnCommandRampVolts, 0.5, 5.0));

    autoChooser.addOption(
        "Spline Test",
        autoCommands.splineToPose(
            new Pose2d(
                new Translation2d(4, 3),
                new Rotation2d(Math.PI / 2)))); // TODO: change these for new robot

    autoChooser.addOption( // drives 10 ft for odometry testing
        "10 foot test", autoCommands.TenFootTest(drive)); // TODO: change these for new robot
  }
}
