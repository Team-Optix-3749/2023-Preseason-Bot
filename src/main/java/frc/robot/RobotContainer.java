package frc.robot;

import frc.robot.Constants;
import frc.robot.commands.Autos;
import frc.robot.commands.Drive;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.WristSubsystem;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.TankDrive;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  private final CommandXboxController pilot = new CommandXboxController(0);
  private final CommandXboxController operator = new CommandXboxController(1);

  private final WristSubsystem wristSubsystem = new WristSubsystem();
  private final Intake intake = new Intake();
  private final Elevator elevator = new Elevator(() -> wristSubsystem.getWristAngle());
  private final TankDrive drivetrain = new TankDrive();
  private final Drive driveCommand = new Drive(drivetrain, pilot::getLeftY, pilot::getRightX);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    wristSubsystem.setElevatorPositionSupplier(() -> elevator.getElevatorPositionInches());
    // Configure the trigger bindings
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
   * an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link
   * CommandXboxController
   * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or
   * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // TODO: set appropriate angles for wrist movement
    pilot.rightTrigger().onTrue(Commands.run(() -> intake.setIntakeVelocity(Constants.Intake.intakeVelocity)));
    pilot.leftTrigger().onTrue(Commands.run(() -> intake.setIntakeVelocity(Constants.Intake.outtakeVelocity)));
    pilot.leftTrigger().onFalse(Commands.runOnce(() -> intake.setIntakeVelocity(Constants.Intake.idleVelocity)));
    pilot.rightTrigger().onFalse(Commands.runOnce(() -> intake.setIntakeVelocity(Constants.Intake.idleVelocity)));

    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`

    pilot.leftBumper().whileTrue(Commands.run(() -> wristSubsystem.moveWrist(1), wristSubsystem));
    pilot.rightBumper().whileTrue(Commands.run(() -> wristSubsystem.moveWrist(-1), wristSubsystem));

    // pilot.a().onTrue(Commands.runOnce(() -> wristSubsystem.setSetpoint(Constants.Setpoints.STOW), wristSubsystem));
    // pilot.b().onTrue(Commands.runOnce(() -> wristSubsystem.setSetpoint(Constants.Setpoints.MID_SCORING),
    //     wristSubsystem));
    // pilot.x().onTrue(Commands.runOnce(() -> wristSubsystem.setSetpoint(Constants.Setpoints.TOP_SCORING),
    //     wristSubsystem));

    pilot.a().onTrue(new ParallelCommandGroup(
        Commands.runOnce(() -> elevator.setSetpoint(Constants.Setpoints.STOW)),
        Commands.runOnce(() -> wristSubsystem.setSetpoint(Constants.Setpoints.STOW))));

    pilot.b().onTrue(new ParallelCommandGroup(
        Commands.runOnce(() -> elevator.setSetpoint(Constants.Setpoints.MID_SCORING)),
        Commands.runOnce(() -> wristSubsystem.setSetpoint(Constants.Setpoints.MID_SCORING))));

    pilot.y().onTrue(new ParallelCommandGroup(
        Commands.runOnce(() -> elevator.setSetpoint(Constants.Setpoints.TOP_SCORING)),
        Commands.runOnce(() -> wristSubsystem.setSetpoint(Constants.Setpoints.TOP_SCORING))));

    pilot.y().onTrue(new SequentialCommandGroup(
          Commands.runOnce(() -> elevator.setSetpoint(Constants.Setpoints.GROUND_INTAKE)),
          Commands.runOnce(() -> wristSubsystem.setSetpoint(Constants.Setpoints.GROUND_INTAKE))));
  
    // Schedule `exampleMethodCommand` when the Xbox controller's B button is
    // pressed,
    // cancelling on release.

    drivetrain.setDefaultCommand(driveCommand);

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return (new PrintCommand("Test"));
  }
}
