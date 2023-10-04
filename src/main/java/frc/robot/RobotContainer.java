package frc.robot;

import frc.robot.commands.*;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class RobotContainer {

    private final XboxController pilot = new XboxController(Constants.XboxConstants.kDriverControllerPortPilot);

    private final TankDrive drivetrain = new TankDrive();
    private final Drive driveCommand = new Drive(drivetrain, pilot::getLeftY, pilot::getRightX);
     // 0 is the USB Port to be used as indicated on the Driver Station
    // private final Elevator m_elevator = new Elevator();

    public RobotContainer() {
        configureButtonBindings();
    }

    private void configureButtonBindings() {

      drivetrain.setDefaultCommand(driveCommand);

        // m_intake.setDefaultCommand(
        //     new Input(m_intake, Pilot::getLeftTrigger, Pilot::getRightTrigger));
    }
    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
  }      
}
 