package frc.robot;

import frc.robot.commands.*;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class RobotContainer {

    private final TankDrive drivetrain = new TankDrive();
     // 0 is the USB Port to be used as indicated on the Driver Station
    // private final Elevator m_elevator = new Elevator();

    public RobotContainer() {
        configureButtonBindings();
    }

    private void configureButtonBindings() {
      XboxController pilot = new XboxController(Constants.XboxConstants.kDriverControllerPortPilot);

      drivetrain.setDefaultCommand(new Drive(drivetrain, pilot::getLeftY, pilot::getRightX));

        // m_intake.setDefaultCommand(
        //     new Input(m_intake, Pilot::getLeftTrigger, Pilot::getRightTrigger));
    }
    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
  }      
}
 