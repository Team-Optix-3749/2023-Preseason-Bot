// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.*;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.Drive;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;

/**
//  * This class is where the bulk of the robot should be declared. Since Command-based is a
//  * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
//  * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
//  * subsystems, commands, and trigger mappings) should be declared here.
//  */
// public class RobotContainer {
//    private final tankdrive drivetrain = new tankdrive();


//     private void configureButtonBindings() {
//       XboxController pilot = new XboxController(1); 


//         drivetrain.setDefaultCommand(new tankdrive(drivetrain, pilot::getLeftX, pilot::getRightY));

//         // m_intake.setDefaultCommand(
//         //     new Input(m_intake, Pilot::getLeftTrigger, Pilot::getRightTrigger));
//     }
//     public Command getAutonomousCommand() {
//         return Commands.print("No autonomous command configured");
//   }

//   // Replace with CommandPS4Controller or CommandJoystick if needed
//   private final CommandXboxController m_driverController =
//       new CommandXboxController(OperatorConstants.kDriverControllerPort);

//   /** The container for the robot. Contains subsystems, OI devices, and commands. */
//   public RobotContainer() {
//     // Configure the trigger bindings
//     configureButtonBindings();
//   }
// }
public class RobotContainer {

  private final TankDrive drivetrain = new TankDrive();

  // private final Elevator m_elevator = new Elevator();

  public RobotContainer() {
      configureButtonBindings();
  }

  private void configureButtonBindings() {
    XboxController pilot = new XboxController(1); 
    drivetrain.setDefaultCommand(new TankDrive(drivetrain, pilot::getLeftX, pilot::getRightY));

  }
  public Command getAutonomousCommand() {
      return Commands.print("No autonomous command configured");
}      
}
