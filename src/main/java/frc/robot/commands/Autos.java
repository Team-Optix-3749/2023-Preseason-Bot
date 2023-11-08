// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.TankDrive;
import frc.robot.subsystems.WristSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.subsystems.Intake;

public final class Autos {
  /** Example static factory for an autonomous command. */
  private static Command driveBackwardSeconds(TankDrive drivetrain, double seconds) {

    return new Drive(drivetrain, () -> 1, () -> 0).withTimeout(1.5);
  }

  public static Command taxi(TankDrive drivetrain){
    return driveBackwardSeconds(drivetrain, 8);
  }

  public static Command scoreTop(Elevator elevator, WristSubsystem wrist, Intake intake){

    Command goUp = new ParallelCommandGroup(
      Commands.runOnce(() -> elevator.setSetpoint(Constants.Setpoints.TOP_SCORING)),
      Commands.runOnce(() -> wrist.setSetpoint(Constants.Setpoints.TOP_SCORING)));
    Command runIntake = new SequentialCommandGroup(5
      new WaitCommand(4),
      Commands.runOnce(() -> intake.setIntakeVoltage(Constants.Intake.outtakeVoltage-1)), 
      new WaitCommand(2),
      Commands.runOnce(() -> intake.setIntakeVoltage(Constants.Intake.idleVoltage)));

    Command goDown = new ParallelCommandGroup(
        Commands.runOnce(() -> elevator.setSetpoint(Constants.Setpoints.STOW)),
        Commands.runOnce(() -> wrist.setSetpoint(Constants.Setpoints.STOW)));
   
    Command command = new SequentialCommandGroup(Commands.run(() -> wrist.setVoltage(-1)).withTimeout(0.3),goUp, runIntake, goDown); 

    return command;
  }

  public static Command scoreTopTaxi(Elevator elevator, WristSubsystem wrist, Intake intake, TankDrive tankDrive){
    return new SequentialCommandGroup(scoreTop(elevator, wrist, intake),new WaitCommand(2), taxi(tankDrive));
  }
}

/** Example static factory for an autonomous command. */
// public static CommandBase exampleAuto(ExampleSubsystem subsystem) {
// return Commands.sequence(subsystem.exampleMethodCommand(), new
// ExampleCommand(subsystem));
// }

// private Autos() {
// throw new UnsupportedOperationException("This is a utility class!");
// }
// }
