package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.Constants.Setpoints;
import frc.robot.subsystems.*;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class Elevator_Wrist extends CommandBase {
    @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })

    private final Elevator elevator;
    private final WristSubsystem wrist;
    private final Setpoints setpoint;



    public Elevator_Wrist(Elevator elevator, WristSubsystem wrist, Setpoints setpoint) {
        this.elevator = elevator;
        this.wrist = wrist;
        this.setpoint = setpoint;
        addRequirements(elevator,wrist);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {

        if (wrist.getWristAngle()>90){
            wrist.setSetpoint(setpoint);
            
        }
        

    }

    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}