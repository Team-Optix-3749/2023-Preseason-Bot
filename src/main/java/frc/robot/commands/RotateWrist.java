package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.WristSubsystem;

public class RotateWrist extends CommandBase{
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final WristSubsystem wristSubsystem;
    private final double angle;

    public RotateWrist(WristSubsystem subsystem, double angle) {
        this.wristSubsystem = subsystem;
        this.angle = angle;
        addRequirements(wristSubsystem);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() { // check if this works
        wristSubsystem.adjustWristAngle(angle);
        wristSubsystem.setWristMotor();
    }

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
        return false;
    }
}
