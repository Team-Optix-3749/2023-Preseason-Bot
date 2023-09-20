package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.WristSubsystem;

public class ResetWrist extends CommandBase {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final WristSubsystem wristSubsystem;

    public ResetWrist(WristSubsystem subsystem) {
        this.wristSubsystem = subsystem;
        addRequirements(wristSubsystem);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() { // check if this works
        wristSubsystem.resetWristAngle();
    }

    @Override
    public void end(boolean interrupted) {
}

    @Override
    public boolean isFinished() {
        return false;
    }
}
