package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClawSubsystem;

public class IntakeReleaseCommand extends CommandBase{
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final ClawSubsystem clawSubsystem;
    private final double speed;

    public IntakeReleaseCommand(ClawSubsystem subsystem, double speed) {
        this.clawSubsystem = subsystem;
        this.speed = speed;
        addRequirements(clawSubsystem);
    }

    @Override
    public void initialize() {
        clawSubsystem.setClawMotor(speed);
    }

    @Override
    public void execute() {}

    @Override
    public void end(boolean interrupted) {
        clawSubsystem.stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
