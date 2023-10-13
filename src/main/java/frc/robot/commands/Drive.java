package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.*;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class Drive extends CommandBase {
    @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })

    private final TankDrive drive;
    private DoubleSupplier y;
    private DoubleSupplier x;

    public Drive(TankDrive drivetrain, DoubleSupplier leftY, DoubleSupplier rightX) {
        drive = drivetrain;
        y = leftY;
        x = rightX;
        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        SmartDashboard.putNumber("Y", y.getAsDouble());
        SmartDashboard.putNumber("X", x.getAsDouble());

        drive.arcadeDrive( -x.getAsDouble()*Constants.Drivetrain.speedScalar, -y.getAsDouble() * Constants.Drivetrain.speedScalar);
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