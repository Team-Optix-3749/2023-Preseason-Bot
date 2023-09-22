package frc.robot.commands;

import java.util.function.DoubleSupplier;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.TankDrive;

/** An example command that uses an example subsystem. */
public class Drive extends CommandBase {

  private final TankDrive drive ;
  private DoubleSupplier y;
  private DoubleSupplier x;
  TankDrive differentialDrive;
  private boolean speedToggle = false;

  public void Tankdrive (TankDrive drivetrain) {
    differentialDrive = drivetrain;
    addRequirements(drivetrain);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    drive.arcadeDrive(y.getAsDouble()/1.5, x.getAsDouble()/1.5);
  }


  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}