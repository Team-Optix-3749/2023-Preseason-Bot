package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
    private final CANSparkMax intakeMotor = new CANSparkMax(Constants.Claw.claw_motor_id, CANSparkMax.MotorType.kBrushless);
    private final RelativeEncoder intakeEncoder = intakeMotor.getEncoder();
    private PIDController intakeController = new PIDController(1, 0,0 );
    
    public Intake() {
        intakeMotor.restoreFactoryDefaults();

        intakeMotor.setInverted(true); // NOTE: dk if needed

        intakeEncoder.setPositionConversionFactor(1.0 / 5.0); // TODO: check for value on this
        intakeEncoder.setVelocityConversionFactor(1.0 / (60.0 * 5.0)); // TODO: check for value on this


        intakeMotor.setSmartCurrentLimit(40);
    }

    // speed is rot/s
    public void setIntakeMotor(double speed) {

        double voltage = intakeController.calculate(intakeEncoder.getVelocity(), speed);
        intakeMotor.set(voltage);
    }

    public double getClawMotorSpeed() {
        return intakeMotor.get();
    }

    public void stop() {
        intakeMotor.stopMotor();
    }
    
    @Override
    public void periodic(){

    }
}
