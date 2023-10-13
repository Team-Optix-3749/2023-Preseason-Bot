package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
    private final CANSparkMax intakeMotor = new CANSparkMax(Constants.Intake.intakeMotor, CANSparkMax.MotorType.kBrushless);
    private final RelativeEncoder intakeEncoder = intakeMotor.getEncoder();

    private double intakeMotorVoltage = Constants.Intake.idleVoltage;
    double speed = 0;
    
    public Intake() {
        intakeMotor.restoreFactoryDefaults();

        intakeMotor.setInverted(true); // NOTE: dk if needed

        intakeEncoder.setPositionConversionFactor(1.0 / 5.0); // TODO: check for value on this
        intakeEncoder.setVelocityConversionFactor(1.0 / (60.0 * 5.0)); // TODO: check for value on this

        intakeMotor.setSmartCurrentLimit(40);
    }


    public void setIntakeVelocity(double velocity) { // get better name
        this.intakeMotorVoltage = velocity;
    }
    public void setIntakeVoltage(double volts){
        intakeMotorVoltage = volts;
    }

    @Override
    public void periodic(){
        intakeMotor.setVoltage(intakeMotorVoltage);
        // setIntakeMotorVoltage(intakeMotorVelocity);
    }
}
