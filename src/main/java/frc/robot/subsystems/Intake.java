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
    private final PIDController intakeController = new PIDController(0, 0, 0);
    private final SimpleMotorFeedforward intakeFeedforward = new SimpleMotorFeedforward(0, 0, 0);

    private double intakeMotorVelocity = Constants.Intake.idleVelocity;
    double speed = 0;
    
    public Intake() {
        intakeMotor.restoreFactoryDefaults();

        intakeMotor.setInverted(true); // NOTE: dk if needed

        intakeEncoder.setPositionConversionFactor(1.0 / 5.0); // TODO: check for value on this
        intakeEncoder.setVelocityConversionFactor(1.0 / (60.0 * 5.0)); // TODO: check for value on this

        intakeMotor.setSmartCurrentLimit(40);
    }

    public void setIntakeMotorVoltage(double velocity) {

        double voltage = intakeController.calculate(intakeEncoder.getVelocity(), velocity);
        voltage += intakeFeedforward.calculate(velocity);
        intakeMotor.setVoltage(voltage);

    }

    public void setIntakeVelocity(double velocity) { // get better name
        this.intakeMotorVelocity = velocity;
    }
    public void setIntakeSpeed(double speedincrease){
        speed += speedincrease;
    }

    @Override
    public void periodic(){
        intakeMotor.set(speed);

        setIntakeMotorVoltage(intakeMotorVelocity);
    }
}
