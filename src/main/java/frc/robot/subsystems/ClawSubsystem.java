package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClawSubsystem extends SubsystemBase {
    private final CANSparkMax clawMotor = new CANSparkMax(Constants.Claw.claw_motor_id, CANSparkMax.MotorType.kBrushless);
    private final RelativeEncoder clawEncoder = clawMotor.getEncoder();
    
    public ClawSubsystem() {
        clawMotor.restoreFactoryDefaults();

        clawMotor.setInverted(true);

        clawEncoder.setPositionConversionFactor(1.0 / 5.0); // TODO: check for value on this
        clawEncoder.setVelocityConversionFactor(1.0 / (60.0 * 5.0)); // TODO: check for value on this


        clawMotor.setSmartCurrentLimit(40); // TODO: check for optimal value on this
    }

    public void setClawMotor(double speed) {
        clawMotor.set(speed);
    }

    public double getClawMotorSpeed() {
        return clawMotor.get();
    }

    public void stop() {
        clawMotor.stopMotor();
    }
}
