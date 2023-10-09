package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class WristSubsystem extends SubsystemBase {
    private final CANSparkMax wristMotor = new CANSparkMax(Constants.Wrist.wristMotor, CANSparkMax.MotorType.kBrushless);
    private final RelativeEncoder wristEncoder = wristMotor.getEncoder();
    private final PIDController wristController = new PIDController(0, 0,0 );
    private final double ks = 0;

    private Constants.Setpoints currentSetpoint = Constants.Setpoints.STOW;
    
    public WristSubsystem() {
        wristMotor.restoreFactoryDefaults();
        wristMotor.setSmartCurrentLimit(40);
    }

    // angle is rotation in radians
    public void setWristMotorVoltage() {

        double voltage = wristController.calculate(getWristAngle(), currentSetpoint.wristAngle);
        voltage += ks * Math.cos(getWristAngle());
        wristMotor.set(voltage);

    }

    public void setSetpoint(Constants.Setpoints setpoint) {
        currentSetpoint = setpoint;
    }

    public double getWristAngle() {
        return (wristEncoder.getPosition() * 1/15 * 45/56 * 15/32* 360);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Encoder Value",getWristAngle());
        // setWristMotorVoltage();
    }
}
