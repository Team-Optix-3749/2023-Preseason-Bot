package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class WristSubsystem extends SubsystemBase {

    private final CANSparkMax wristMotor = new CANSparkMax(Constants.Wrist.wristMotor,
            CANSparkMax.MotorType.kBrushless);
    private final RelativeEncoder wristEncoder = wristMotor.getEncoder();
    private final PIDController wristController = new PIDController(0, 0, 0);
    private final double kg = 0.15;
    private final double ks = 0.35;

    private Constants.Setpoints currentSetpoint = Constants.Setpoints.STOW;

    public WristSubsystem() {
        wristMotor.restoreFactoryDefaults();
        wristMotor.setSmartCurrentLimit(40);
        wristMotor.setInverted(true);
    }

    // angle is rotation in radians
    public void setWristMotorVoltage() {
        double voltage = 0;

        // voltage = wristController.calculate(getWristAngle(),
        //     currentSetpoint.wristAngle);
        if (getWristAngle() > 11.5) {
            voltage += ks;
            voltage += kg * Math.sin(getWristAngle());
        }

        
        SmartDashboard.putNumber("Wrist Voltage", voltage);
        wristMotor.setVoltage(voltage);

    }

    public void setSetpoint(Constants.Setpoints setpoint) {
        currentSetpoint = setpoint;
    }

    public double getWristAngle() {
        return -(-10 + wristEncoder.getPosition() * 1 / 15 * 45 / 56 * 15 / 32 * 360);
    }

    public void moveWrist(double volts) {
        wristMotor.setVoltage(volts);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Encoder Value", getWristAngle());
        setWristMotorVoltage();
    }
}
