package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

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
    private final PIDController wristController = new PIDController(0.10, 0, 0); // 0.1
    private final double kg = 0.55;
    private final double ks = 0.1;

    private Constants.Setpoints currentSetpoint = Constants.Setpoints.STOW;

    private Constants.Setpoints setpointOnHold = Constants.Setpoints.STOW;

    private DoubleSupplier elevatorPositionSupplier;

    public WristSubsystem() {
        wristMotor.restoreFactoryDefaults();
        wristMotor.setSmartCurrentLimit(40);
        wristMotor.setInverted(true);
    }

    public void setElevatorPositionSupplier(DoubleSupplier supplier) {
        elevatorPositionSupplier = supplier;
    }

    // angle is rotation in radians
    public void setWristMotorVoltage() {
        double voltage = 0;

        voltage = -wristController.calculate(getWristAngle(),
                currentSetpoint.wristAngle);
        if (voltage > 0) {
            voltage *= 4;
        }
        SmartDashboard.putNumber("Wrist PID Voltage", voltage);

        if (getWristAngle() > 10.5) {
            voltage += ks * Math.signum(voltage);
            voltage += kg * Math.abs(Math.sin(getWristAngle() * Math.PI / 180));
        }

        else if (voltage > 0) {
            voltage = 0;
        }

        if (voltage > 2) {
            voltage = 2;
        }
        if (voltage < -2) {
            voltage = -2;
        }
        SmartDashboard.putNumber("Wrist Voltage", voltage);

        wristMotor.setVoltage(voltage);

    }

    public void setSetpoint(Constants.Setpoints setpoint) {
        setpointOnHold = setpoint;
        double wristRoom = elevatorPositionSupplier.getAsDouble() * Math.sin(1.0 / 3.0 * Math.PI) + 4
                + Math.cos(setpointOnHold.wristAngle / 180.0 * Math.PI) * 19.5;
        if (wristRoom > 0) {
            currentSetpoint = setpoint;

        }
        SmartDashboard.putBoolean("in setSetpoint",
                wristRoom > 0);
    }

    public double getWristAngle() {
        return -(-10 + wristEncoder.getPosition() * 1 / 15 * 45 / 56 * 15 / 32 * 360);
    }

    public void moveWrist(double volts) {

        if (getWristAngle() > 10.5) {
            volts += ks;
            volts += kg * Math.abs(Math.sin(getWristAngle() * Math.PI / 180.0));
        }

        else if (volts > 0) {
            volts = 0;
        }
        wristMotor.setVoltage(volts);
    }

    @Override
    public void periodic() {
        double wristRoom = elevatorPositionSupplier.getAsDouble() * Math.sin(1.0 / 3.0 * Math.PI) + 4
                + Math.cos(setpointOnHold.wristAngle / 180.0 * Math.PI) * 19.5;

        if (wristRoom > 0) {
            currentSetpoint = setpointOnHold;
        }
        SmartDashboard.putBoolean("in periodic",
                wristRoom > 0);
        SmartDashboard.putNumber("Wrist Angle", getWristAngle());
        SmartDashboard.putString("WristSetpoint", currentSetpoint.name());
        SmartDashboard.putString("WristSetpointOnHold", setpointOnHold.name());
        SmartDashboard.putNumber("Wrist Room", wristRoom);
        SmartDashboard.putNumber("elevator hieght",
                elevatorPositionSupplier.getAsDouble() * Math.sin(1.0 / 3.0 * Math.PI) + 4);

        SmartDashboard.putNumber("wrist hieght", Math.cos(setpointOnHold.wristAngle / 180.0 * Math.PI) * 19.5);
        SmartDashboard.putNumber("elevator value from wrist", elevatorPositionSupplier.getAsDouble());
        setWristMotorVoltage();
    }
}
