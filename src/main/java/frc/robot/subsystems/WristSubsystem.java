package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class WristSubsystem extends SubsystemBase {
    private final CANSparkMax wristMotor = new CANSparkMax(Constants.Claw.claw_motor_id, CANSparkMax.MotorType.kBrushless);
    private final RelativeEncoder wristEncoder = wristMotor.getEncoder(); // TODO: check if absolute encoder??
    private PIDController wristController = new PIDController(1, 0,0 );

    private double desiredWristAngle = 0; // something to set wrist angle
    private double speed = 0;
    
    public WristSubsystem() {
        wristMotor.restoreFactoryDefaults();

        wristMotor.setInverted(true); // NOTE: dk if needed

        wristEncoder.setPositionConversionFactor(1.0 / 15.0); // TODO: check for value on this
        wristEncoder.setVelocityConversionFactor(1.0 / (60.0 * 15.0)); // TODO: check for value on this


        wristMotor.setSmartCurrentLimit(40); // TODO: check for optimal value on this

        // Schedule the periodic command to run continuously
        // NOTE: Idk if this is the right way to do periodic so hopefully it is
        // CommandScheduler.getInstance().registerSubsystem(this);
        // CommandScheduler.getInstance().schedule(new WristPeriodicCommand(this));
    }

    // angle is rotation in radians
    public void setWristMotor() {

        double voltage = wristController.calculate(wristEncoder.getPosition(), desiredWristAngle);
        voltage = speed;
        wristMotor.set(voltage);
    }

    public double getWristMotorSpeed() {
        return wristMotor.get();
    }

    public double getWristAngle() {
        return (wristEncoder.getPosition() * 360); // TODO: find out how to get value for this
    }

    public void adjustWristAngle(double angle) {
        double newAngle = desiredWristAngle + angle;
        if (newAngle >= 90){ // TODO: find limit here
            desiredWristAngle = 90;
        } else if (newAngle <= -90) {
            desiredWristAngle = -90;
        } else {
            desiredWristAngle = newAngle;
        }
    }

    public double resetWristAngle() {
        return desiredWristAngle = 0;
    }

    public void stop() {
        wristMotor.stopMotor();
    }

    public void setSpeed(double speed){
        this.speed = speed;
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Encoder Value", wristEncoder.getPosition());
        setWristMotor();
    }
}
