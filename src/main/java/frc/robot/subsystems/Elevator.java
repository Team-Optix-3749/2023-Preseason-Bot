// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.io.PipedInputStream;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Elevator extends SubsystemBase {

  
  /** Creates a new ExampleSubsystem. */
  


  private final CANSparkMax motorOne = new CANSparkMax(Constants.ElevatorConstants.elevatorMotorOne, MotorType.kBrushless);
  private final CANSparkMax motorTwo = new CANSparkMax(Constants.ElevatorConstants.elevatorMotorTwo, MotorType.kBrushless);
  private final RelativeEncoder motorOneEncoder = motorOne.getEncoder(); // TODO: check if absolute encoder??
  private final RelativeEncoder motorTwoEncoder = motorTwo.getEncoder(); // TODO: check if absolute encoder??

  private final PIDController elevatorController = new PIDController(1,0,0);

  private double setpointPercentage = 0; 


  public Elevator() 
  {
    motorTwo.setInverted(true);
    // gear ratio  * 1 / circumfrance rotarty bar * 1 / total length elevator
    // motorOneEncoder.setPositionConversionFactor(1/225);
    motorTwoEncoder.setPositionConversionFactor(1/225);

    motorOne.setSmartCurrentLimit(60);
    motorTwo.setSmartCurrentLimit(60);


    motorOne.setIdleMode(IdleMode.kCoast);
    motorTwo.setIdleMode(IdleMode.kCoast);

  }
  /**
   * Example command factory method.
   *
   * @return a command
   */
  public CommandBase exampleMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }



  public void setElevatorPercent(double percent){
    setpointPercentage = percent;
  }

  public void stop()
  {
    motorOne.set(0);
    motorTwo.set(0); 
  }

  public void setVoltage(double volts)
  {
    if (getElevatorPositionInches() < 0.15 && volts <0){
      volts = 0;
    }
    else if (getElevatorPositionInches() > 42 && volts > 0){
      volts = 0;
    }
    SmartDashboard.putNumber("Elevator Voltage", volts);
    motorOne.setVoltage(volts);
    motorTwo.setVoltage(volts);;
  }

  public double getEncoderValue()
  {
    return motorOneEncoder.getPosition();
  }

  public void runElevator(){

    double voltage = 0;
    voltage = elevatorController.calculate(getEncoderValue(), setpointPercentage);
    // setVoltage(voltage);


  }

  // Average of the two encoders * 1/58 (the raw reading at max extension) * 42.5 (the max extenstion in inches)
  public double getElevatorPositionInches(){
    return ((motorOneEncoder.getPosition()+motorTwoEncoder.getPosition())/2 * 1/58 *42.5);
  }

  @Override
  public void periodic() {
    

    SmartDashboard.putNumber("Motor 5 Bus Voltage", motorOne.getBusVoltage());
    SmartDashboard.putNumber("Motor 6 Bus Voltage", motorTwo.getBusVoltage());
    SmartDashboard.putNumber("Motor 5 Current", motorOne.getOutputCurrent());
    SmartDashboard.putNumber("Motor 6 Current", motorTwo.getOutputCurrent());

    SmartDashboard.putNumber("Elevator Position", getElevatorPositionInches());

    // SmartDashboard.putNumber("Motor 5 Current", motorOne.getBusVoltage()*motorOne.getAppliedOutput());
    // SmartDashboard.putNumber("Motor 6 Current", motorTwo.getBusVoltage()*motorTwo.getAppliedOutput());

    // SmartDashboard.putNumber("Motor5 Current", motorOne.);

    // This method will be called once per scheduler run

    // runElevator();
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
