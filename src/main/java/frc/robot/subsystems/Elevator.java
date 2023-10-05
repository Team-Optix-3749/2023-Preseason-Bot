// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.io.PipedInputStream;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.controller.PIDController;
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
    motorOne.setInverted(true);
    // gear ratio  * 1 / circumfrance rotarty bar * 1 / total length elevator
    motorOneEncoder.setPositionConversionFactor(1);
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
    setVoltage(voltage);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    runElevator();
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
