// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.io.PipedInputStream;
import java.util.function.DoubleSupplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Elevator extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */

  private final CANSparkMax motorOne = new CANSparkMax(Constants.Elevator.elevatorMotorOneID,
      MotorType.kBrushless);
  private final CANSparkMax motorTwo = new CANSparkMax(Constants.Elevator.elevatorMotorTwoID,
      MotorType.kBrushless);
  private final RelativeEncoder motorOneEncoder = motorOne.getEncoder();
  private final RelativeEncoder motorTwoEncoder = motorTwo.getEncoder();

  private final PIDController elevatorController = new PIDController(0.3, 0, 0);

  private final DoubleSupplier wristAngleSupplier;
  // feed forward constants
  private final double ks = 0.3;
  private final double kg = 0.25;
  // extension
  private final double ke = 0.01;
  // wrist rotation
  private final double kr = 0;



  private Constants.Setpoints currentSetpoint = Constants.Setpoints.STOW;

  public Elevator(DoubleSupplier wristAngleSupplier) {

    this.wristAngleSupplier = wristAngleSupplier;

    motorTwo.setInverted(true);
    // gear ratio * 1 / circumfrance rotarty bar * 1 / total length elevator
    // motorOneEncoder.setPositionConversionFactor(1/225);
    motorTwoEncoder.setPositionConversionFactor(1 / 225);

    motorOne.setSmartCurrentLimit(60);
    motorTwo.setSmartCurrentLimit(60);

    motorOne.setIdleMode(IdleMode.kCoast);
    motorTwo.setIdleMode(IdleMode.kCoast);

  }

  public void stop() {
    motorOne.set(0);
    motorTwo.set(0);
  }

  public void setVoltage(double volts) {

    SmartDashboard.putNumber("Elevator Voltage", volts);
    motorOne.setVoltage(volts);
    motorTwo.setVoltage(volts);
    ;
  }

  public void runElevator() {
    double voltage = 0;
    if (Math.abs(currentSetpoint.eleveatorExtension - getElevatorPositionInches()) > 0.25) {
      // voltage = elevatorController.calculate(getElevatorPositionInches(),
      // currentSetpoint.eleveatorExtension);
    }

    if (getElevatorPositionInches() < 0.15 && voltage < 0) {
      voltage = 0;
    } else if (getElevatorPositionInches() > 42 && voltage > 0) {
      voltage = 0;
    }

    // voltage += elevatorFeedForward.calculate(1);
    SmartDashboard.putNumber("Elevator Voltage", voltage);
    setVoltage(voltage);

  }

  // Average of the two encoders * 1/58 (the raw reading at max extension) * 42.5
  // (the max extenstion in inches)
  public double getElevatorPositionInches() {
    return ((motorOneEncoder.getPosition() + motorTwoEncoder.getPosition()) / 2 * 1 / 58 * 42.5);
  }

  public void setSetpoint(Constants.Setpoints setpoint) {
    currentSetpoint = setpoint;
  }

  public void ffcalculate(double velocity){
    double total = 0;
    total += ks * Math.signum(velocity);
    total += kg;
    total += ke * getElevatorPositionInches();
    total += kr * Math.cos(wristAngleSupplier.getAsDouble());
    
  }

  @Override
  public void periodic() {
    runElevator();

    SmartDashboard.putNumber("Motor 5 Bus Voltage", motorOne.getBusVoltage());
    SmartDashboard.putNumber("Motor 6 Bus Voltage", motorTwo.getBusVoltage());
    SmartDashboard.putNumber("Motor 5 Current", motorOne.getOutputCurrent());
    SmartDashboard.putNumber("Motor 6 Current", motorTwo.getOutputCurrent());

    SmartDashboard.putNumber("Elevator Position", getElevatorPositionInches());
    SmartDashboard.putString("Elevator Setpoint", currentSetpoint.name());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
