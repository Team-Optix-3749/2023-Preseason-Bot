// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;


public class TankDrive extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */

  private WPI_TalonFX frontLeft = new WPI_TalonFX(Constants.Drivetrain.frontLeftID);
  private WPI_TalonFX frontRight = new WPI_TalonFX(Constants.Drivetrain.frontRightID);

  private WPI_TalonFX backLeft = new WPI_TalonFX(Constants.Drivetrain.backLeftID);
  private WPI_TalonFX backRight = new WPI_TalonFX(Constants.Drivetrain.backRightID);

  private MotorControllerGroup leftMotorControl = new MotorControllerGroup(backLeft, frontLeft);
  private MotorControllerGroup rightMotorControl = new MotorControllerGroup(backRight, frontRight);

  private final Gyro m_gyro = new ADXRS450_Gyro();

  private DifferentialDrive differentialDrive = new DifferentialDrive(leftMotorControl, rightMotorControl);

  private boolean isBreaked = true;

  public void Drivetrain() {
    rightMotorControl.setInverted(true);

    frontLeft.configSupplyCurrentLimit( new SupplyCurrentLimitConfiguration(true, 40, 40, 2));
    frontRight.configSupplyCurrentLimit( new SupplyCurrentLimitConfiguration(true, 40, 40, 2));
    backRight.configSupplyCurrentLimit( new SupplyCurrentLimitConfiguration(true, 40, 40, 2));

    backLeft.configSupplyCurrentLimit( new SupplyCurrentLimitConfiguration(true, 40, 40, 2));



    frontLeft.setSelectedSensorPosition(0);
    frontRight.setSelectedSensorPosition(0);
    backLeft.setSelectedSensorPosition(0);
    backRight.setSelectedSensorPosition(0);

    frontLeft.setNeutralMode(NeutralMode.Brake);
    frontRight.setNeutralMode(NeutralMode.Brake);
    backLeft.setNeutralMode(NeutralMode.Brake);
    backRight.setNeutralMode(NeutralMode.Brake);

  }

  public void flipBraked() {
    if (!isBreaked) {
      frontLeft.setNeutralMode(NeutralMode.Brake);
      frontRight.setNeutralMode(NeutralMode.Brake);
      backLeft.setNeutralMode(NeutralMode.Brake);
      backRight.setNeutralMode(NeutralMode.Brake);
    } else {
      frontLeft.setNeutralMode(NeutralMode.Coast);
      frontRight.setNeutralMode(NeutralMode.Coast);
      backLeft.setNeutralMode(NeutralMode.Coast);
      backRight.setNeutralMode(NeutralMode.Coast);
    }

    isBreaked = !isBreaked;
  }

  public double getLeftDistance()
  {
    return (frontLeft.getSelectedSensorPosition() + backLeft.getSelectedSensorPosition())/2/Constants.Drivetrain.wheelConversionFactor;
  }

  public double getRightDistance()
  {
    return (frontRight.getSelectedSensorPosition() + frontRight.getSelectedSensorPosition())/2/Constants.Drivetrain.wheelConversionFactor;
  }

  public void arcadeDrive(double speed, double rotation) {
    differentialDrive.arcadeDrive(0.7 * speed, rotation);
    // differentialDrive.arcadeDrive(0.75 * speed * speed * Math.signum(speed), rotation * rotation * Math.signum(rotation));
  }

  public void stop() {
    differentialDrive.arcadeDrive(0, 0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  // public void setDefaultCommand(DrivetrainCommnd drivetrainCommnd) {
  // }
}
