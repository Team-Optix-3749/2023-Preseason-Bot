// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static class Wrist
  {
    public static final int wristMotor = 7;
  }

  public static class Elevator{
    public static final int elevatorMotorOneID = 5;
    public static final int elevatorMotorTwoID = 6;

  }

  public static enum Setpoints {
    // Setpoint(ElevatorExtension, WristAngle)

    STOW(1, 0),
    MID_SCORING(32, 120),
    TOP_SCORING(42, 90);

    public double eleveatorExtension;
    public double wristAngle;

    Setpoints(double elevatorExtension, double wristAngle) {
      this.eleveatorExtension = elevatorExtension;
      this.wristAngle = wristAngle;
    }
  }

  public static class Claw {
    public static final int claw_motor_id = 8;
  }


  public static class Intake {
    public static final int intakeMotor = 8;
    public static final int intakeVelocity = 1;
    public static final int outtakeVelocity = -1;
    public static final int idleVelocity = 0;
  }

  public static class Drivetrain{
    public static final int frontLeftID = 1;
    public static final int frontRightID = 3;
  
    public static final int backLeftID = 2;
    public static final int backRightID = 4;

    public static final double speedScalar = 2.0/3.0;

    public static final double wheelRadius = .0444; //meters
  
    public static final double wheelConversionFactor = 2048 * 9.29 / (2*Math.PI*wheelRadius);
  }

}
