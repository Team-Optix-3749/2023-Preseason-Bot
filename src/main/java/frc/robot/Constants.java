// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class XboxConstants {
    public static final int kDriverControllerPortPilot = 0;
    public static final int kDriverControllerPortOperator = 1;
  }
  public static class Drivetrain{
    public static final int frontLeftID = 1;
    public static final int frontRightID = 3;
  
    public static final int backLeftID = 2;
    public static final int backRightID = 4;

    public static final double speedScalar = 2.0/3.0;
  
  }

}