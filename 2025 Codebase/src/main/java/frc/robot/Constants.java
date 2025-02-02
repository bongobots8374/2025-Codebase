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
  public static class OperatorConstants {
    public static final int DriverControllerPort = 0;

    public static final double LeftXDeadband = 0.3;
    public static final double LeftYDeadband = 0.3;
    public static final double RightXDeadband = 0.3;

    public static final int LeftXAxis = 0;
    public static final int LeftYAxis = 1;
    public static final int RightXAxis = 2;
  }
  public static class SwerveConstants {
    public static final double MaxSpeed = 50;
  }

  public static class LimeLights {
    public static final String three = "limelight-three";
    public static final String four = "limelight-four";
  }
}
