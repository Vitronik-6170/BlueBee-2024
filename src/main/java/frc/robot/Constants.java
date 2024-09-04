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

  

  //ID'S CHASSIS
  public static final int MOTORIDL1 = 7;
  public static final int MOTORIDL2 = 22;
  public static final int MOTORIDR1 = 1;
  public static final int MOTORIDR2 = 21;

  //ID'S SHOOTER
  public static final int MOTOR1 = 20;
  public static final int MOTOR2 = 7;

  //ID INTAKE
  public static final int IDINTAKE = 6;

  //ID ELEVADOR
  public static final int IDELEVADOR = 24;

  //ID CONSTANTES
  public static final double d = 0.1524;
  public static final double kPulse = 1800;

  //public static final int kDriverControllerPort = 0;
  //public static final int kMechanismsControllerPort = 1;
  
  
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kMechanismsControllerPort = 1;
  }
}
