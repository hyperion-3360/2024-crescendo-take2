// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/** Add your docs here. */
public final class Constants {
  public final class ClimberConstants {
    /** Device Id for left climber motor */
    public static int kLeftClimberId = 11;

    /** Device Id for right climber */
    public static int kRightClimberId = 12;
  }

  public final class OperatorConstants {
    public static int kDriverControllerPort = 0;
  }

  public final class ShInConstants {
    /** Device Id for front right shooter/intake motor */
    public static int kRightShIn1 = 13;

    /** Device Id for back right shooter/intake motor */
    public static int kRightShIn2 = 14;

    /** Device Id for front left shooter/intake motor */
    public static int kLeftShIn1 = 15;

    /** Device Id for back left shooter/intake motor */
    public static int kLeftShIn2 = 16;

    /** DIO port for lower intake IR sensor */
    public static int kIntakeIRsensor = 4;

    /** DIO port for higher shooter IR sensor */
    public static int kShooterIRsensor = 5;

    /** Port for note blocker servo (i think) */
    public static int kServoPort = 5;
  }
}
