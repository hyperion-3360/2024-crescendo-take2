// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/** Add your docs here. */
public final class Constants {

  public final class OperatorConstants {

    public static int kDriverControllerPort = 0;

    public static int kCodriverControllerPort = 1;
  }

  public final class ShInConstants {
    /** Device Id for right master shooter/intake motor */
    public static int kRightShIn1 = 15;

    /** Device Id for right follower shooter/intake motor */
    public static int kRightShIn2 = 16;

    /** Device Id for left master shooter/intake motor */
    public static int kLeftShIn1 = 13;

    /** Device Id for left follower shooter/intake motor */
    public static int kLeftShIn2 = 14;

    /** DIO port for lower intake IR sensor */
    public static int kIntakeIRsensor = 4;

    /** DIO port for higher shooter IR sensor */
    public static int kShooterIRsensor = 5;

    /** Port for note blocker servo (i think) */
    public static int kServoPort = 5;
  }
}
