// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/** Add your docs here. */
public final class Constants {
  public static final double stickDeadband = 0.1;

  public static class VisionConstants {
    public static long kPositionCoalescingTime = 100 * 1000; // 100 ms in microseconds
    public static int kRedSpeakerTag = 4;
    public static int kBlueSpeakerTag = 7;
    public static Integer kSpeakerIndex[] = {kRedSpeakerTag, kBlueSpeakerTag};
    public static int kRedAmpTag = 5;
    public static int kBlueAmpTag = 6;
    public static Integer kAmpIndex[] = {kRedAmpTag, kBlueAmpTag};
    public static double kAmpRiseElevatorDistance = 0.5;
    public static double kAmpShootingDistance = 0.1;
    public static double kShooterCameraPitch = 0.1309; // 7.5deg;
    public static double kCameraHeight = 0.432; // 43.2 cm
  }
}
