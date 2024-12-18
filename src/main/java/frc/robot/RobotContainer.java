// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.Pathfinding;
import frc.robot.subsystems.LEDs;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.Climb;
import frc.robot.commands.SpeakerLock;
import frc.robot.commands.TeleopSwerve;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.swerve.CTREConfigs;
import frc.robot.subsystems.swerve.Swerve;
import java.util.function.DoubleSupplier;
import org.photonvision.PhotonCamera;

public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final LEDs m_LEDs = new LEDs();

  public RobotContainer() {
    m_climber.setDefaultCommand(new Climb(m_climber, climbAxis));

    m_swerveDrive.resetModulesToAbsolute();

    m_swerveDrive.setDefaultCommand(
        new TeleopSwerve(
            m_swerveDrive,
            () -> conditionJoystick(translationAxis, translationLimiter, kJoystickDeadband),
            () -> conditionJoystick(strafeAxis, strafeLimiter, kJoystickDeadband),
            () -> conditionJoystick(rotationAxis, rotationLimiter, kJoystickDeadband),
            () -> true));

    m_camera = new PhotonCamera("vision2");
    m_speakerLockCmd = new SpeakerLock(null, null, m_camera);
    configureBindings();
    configureSmartDashboardCommands();
  }

  private void configureBindings() {
    SmartDashboard.putData("rainbow", m_LEDs.rainbow());
    SmartDashboard.putData("red", m_LEDs.red());
    SmartDashboard.putData("green", m_LEDs.green());
    SmartDashboard.putData("blue", m_LEDs.blue());
    SmartDashboard.putData("lavender", m_LEDs.lavender());
    SmartDashboard.putData("yellow", m_LEDs.yellow());
    SmartDashboard.putData("orange", m_LEDs.orange());
    SmartDashboard.putNumber("colorFinderRed", 0);
    SmartDashboard.putNumber("colorFinderGreen", 0);
    SmartDashboard.putNumber("colorFinderBlue", 0);
  }

  private void configureSmartDashboardCommands() {
    SmartDashboard.putString("Camera", m_camera.toString());
    SmartDashboard.putData(m_speakerLockCmd);
  }

  public Command getAutonomousCommand() {
    PathConstraints constraints =
        new PathConstraints(3.0, 4.0, Units.degreesToRadians(540), Units.degreesToRadians(720));

    m_swerveDrive.resetOdometryBlueSide();
    m_swerveDrive.setPose(new Pose2d(0, 0, new Rotation2d(-180)));
    // return AutoBuilder.followPath(PathPlannerPath.fromPathFile("0b"));
    //m_LEDs.rainbow();
    return Pathfinding.doPathfinding();
  }
}
