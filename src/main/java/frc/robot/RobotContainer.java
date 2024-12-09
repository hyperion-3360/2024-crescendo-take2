// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.LEDs;

public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final LEDs m_LEDs = new LEDs();

  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
    SmartDashboard.putData("rainbow", m_LEDs.rainbow());
    SmartDashboard.putData("red", m_LEDs.red());
    SmartDashboard.putData("green", m_LEDs.green());
    SmartDashboard.putData("blue", m_LEDs.blue());
    SmartDashboard.putData("teal", m_LEDs.teal());
    SmartDashboard.putData("yellow", m_LEDs.yellow());
    SmartDashboard.putData("orange", m_LEDs.orange());
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
