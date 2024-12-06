// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.ShooterIntake;

public class RobotContainer {
  private final CommandXboxController m_driverController =
      new CommandXboxController(Constants.OperatorConstants.kDriverControllerPort);
  private final ShooterIntake m_shooter = new ShooterIntake();

  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
    m_driverController.a().whileTrue(m_shooter.testIntake());
    // temporary fix
    m_driverController.a().onFalse(m_shooter.setZeroSpeed());
    m_driverController.b().whileTrue(m_shooter.testVomit());
    // temporary fix
    m_driverController.b().onFalse(m_shooter.setZeroSpeed());
    // please work
    m_driverController.x().onTrue(m_shooter.advIntake());
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
