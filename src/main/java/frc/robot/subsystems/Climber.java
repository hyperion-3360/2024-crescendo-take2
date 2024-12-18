// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climber extends SubsystemBase {
  /** Follows climber_right */
  private CANSparkMax m_climber_left =
      new CANSparkMax(Constants.ClimberConstants.kLeftClimberId, MotorType.kBrushless);

  private CANSparkMax m_climber_right =
      new CANSparkMax(Constants.ClimberConstants.kRightClimberId, MotorType.kBrushless);

  // apparently only 1 encoder is needed (makes sense actually im just dumb)
  private RelativeEncoder m_climber_encoder = m_climber_right.getEncoder();

  private double rampRate = 1;

  /** Climber speed changed by trigger */
  private double climberSpeed = 0.0;

  /** Creates a new Climber. */
  public Climber() {
    m_climber_left.restoreFactoryDefaults();
    m_climber_right.restoreFactoryDefaults();

    m_climber_left.follow(m_climber_right, true);

    m_climber_encoder.setPosition(0.0);

    m_climber_left.setOpenLoopRampRate(rampRate);
    m_climber_right.setOpenLoopRampRate(rampRate);

    m_climber_left.setIdleMode(IdleMode.kBrake);
    m_climber_right.setIdleMode(IdleMode.kBrake);

    m_climber_left.burnFlash();
    m_climber_right.burnFlash();

    // amp limit (2-3A) (boom)
    m_climber_left.setSmartCurrentLimit(15);
    m_climber_right.setSmartCurrentLimit(15);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Climber encoder", m_climber_encoder.getPosition());
    m_climber_right.set(climberSpeed);
  }

  // axis : Right stick Y axis (codrive controller)
  public void setClimbSpeed(double axis) {
    if (axis > 0) {
      climberSpeed = Math.pow(axis, 2);
    } else {
      climberSpeed = -Math.pow(axis, 2);
    }
  }
}
