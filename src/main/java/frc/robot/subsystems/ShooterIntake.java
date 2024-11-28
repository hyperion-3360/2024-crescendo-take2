// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterIntake extends SubsystemBase {
  private CANSparkMax m_ShInMasterL =
      new CANSparkMax(Constants.ShInConstants.kLeftShIn1, MotorType.kBrushless);
  private CANSparkMax m_ShInFollowL =
      new CANSparkMax(Constants.ShInConstants.kLeftShIn2, MotorType.kBrushless);
  private CANSparkMax m_ShInMasterR =
      new CANSparkMax(Constants.ShInConstants.kRightShIn1, MotorType.kBrushless);
  private CANSparkMax m_ShInFollowR =
      new CANSparkMax(Constants.ShInConstants.kRightShIn2, MotorType.kBrushless);

  /** Creates a new ShooterIntake. */
  public ShooterIntake() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
