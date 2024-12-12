// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
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

  // Servo (finger)
  private Servo m_noteBlocker = new Servo(Constants.ShInConstants.kServoPort);

  /** Detects notes getting "intaked" */
  private DigitalInput m_IntakeIR = new DigitalInput(Constants.ShInConstants.kIntakeIRsensor);

  /** Detects notes getting shot out */
  private DigitalInput m_ShooterIR = new DigitalInput(Constants.ShInConstants.kShooterIRsensor);

  // Motor speeds + ramp rate
  private static double kTestIntake = -0.3;
  private static double kTestVomit = 0.8;
  private static double rampRate = 1; // To be adjusted
  private double currentSpeed = 0;

  // Note blocker positions
  private final double kNoteBlockOpen = 100;
  private final double kNoteBlockClosed = 156;

  // IR bools
  private boolean noteIn = m_IntakeIR.get();
  private boolean noteOut = m_ShooterIR.get();

  // enum for different cases will go here
  public enum speedStates {
    INTAKE,
    VOMIT,
    STOP
  }

  /** Creates a new ShooterIntake. */
  public ShooterIntake() {
    // Restore factory default pour pas faire du caca
    m_ShInFollowL.restoreFactoryDefaults();
    m_ShInFollowR.restoreFactoryDefaults();
    m_ShInMasterL.restoreFactoryDefaults();
    m_ShInMasterR.restoreFactoryDefaults();
    // Invert Left side motors
    m_ShInFollowL.setInverted(true);
    m_ShInMasterL.setInverted(true);
    // Set idle mode
    m_ShInMasterL.setIdleMode(IdleMode.kBrake);
    m_ShInFollowL.setIdleMode(IdleMode.kBrake);
    m_ShInMasterR.setIdleMode(IdleMode.kBrake);
    m_ShInFollowR.setIdleMode(IdleMode.kBrake);
    // Make the stuff follow stuff (smart)
    m_ShInFollowL.follow(m_ShInMasterL);
    m_ShInFollowR.follow(m_ShInMasterR);
    // Ramp rate (go look it up i'm too lazy to explain it right now)
    m_ShInFollowL.setOpenLoopRampRate(rampRate);
    m_ShInFollowR.setOpenLoopRampRate(rampRate);
    m_ShInMasterL.setOpenLoopRampRate(rampRate);
    m_ShInMasterR.setOpenLoopRampRate(rampRate);
    // No clue what this does
    m_ShInFollowL.burnFlash();
    m_ShInFollowR.burnFlash();
    m_ShInMasterL.burnFlash();
    m_ShInMasterR.burnFlash();
    // Close note blocker hook
    m_noteBlocker.setAngle(kNoteBlockClosed);
    // Amp limit
    m_ShInMasterL.setSmartCurrentLimit(20);
    m_ShInFollowL.setSmartCurrentLimit(20);
    m_ShInMasterR.setSmartCurrentLimit(20);
    m_ShInFollowR.setSmartCurrentLimit(20);
  }

  @Override
  public void periodic() {
    noteIn = !m_IntakeIR.get();
    noteOut = !m_ShooterIR.get();
    SmartDashboard.putNumber("currentSpeed", currentSpeed);
  }

  // Note: these are test commands, i will make the way it works different
  public Command testIntake() {
    return this.runOnce(() -> setVarSpeed(speedStates.INTAKE));
  }

  public Command testVomit() {
    return this.runOnce(() -> setVarSpeed(speedStates.VOMIT));
  }

  public Command setZeroSpeed() {
    return run(
        () -> {
          m_ShInMasterL.set(0);
          m_ShInMasterR.set(0);
        });
  }

  // More advanced intake command
  public Command advIntake() {
    return Commands.sequence(
        closeNoteBlocker(),
        runOnce(() -> setVarSpeed(speedStates.INTAKE)),
        setMotorSpeed(),
        new WaitUntilCommand(() -> noteIn()),
        stop(),
        setMotorSpeed());
  }

  private boolean noteIn() {
    // Implement the logic for noteIn
    return !m_IntakeIR.get();
  }

  public void openNoteBlocker() {
    m_noteBlocker.setAngle(kNoteBlockOpen);
  }

  public Command closeNoteBlocker() {
    return runOnce(() -> m_noteBlocker.setAngle(kNoteBlockClosed));
  }

  public void setVarSpeed(speedStates speed) {
    // Change value of currentSpeed with a switch case using the enum
    switch (speed) {
      case STOP:
        currentSpeed = 0;
        break;
      case INTAKE:
        currentSpeed = kTestIntake;
        break;
      case VOMIT:
        currentSpeed = kTestVomit;
        break;
      default:
        currentSpeed = 0;
        break;
    }
  }

  // this is complicated for no reason but its simpler to just keep it
  public Command setMotorSpeed() {
    return runOnce(
        () -> {
          m_ShInMasterL.set(currentSpeed);
          m_ShInMasterR.set(currentSpeed);
        });
  }

  public Command stop() {
    return runOnce(() -> setVarSpeed(speedStates.STOP));
  }
}
