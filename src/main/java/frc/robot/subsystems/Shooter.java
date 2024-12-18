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
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;

// TODO: un-comment the led code when this is merged with the rest

public class Shooter extends SubsystemBase {
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

  // // Leds
  // private final LEDs m_led = new LEDs();

  /** Detects notes getting "intaked" */
  private DigitalInput m_IntakeIR = new DigitalInput(Constants.ShInConstants.kIntakeIRsensor);

  /** Detects notes getting shot out */
  private DigitalInput m_ShooterIR = new DigitalInput(Constants.ShInConstants.kShooterIRsensor);

  // Note blocker positions
  private final double kNoteBlockOpen = 100;
  private final double kNoteBlockClosed = 156;

  // bools
  private boolean noteIn = m_IntakeIR.get();
  private boolean noteOut = m_ShooterIR.get();
  private boolean timerStarted = false;

  // enum for different cases will go here
  public enum speedStates {
    INTAKE,
    VOMIT,
    STOP,
    HIGH,
    FAR_HIGH,
    LOW,
    EJECT,
    MAX;
  }

  speedStates m_target = speedStates.STOP;

  // Motor speeds + ramp rate (constants) (adjust if needed)
  private static double kIntake = 0.3;
  private static double kVomit = -0.8;
  private static double kLow = 0.5;
  private static double kHigh = 0.8;
  private static double kFarHigh = 0.95;
  private static double kMax = 1;
  private static double rampRate = 1;

  // Speed variables
  private double currentSpeed = 0;
  private double lastSpeed = currentSpeed;

  /** Note status */
  public enum noteStates {
    IDLE,
    INTAKING,
    HAS_NOTE,
    FIRST_SIDE,
    CENTER_HOLE,
    SECOND_SIDE,
    NOTE_SHOT;
  }

  noteStates m_noteStatus = noteStates.IDLE;

  /** Creates a new ShooterIntake. */
  public Shooter() {
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
    // Ramp rate
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
    closeNoteBlocker();
    // reset enums
    m_target = speedStates.STOP;
    m_noteStatus = noteStates.IDLE;
  }

  @Override
  public void periodic() {
    noteIn = !m_IntakeIR.get();
    SmartDashboard.putNumber("Shooter motors' speed", currentSpeed);
    SmartDashboard.putBoolean("Note in", noteIn);
    SmartDashboard.putBoolean("Note out", noteOut);
    checkNoteStatus();
  }

  // checks if what state the note should have and changes it (will be in periodic)
  public void checkNoteStatus() {
    switch (m_noteStatus) {
      case IDLE:
      case INTAKING:
        if (!m_IntakeIR.get()) {
          m_noteStatus = noteStates.HAS_NOTE;
        }
        break;
      case HAS_NOTE:
        if (!m_ShooterIR.get()) {
          shootTimer();
          m_noteStatus = noteStates.FIRST_SIDE;
        }
        break;
      case FIRST_SIDE:
        if (m_ShooterIR.get()) {
          m_noteStatus = noteStates.CENTER_HOLE;
        }
        break;
      case CENTER_HOLE:
        if (!m_ShooterIR.get()) {
          m_noteStatus = noteStates.SECOND_SIDE;
        }
        break;
      case SECOND_SIDE:
        if (m_ShooterIR.get()) {
          m_noteStatus = noteStates.NOTE_SHOT;
        }
        break;
      default:
        break;
    }
  }

  public Command Intake() {
    return Commands.sequence(
        runOnce(() -> m_noteStatus = noteStates.INTAKING),
        closeNoteBlocker(),
        runOnce(() -> setShInSpeed(speedStates.INTAKE)),
        new WaitUntilCommand(() -> noteIn()),
        stop());
  }

  public Command Vomit() {
    setShInSpeed(speedStates.VOMIT);
    return run(
        () -> {
          setShInSpeed(speedStates.VOMIT);
        });
  }

  public Command Eject() {
    setShInSpeed(speedStates.EJECT);
    return run(
        () -> {
          setShInSpeed(speedStates.EJECT);
        });
  }

  public Command Shoot(speedStates ShootSpd) {
    return Commands.sequence(
        closeNoteBlocker(),
        runOnce(() -> setShInSpeed(ShootSpd)),
        new WaitCommand(1),
        openNoteBlocker(),
        new WaitCommand(0.7),
        stop());
  }

  // timer in case the IR doesnt detect both sides of the note
  public void shootTimer() {
    // time to be adjusted
    if (!timerStarted) {
      timerStarted = true;
      new WaitCommand(1.5);
      if (m_noteStatus == noteStates.FIRST_SIDE || m_noteStatus == noteStates.CENTER_HOLE) {
        m_noteStatus = noteStates.NOTE_SHOT;
      }
      timerStarted = false;
    }
  }

  private boolean noteIn() {
    return !m_IntakeIR.get();
  }

  private boolean noteOut() {
    return m_noteStatus == noteStates.NOTE_SHOT;
  }

  public Command openNoteBlocker() {
    return runOnce(() -> m_noteBlocker.setAngle(kNoteBlockOpen));
  }

  public Command closeNoteBlocker() {
    return runOnce(() -> m_noteBlocker.setAngle(kNoteBlockClosed));
  }

  public Command setTarget(speedStates target) {
    return runOnce(() -> m_target = target);
  }

  // public Command setSpeedFromTarget() {
  //   return runOnce(() -> setShInSpeed(m_target));
  // }

  public void setShInSpeed(speedStates speed) {
    // Change value of currentSpeed with a switch case using the enum
    switch (speed) {
      case STOP:
        currentSpeed = 0;
        break;
      case INTAKE:
        currentSpeed = kIntake;
        break;
      case VOMIT:
        currentSpeed = kVomit;
        break;
      case EJECT:
        currentSpeed = -kVomit;
        break;
      case LOW:
        currentSpeed = kLow;
        break;
      case HIGH:
        currentSpeed = kHigh;
        break;
      case FAR_HIGH:
        currentSpeed = kFarHigh;
        break;
      case MAX:
        currentSpeed = kMax;
        break;
      default:
        currentSpeed = 0;
        break;
    }
    if (currentSpeed != lastSpeed) {
      m_ShInMasterL.set(currentSpeed);
      m_ShInMasterR.set(currentSpeed);
      lastSpeed = currentSpeed;
    }
  }

  public Command stop() {
    return runOnce(
        () -> {
          setShInSpeed(speedStates.STOP);
          closeNoteBlocker();
        });
  }
}
