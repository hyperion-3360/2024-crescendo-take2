// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Climber;
import java.util.function.DoubleSupplier;

public class Climb extends Command {

  private Climber s_climber;
  private DoubleSupplier climbSup;

  public Climb(Climber s_climber, DoubleSupplier climbSup) {
    this.s_climber = s_climber;
    this.climbSup = climbSup;
    addRequirements(s_climber);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double climbVal = MathUtil.applyDeadband(climbSup.getAsDouble(), Constants.kDeadband);

    // climb
    s_climber.setClimbSpeed(climbVal);
  }
}
