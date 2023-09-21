// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveDrive;

public class AutoTurn extends CommandBase {
  private SwerveDrive m_swerve;

  private double goal;
  private int counter;
  private double angle;

  /** Creates a new AutoAlignAprilTag. */
  public AutoTurn(SwerveDrive swerve, double theGoal) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_swerve = swerve;
    goal = theGoal;
    counter = 0;
    angle = m_swerve.getYawDegrees();
    addRequirements(m_swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    counter = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    angle = MathUtil.inputModulus(m_swerve.getYawDegrees(), -180, 180);
    double error = goal - angle;
    if (error < -0.5) {
      counter = 0;
      m_swerve.drive(0, 0, -0.25, true);
    } else if (error > 0.5) {
      counter = 0;
      m_swerve.drive(0, 0, 0.25, true);
    } else {
      counter++;
      m_swerve.defenseMode();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (counter >= 10) {
      return true;
    } else {
      return false;
    }
  }
}
