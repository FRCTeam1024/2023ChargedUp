// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.oi.Logitech;
import frc.robot.subsystems.EndEffector;

public class TurnWristWithJoysticks extends CommandBase {
  /** Creates a new TurnWristWithJoysticks. */
  EndEffector endEffector;
  Logitech controller;
  public TurnWristWithJoysticks(EndEffector m_EndEffector, Logitech m_Controller) {
    // Use addRequirements() here to declare subsystem dependencies.
    endEffector = m_EndEffector;
    controller = m_Controller;
    addRequirements(endEffector);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double speed = controller.getRightStickY() *1;//0.7
    endEffector.turnWrist(speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
