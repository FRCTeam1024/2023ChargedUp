// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.oi.Logitech;
import frc.robot.subsystems.SwerveDrive;

public class DriveWithJoysticks extends CommandBase {
  /** Creates a new DriveWithJoysticks. */
  Logitech controller;
  SwerveDrive drivetrain;
  boolean fieldRelative;
  double speedFactor;
  public DriveWithJoysticks(SwerveDrive m_swerve, Logitech controllerParam, boolean isFieldRelative, double speed) {
    // Use addRequirements() here to declare subsystem dependencies.
    fieldRelative = isFieldRelative;
    drivetrain = m_swerve;
    controller = controllerParam;
    speedFactor = speed;
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double xSpeed = -controller.getRightStickY() * DriveConstants.kMaxWheelSpeedMetersPerSecond * speedFactor;
    double ySpeed = -controller.getRightStickX() * DriveConstants.kMaxWheelSpeedMetersPerSecond * speedFactor;
    double rot = controller.getLeftStickX() * DriveConstants.kMaxAngularSpeedRadiansPerSecond * speedFactor;

    drivetrain.drive(xSpeed, ySpeed, rot, fieldRelative);
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
