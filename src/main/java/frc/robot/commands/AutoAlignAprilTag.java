// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.Vision;
import org.photonvision.targeting.PhotonTrackedTarget;

public class AutoAlignAprilTag extends CommandBase {
  private SwerveDrive m_swerve;

  private Vision m_camera;
  private int counter;

  /** Creates a new AutoAlignAprilTag. */
  public AutoAlignAprilTag(SwerveDrive swerve, Vision camera) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_swerve = swerve;
    m_camera = camera;
    counter = 0;
    addRequirements(m_swerve, m_camera);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    counter = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    PhotonTrackedTarget target;
    double angle;
    if (m_camera.hasTargets()) {
      target = m_camera.getBestTarget();
      angle = target.getYaw();
    } else {
      angle = 0;
    }
    if (angle < -0.5) {
      counter = 0;
      m_swerve.drive(0, 0, 0.1, true);
    } else if (angle > 0.5) {
      counter = 0;
      m_swerve.drive(0, 0, -0.1, true);
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
