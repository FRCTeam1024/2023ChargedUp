// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.SwerveDrive;

public class PathPlannerCommand extends PPSwerveControllerCommand {
  /** Creates a new PathPlannerCommand. */
  private SwerveDrive m_Drivetrain;
  private PathPlannerTrajectory path;
  private boolean first;

  public PathPlannerCommand(PathPlannerTrajectory traj, SwerveDrive m_drivetrain, boolean isFirstPath) {
    super(
      traj, 
      m_drivetrain::getPose, // Pose supplier
      m_drivetrain.getSwerveDriveKinematics(), // SwerveDriveKinematics
      new PIDController(7.5/**1.5*/, 0, 0), // X controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
      new PIDController(7.5/**1.5*/, 0, 0), // Y controller (usually the same values as X controller)
      new PIDController(0.5/**4*/, 0, 0.005), // Rotation controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
      m_drivetrain::setModuleStates, // Module states consumer
      m_drivetrain // Requires this drive subsystem
    );
    path = traj;
    m_Drivetrain = m_drivetrain;
    first = isFirstPath;
  }

  public SequentialCommandGroup configure() {
    return this.beforeStarting(new InstantCommand(()-> {
          if(first) {
            m_Drivetrain.resetPosition(path.getInitialHolonomicPose());
          }
        }));
  }
}
