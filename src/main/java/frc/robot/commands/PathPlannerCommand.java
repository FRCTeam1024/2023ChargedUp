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
  SwerveDrive m_Drivetrain;
  PathPlannerTrajectory path;
  boolean first;
  public PathPlannerCommand(PathPlannerTrajectory traj, SwerveDrive m_drivetrain, boolean isFirstPath) {
    // Use addRequirements() here to declare subsystem dependencies.
    super(
      traj, 
      m_drivetrain::getPose, // Pose supplier
      m_drivetrain.getSwerveDriveKinematics(), // SwerveDriveKinematics
      new PIDController(1/**1.5*/, 0, 0), // X controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
      new PIDController(1/**1.5*/, 0, 0), // Y controller (usually the same values as X controller)
      new PIDController(1/**4*/, 0, 0.005), // Rotation controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
      m_drivetrain::setModuleStates, // Module states consumer
      m_drivetrain // Requires this drive subsystem
    );
    path = traj;
    m_Drivetrain = m_drivetrain;
    first = isFirstPath;
    addRequirements(m_Drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if(first){
      m_Drivetrain.resetPosition(path.getInitialHolonomicPose());
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  /**@Override
  public boolean isFinished() {
    return false;
  }*/
}
