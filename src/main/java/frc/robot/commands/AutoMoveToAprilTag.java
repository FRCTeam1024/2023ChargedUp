// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.Vision;

public class AutoMoveToAprilTag extends CommandBase {
  private SwerveDrive m_swerve;
  private Vision m_camera;

  private Transform3d camToTarget;
  private Pose2d targetPose;

  /** Creates a new AutoMoveToAprilTag. */
  public AutoMoveToAprilTag(SwerveDrive swerve, Vision camera) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_swerve = swerve;
    m_camera = camera;
    //addRequirements(m_swerve, m_camera);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (m_camera.hasTargets()) {  
      Pose2d robotPose = m_swerve.getPose();
      Translation2d robotPosition = new Translation2d(robotPose.getX(), robotPose.getY());
      Rotation2d robotRotation = robotPose.getRotation();
      camToTarget = m_camera.getBestTarget().getBestCameraToTarget();
      System.out.println(camToTarget.toString());
      targetPose = robotPose.plus(
        new Transform2d(
          new Translation2d(camToTarget.getX() + 1, camToTarget.getY()),
          new Rotation2d(camToTarget.getRotation().getZ())
        )
      );
      PathPlannerTrajectory trajectoryToTag = PathPlanner.generatePath(
        new PathConstraints(1, 1),
        new PathPoint(robotPosition, robotRotation),
        new PathPoint(targetPose.getTranslation(), targetPose.getRotation())
      );
      System.out.println(trajectoryToTag.getInitialState().toString());
      System.out.println(trajectoryToTag.getEndState().toString());
      //new PathPlannerCommand(trajectoryToTag, m_swerve, true);
      //m_swerve.followTrajectory(trajectoryToTag);
      new SequentialCommandGroup(
      new PrintCommand("Following Trajectory"),
      new InstantCommand(() -> m_swerve.resetPosition(trajectoryToTag.getInitialHolonomicPose())),
      new ParallelCommandGroup(
        new PPSwerveControllerCommand(
          trajectoryToTag,
          m_swerve::getPose, // Pose supplier
          m_swerve.getSwerveDriveKinematics(), // SwerveDriveKinematics
          new PIDController(7.5, 0, 0), // X controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
          new PIDController(7.5, 0, 0), // Y controller (usually the same values as X controller)
          new PIDController(0.5, 0, 0.005), // Rotation controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
          m_swerve::setModuleStates, // Module states consumer
          m_swerve // Requires this drive subsystem
        )
      ),
      new InstantCommand(() -> m_swerve.defenseMode())
    );
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
