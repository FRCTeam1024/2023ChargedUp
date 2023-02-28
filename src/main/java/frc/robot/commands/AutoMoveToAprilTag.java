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
import edu.wpi.first.wpilibj2.command.Command;
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
  private PathPlannerTrajectory trajectoryToTag;
  private Pose2d robotPose;
  private Translation2d robotPosition;
  private Rotation2d robotRotation;

  /** Creates a new AutoMoveToAprilTag. */
  public AutoMoveToAprilTag(SwerveDrive swerve, Vision camera) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_swerve = swerve;
    m_camera = camera;
    robotPose = m_swerve.getPose();
    robotPosition = new Translation2d(robotPose.getX(), robotPose.getY());
    robotRotation = robotPose.getRotation();
    if (m_camera.hasTargets()) {
      System.out.println(m_camera.hasTargets());  
      camToTarget = m_camera.getBestTarget().getBestCameraToTarget();
      System.out.println(camToTarget.toString());
      targetPose = robotPose.plus(
        new Transform2d(
          new Translation2d(-camToTarget.getX() + 0.5, -camToTarget.getY()),
          new Rotation2d(camToTarget.getRotation().getZ())
        )
      );
      trajectoryToTag = PathPlanner.generatePath(
        new PathConstraints(1, 1),
        new PathPoint(robotPosition, robotRotation),
        new PathPoint(targetPose.getTranslation(), targetPose.getRotation())
      );
      System.out.println(trajectoryToTag.getInitialState().toString());
      System.out.println(trajectoryToTag.getEndState().toString());
    }else{
      trajectoryToTag = PathPlanner.generatePath(
        new PathConstraints(1, 1),
        new PathPoint(robotPosition, robotRotation),
        new PathPoint(new Translation2d(robotPosition.getX() + 0.1, robotPosition.getY()), robotRotation)
      );
    }
    //addRequirements(m_swerve, m_camera);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    /**if (m_camera.hasTargets()) {  
      robotPose = m_swerve.getPose();
      robotPosition = new Translation2d(robotPose.getX(), robotPose.getY());
      robotRotation = robotPose.getRotation();
      camToTarget = m_camera.getBestTarget().getBestCameraToTarget();
      System.out.println(camToTarget.toString());
      targetPose = robotPose.plus(
        new Transform2d(
          new Translation2d(-camToTarget.getX() + 1, -camToTarget.getY()),
          new Rotation2d(camToTarget.getRotation().getZ())
        )
      );
      trajectoryToTag = PathPlanner.generatePath(
        new PathConstraints(1, 1),
        new PathPoint(robotPosition, robotRotation),
        new PathPoint(targetPose.getTranslation(), targetPose.getRotation())
      );
      System.out.println(trajectoryToTag.getInitialState().toString());
      System.out.println(trajectoryToTag.getEndState().toString());
    }*/
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

  public PathPlannerTrajectory update(){
    Pose2d currentPose = m_swerve.getPose();
    robotPosition = new Translation2d(currentPose.getX(), currentPose.getY());
    robotRotation = currentPose.getRotation();
    if (m_camera.hasTargets()) {  
      camToTarget = m_camera.getBestTarget().getBestCameraToTarget();
      targetPose = currentPose.plus(
        new Transform2d(
          new Translation2d(-camToTarget.getX() + 0.5, -camToTarget.getY()),
          new Rotation2d(camToTarget.getRotation().getZ() + Math.PI)
        )
      );
      System.out.println("\n\n" + camToTarget.toString() + "\n\n" + targetPose.toString() + "\n\n");
      trajectoryToTag = PathPlanner.generatePath(
        new PathConstraints(0.5, 0.5),
        new PathPoint(robotPosition, robotRotation),
        new PathPoint(targetPose.getTranslation(), targetPose.getRotation())
      );
      System.out.println(trajectoryToTag.getInitialState().toString());
      System.out.println(trajectoryToTag.getEndState().toString());
    }else{
      trajectoryToTag = PathPlanner.generatePath(
        new PathConstraints(0.5, 0.5),
        new PathPoint(robotPosition, robotRotation),
        new PathPoint(new Translation2d(robotPosition.getX() + 0.1, robotPosition.getY()), robotRotation)
      );
    }
    return trajectoryToTag;
  }

  public SequentialCommandGroup move(){
    return new PathPlannerCommand(update(), m_swerve, true).configure();
  }
}
