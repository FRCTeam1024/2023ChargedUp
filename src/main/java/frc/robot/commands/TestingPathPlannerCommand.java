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

public class TestingPathPlannerCommand extends CommandBase {

  private PathPlannerTrajectory m_Path;
  private SwerveDrive m_Drivetrain;

  public TestingPathPlannerCommand(
      PathPlannerTrajectory traj, SwerveDrive m_drivetrain, boolean isFirstPath) {
    m_Drivetrain = m_drivetrain;

    new SequentialCommandGroup(
        new InstantCommand(
            () -> {
              // Reset odometry for the first path you run during auto
              if (isFirstPath) {
                m_drivetrain.resetPosition(traj.getInitialHolonomicPose());
              }
            }),
        new PPSwerveControllerCommand(
            traj,
            m_drivetrain::getPose, // Pose supplier
            m_drivetrain.getSwerveDriveKinematics(), // SwerveDriveKinematics
            // X controller. Tune these values for your robot. Leaving them 0 will only use
            // feedforwards.
            /* 1.5 */
            new PIDController(7.5, 0, 0),

            // Y controller (usually the same values as X controller)
            /* 1.5 */
            new PIDController(7.5, 0, 0),

            // Rotation controller. Tune these values for your robot. Leaving them 0 will only use
            // feedforwards.
            /* 4 */
            new PIDController(.5, 0, 0.005),
            m_drivetrain::setModuleStates, // Module states consumer
            m_drivetrain // Requires this drive subsystem
            ));
  }
  // spotless:off
  /*
   * Legacy code from 2022 - might not work with swerve, not a major concern right now
  public SequentialCommandGroup configure() {
    return this.beforeStarting(
            () ->
                m_Drivetrain.resetOdometry(
                    m_Drivetrain.getRotation2d(),
                    m_Drivetrain.getSwerveModulePositions(),
                    m_Path.getInitialPose()))
        .andThen(() -> m_Drivetrain.tankDriveVolts(0, 0))
        .andThen(
            new ParallelRaceGroup(
                new TurnToHeading(
                    m_Drivetrain,
                    m_Path
                        .sample(m_Path.getTotalTimeSeconds())
                        .poseMeters
                        .getRotation()
                        .getDegrees()),
                new WaitCommand(0.1))); // cut from 0.25 sec
  }*/
  // spotless:on
}
