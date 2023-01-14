// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.SwerveDrive;

public class PathPlannerCommand extends CommandBase {

    private PathPlannerTrajectory m_Path;
    private SwerveDrive m_Drivetrain;

    public PathPlannerCommand(PathPlannerTrajectory traj, SwerveDrive m_drivetrain, boolean isFirstPath) {
        m_Drivetrain = m_drivetrain;
        
        new SequentialCommandGroup(
             new InstantCommand(() -> {
               // Reset odometry for the first path you run during auto
               if(isFirstPath){
                    m_Drivetrain.resetPosition(traj.getInitialHolonomicPose());
               }
             }),
             new PPSwerveControllerCommand(
                 traj, 
                 m_Drivetrain::getPose, // Pose supplier
                 m_Drivetrain.getSwerveDriveKinematics(), // SwerveDriveKinematics
                 new PIDController(1.5, 0, 0), // X controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
                 new PIDController(1.5, 0, 0), // Y controller (usually the same values as X controller)
                 new PIDController(4, 0, 0.005), // Rotation controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
                 m_Drivetrain::setModuleStates, // Module states consumer
                 m_Drivetrain // Requires this drive subsystem
             )
         );
     }

    /**
     * Legacy code from 2022 - might not workwith swerve, not a major concern right now
     * public SequentialCommandGroup configure() {
        return this.beforeStarting(() -> m_Drivetrain.resetOdometry(m_Drivetrain.getRotation2d(), 
        m_Drivetrain.getSwerveModulePositions(),m_Path.getInitialPose()))
                    .andThen(() -> m_Drivetrain.tankDriveVolts(0, 0))
                    .andThen(new ParallelRaceGroup(
                                new TurnToHeading(m_Drivetrain, m_Path.sample(m_Path.getTotalTimeSeconds())
                                     .poseMeters.getRotation().getDegrees())),
                                new WaitCommand(0.1)); //cut from 0.25 sec
    }**/
    
}