// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.SwerveDrive;

public class PathPlannerCommand extends RamseteCommand {

    private Trajectory m_Path;
    private SwerveDrive m_Drivetrain;

    public PathPlannerCommand (Trajectory path, SwerveDrive drivetrain) {

        super(
            path,
            drivetrain::getPose,
            new RamseteController(DriveConstants.kRamseteB, DriveConstants.kRamseteZeta),
            new SimpleMotorFeedforward(DriveConstants.ksVolts,
                                       DriveConstants.kvVoltSecondsPerMeter,
                                       DriveConstants.kaVoltSecondsSquaredPerMeter),
            DriveConstants.kDriveKinematics,
            drivetrain::getWheelSpeeds,
            new PIDController(DriveConstants.kPDriveVel, 0, 0),
            new PIDController(DriveConstants.kPDriveVel, 0, 0),
            // RamseteCommand passes volts to the callback
            drivetrain::tankDriveVolts,
            drivetrain
        );

        m_Path = path;
        m_Drivetrain = drivetrain;


    }

    public SequentialCommandGroup configure() {
        return this.beforeStarting(() -> m_Drivetrain.resetOdometry(m_Path.getInitialPose()))
                    .andThen(() -> m_Drivetrain.tankDriveVolts(0, 0))
                    .andThen(new ParallelRaceGroup(
                                new TurnToHeading(m_Drivetrain, m_Path.sample(m_Path.getTotalTimeSeconds())
                                     .poseMeters.getRotation().getDegrees())),
                                new WaitCommand(0.1)); //cut from 0.25 sec
    }
    
}