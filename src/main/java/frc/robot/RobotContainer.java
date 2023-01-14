// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;

import java.util.Arrays;
import java.util.Random;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.DriveWithJoysticks;
import frc.robot.commands.PathPlannerCommand;
import frc.robot.oi.Logitech;
import frc.robot.Constants.*;
import frc.robot.subsystems.SwerveDrive;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  //Subsystems
  private final SwerveDrive drivetrain = new SwerveDrive();

  //Operator Inputs
  private final Logitech driverController = new Logitech(0);

  //Default Commands
  private final DriveWithJoysticks driveWithController = new DriveWithJoysticks(drivetrain, driverController, true);

  //Chooser for auto
  SendableChooser<Command> m_AutoChooser = new SendableChooser<>();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    
    // Configure the dashboard
    configureDashboard();

    //Assign default commands
    drivetrain.setDefaultCommand(driveWithController);

    // Configure the button bindings
    configureButtonBindings();
  }

    /**
   * List any PID subsystems here so that they get disabled when the robot
   * is disabled and integral error doesn't accumulate.  Usually doesn't matter
   * since we often don't use integral gain but just in case.
   */
  public void disablePIDSubsystems() {
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    //need to add button to switch from field to robot relative

    // DRIVER CONTROLS
    driverController.aButton.onTrue(new InstantCommand(() -> drivetrain.zeroHeading()));
    driverController.rightTrigger.whileTrue(new DriveWithJoysticks(drivetrain, driverController, false));
    //OPERATOR CONTROLS
  }

   /**
   * Use this method to configure the dashboard
   * 
   */
  private void configureDashboard() {

    //Create ShuffleBoard Tabs
    ShuffleboardTab diagnosticsTab = Shuffleboard.getTab("1024Diagnostics");
    ShuffleboardTab driverTab = Shuffleboard.getTab("1024Driver");
    
    /**
     * Diagnostics for programmers
     */
    //Add command status to dashboard
    diagnosticsTab.add("DrivetrainCommand",drivetrain)
       .withSize(2,1)
       .withPosition(8,0);

    /**
     * Driver's operator interface
     */

    //Display the name and version number of the code.
    /**driverTab.add("Running Code Version:", BuildConfig.APP_NAME + " " + BuildConfig.APP_VERSION)
        .withSize(3,1)
        .withPosition(0,0);*/

    //Add commands to auto chooser, set default to null to avoid surprise operation
    m_AutoChooser.setDefaultOption("None", null);
    m_AutoChooser.addOption("Test Path", TestAuto());
    m_AutoChooser.addOption("Test2", TestAuto2());

    //Put the auto chooser on the dashboard
    driverTab.add("Auto Mode",m_AutoChooser)
       .withSize(3,1)
       .withPosition(3,0);


    driverTab.addNumber("SwerveModule A Angle", () -> drivetrain.getAngleRad(1))
        .withSize(1,1)
        .withPosition(4,0);

    driverTab.addNumber("SwerveModule B Angle", () -> drivetrain.getAngleRad(2))
        .withSize(1,1)
        .withPosition(5,0);

    driverTab.addNumber("SwerveModule C Angle", () -> drivetrain.getAngleRad(3))
        .withSize(1,1)
        .withPosition(4,1);

    driverTab.addNumber("SwerveModule D Angle", () -> drivetrain.getAngleRad(4))
        .withSize(1,1)
        .withPosition(5,1);

    driverTab.addNumber("Robot Yaw", () -> drivetrain.getYawDegrees())
        .withSize(1,1);
/*
    driverTab.addNumber("SwerveModule A Target Angle", () -> drivetrain.getTargetAngleRad(1))
        .withSize(1,1)
        .withPosition(4,3);

    driverTab.addNumber("SwerveModule B Target Angle", () -> drivetrain.getTargetAngleRad(2))
        .withSize(1,1)
        .withPosition(5,3);

    driverTab.addNumber("SwerveModule C Target Angle", () -> drivetrain.getTargetAngleRad(3))
        .withSize(1,1)
        .withPosition(4,4);

    driverTab.addNumber("SwerveModule D Target Angle", () -> drivetrain.getTargetAngleRad(4))
        .withSize(1,1)
        .withPosition(5,4);
        */
    
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return m_AutoChooser.getSelected();
  }

  private Command TestAuto(){
    PathPlannerTrajectory path = PathPlanner.loadPath("Circle Path", new PathConstraints(2.5, 2.5));
    return new SequentialCommandGroup(
      new PrintCommand(path.toString()),
      new ParallelCommandGroup(
        new PPSwerveControllerCommand(
          path, 
          drivetrain::getPose, // Pose supplier
          drivetrain.getSwerveDriveKinematics(), // SwerveDriveKinematics
          new PIDController(0, 0, 0), // X controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
          new PIDController(0, 0, 0), // Y controller (usually the same values as X controller)
          new PIDController(0, 0, 0.005), // Rotation controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
          drivetrain::setModuleStates, // Module states consumer
          drivetrain // Requires this drive subsystem
        ),
        new PrintCommand(drivetrain.getStates().toString())
      ),
      new InstantCommand(() -> drivetrain.defenseMode())
    );
  }

  private Command TestAuto2(){
    PathPlannerTrajectory path = PathPlanner.loadPath("Circle Path", new PathConstraints(2.5, 2.5));
    return new SequentialCommandGroup(
      new PathPlannerCommand(path,drivetrain,true),
      new InstantCommand(() -> drivetrain.defenseMode())
    );
  }
}
