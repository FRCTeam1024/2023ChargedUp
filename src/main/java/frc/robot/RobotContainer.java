// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;

import java.util.Arrays;
import java.util.Random;

import javax.swing.plaf.ComponentInputMapUIResource;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.hal.DriverStationJNI;
import edu.wpi.first.hal.simulation.DriverStationDataJNI;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableEvent;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableValue;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.ProxyCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.AutoAlignAprilTag;
import frc.robot.commands.AutoBalance;
import frc.robot.commands.AutoMoveToAprilTag;
import frc.robot.commands.DriveWithJoysticks;
import frc.robot.commands.HoldEndEffectorPosition;
import frc.robot.commands.PathPlannerCommand;
import frc.robot.commands.TestingPathPlannerCommand;
import frc.robot.oi.Logitech;
import frc.robot.Constants.*;
import frc.robot.subsystems.EndEffector;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.Vision;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  //Subsystems
  private final SwerveDrive drivetrain = new SwerveDrive();
  private final Arm arm = new Arm();
  private final EndEffector endEffector = new EndEffector();

  //Operator Inputs
  private final Logitech driverController = new Logitech(0);
  private final Logitech operatorController = new Logitech(1);

  //Default Commands
  private final DriveWithJoysticks driveWithController = new DriveWithJoysticks(drivetrain, driverController, 1);
  private final HoldEndEffectorPosition holdEndEffectorPosition = new HoldEndEffectorPosition(endEffector);
  //Chooser for auto
  SendableChooser<Command> m_AutoChooser = new SendableChooser<>();

  Field2d field = new Field2d();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    
    // Configure the dashboard
    configureDashboard();

    //Assign default commands
    drivetrain.setDefaultCommand(driveWithController);
    endEffector.setDefaultCommand(holdEndEffectorPosition);

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
    driverController.leftTrigger.whileTrue(new DriveWithJoysticks(drivetrain,driverController,0.35));
 
    driverController.rightBumper.onTrue(new ProxyCommand(
      () -> drivetrain.followVisionTrajectory()
    ));
    

    //OPERATOR CONTROLS


    //controls for arm - could change if hand needs more buttons
    operatorController.dPadUp.whileTrue(new ProxyCommand(() -> arm.moveTo(ArmConstants.highLevel)));  //Use moveTo(ArmConstant.MaxArmAngle)
    operatorController.dPadDown.whileTrue(new ProxyCommand(() -> arm.moveTo(ArmConstants.stowLevel)));  //Use moveTo(ArmConstants.MinArmAngle)

    operatorController.aButton.onTrue(new ProxyCommand(() -> arm.moveTo(ArmConstants.stowLevel)));
    operatorController.xButton.onTrue(new ProxyCommand(() -> arm.moveTo(ArmConstants.lowLevel)));
    operatorController.bButton.onTrue(new ProxyCommand(() -> arm.moveTo(ArmConstants.midLevel)));
    operatorController.yButton.onTrue(new ProxyCommand(() -> arm.moveTo(ArmConstants.highLevel)));

    //need to test and see if these should be instantcommands or proxycommands, as well as if we need an automatic stop after movement
    operatorController.dPadLeft.whileTrue(new ProxyCommand(() -> endEffector.intakeCone(true)));
    operatorController.dPadRight.whileTrue(new ProxyCommand(() -> endEffector.intakeCone(false)));

    operatorController.leftTrigger.whileTrue(new ProxyCommand(() -> endEffector.flipCone()));
    operatorController.leftBumper.whileTrue(new ProxyCommand(() -> endEffector.releaseCone()));
    operatorController.rightTrigger.whileTrue(new InstantCommand(() -> endEffector.intakeCube()));
    operatorController.rightBumper.whileTrue(new InstantCommand(() -> endEffector.releaseCube()));
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
    m_AutoChooser.addOption("Test Path", new ProxyCommand(() -> TestAuto()));
    m_AutoChooser.addOption("Test2", new ProxyCommand(() -> TestAuto2()));
    m_AutoChooser.addOption("Testing Swerve Auto", new ProxyCommand(() -> returnAutoCommand()));
    m_AutoChooser.addOption("Testing Auto Balance", new ProxyCommand(() -> TestAutoBalance()));
    m_AutoChooser.addOption("C-Charge", new ProxyCommand(() -> C_Charge()));
    m_AutoChooser.addOption("C-Cross-Charge", new ProxyCommand(() -> C_Cross_Charge()));
    m_AutoChooser.addOption("C-1-O", new ProxyCommand(() -> C_1_O()));
    m_AutoChooser.addOption("C-2-O", new ProxyCommand(() -> C_2_O()));
    m_AutoChooser.addOption("C-OuterRoute-Charge", new ProxyCommand(() -> C_OuterRoute_Charge()));
    m_AutoChooser.addOption("C-4-I", new ProxyCommand(() -> C_4_I()));
    m_AutoChooser.addOption("C-3-I", new ProxyCommand(() -> C_3_I()));
    m_AutoChooser.addOption("I-3-I", new ProxyCommand(() -> I_3_I()));
    m_AutoChooser.addOption("I-4-I", new ProxyCommand(() -> I_4_I()));
    m_AutoChooser.addOption("O-1-O", new ProxyCommand(() -> O_1_O()));
    m_AutoChooser.addOption("O-2-O", new ProxyCommand(() -> O_2_O()));
    m_AutoChooser.addOption("C-InnerRoute-Charge", new ProxyCommand(() -> C_InnerRoute_Charge()));
    m_AutoChooser.addOption("I-Charge", new ProxyCommand(() -> I_Charge()));
    m_AutoChooser.addOption("O-Charge", new ProxyCommand(() -> O_Charge()));
    

    //Put the auto chooser on the dashboard
    driverTab.add("Auto Mode",m_AutoChooser)
       .withSize(3,1)
       .withPosition(0,0);

    driverTab.addCamera("limelight", "OV5647", "http://photonvision.local:1182/stream.mjpg")
        .withSize(4,4)
        .withPosition(3,0); //double check obtaining camera streams

    driverTab.addNumber("Arm Angle", () -> arm.getArmAngle())
        .withSize(1,1)
        .withPosition(0,1);

    driverTab.addNumber("Crank Angle", () -> arm.getCrankAngle())
        .withSize(1,1)
        .withPosition(0,2);

    driverTab.addNumber("Raw Crank Angle", () -> arm.getRawCrankAngle())
        .withSize(1,1)
        .withPosition(0,3);

    /**driverTab.addNumber("AprilTag ID", () -> drivetrain.getCamera().getBestTarget().getFiducialId())
        .withSize(1,1)
        .withPosition(0,2);

    driverTab.addBoolean("AprilTag targets", () -> drivetrain.getCamera().hasTargets())
        .withSize(1,1)
        .withPosition(0,3);*/

    diagnosticsTab.addNumber("SwerveModule A Angle", () -> drivetrain.getAngleRad(1))
        .withSize(1,1)
        .withPosition(5,0);

    diagnosticsTab.addNumber("SwerveModule B Angle", () -> drivetrain.getAngleRad(2))
        .withSize(1,1)
        .withPosition(6,0);

    diagnosticsTab.addNumber("SwerveModule C Angle", () -> drivetrain.getAngleRad(3))
        .withSize(1,1)
        .withPosition(5,1);

    diagnosticsTab.addNumber("SwerveModule D Angle", () -> drivetrain.getAngleRad(4))
        .withSize(1,1)
        .withPosition(6,1);

    diagnosticsTab.addNumber("Robot Yaw", () -> drivetrain.getYawDegrees())
        .withSize(1,1)
        .withPosition(0,0);

    diagnosticsTab.addNumber("X-Position", () -> drivetrain.getPose().getX())
        .withSize(1,1)
        .withPosition(1,0);

    //putting the field in in case we want to have that visualization on shuffleboard
    diagnosticsTab.add(field)
        .withSize(4,3)
        .withPosition(1,1);

    diagnosticsTab.addString("Vision Estimated Pose", () -> drivetrain.getPose().toString())
        .withSize(4,1)
        .withPosition(0,4);

    diagnosticsTab.add("AutoAlignAprilTag", new AutoAlignAprilTag(drivetrain, drivetrain.getCamera()))
        .withSize(2,1)
        .withPosition(0,2);

    diagnosticsTab.addNumber("Target Yaw", () -> drivetrain.getTargetYaw())
        .withSize(1,1)
        .withPosition(0,3);

    diagnosticsTab.add("AutoMoveToAprilTag", new AutoMoveToAprilTag(drivetrain, drivetrain.getCamera()).move())
        .withSize(2,1)
        .withPosition(7, 0);

    diagnosticsTab.add("AutoBalance", new AutoBalance(drivetrain))
        .withSize(2,1)
        .withPosition(7,2);

    diagnosticsTab.addString("Pose", () -> drivetrain.getPose().toString())
        .withSize(2,1)
        .withPosition(7,1);

    diagnosticsTab.addNumber("armEncoder", () -> arm.getCrankAngle())
        .withSize(1,1)
        .withPosition(0,1);

    


    /**diagnosticsTab.addNumber("RobotPitch", () -> drivetrain.getPitch())
        .withSize(1,1)
        .withPosition(7,1);

    diagnosticsTab.addNumber("RobotRoll", () -> drivetrain.getRoll())
        .withSize(1,1)
        .withPosition(8,1);

    diagnosticsTab.addNumber("Charging Station Angle", () -> drivetrain.getChargeStationAngle())
        .withSize(1,1)
        .withPosition(7,3);*/

    driverTab.addNumber("Battery Voltage", () -> RobotController.getBatteryVoltage())
        .withSize(1,1)
        .withPosition(1,3);
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

  public Command returnAutoCommand(){
    return drivetrain.followTrajectory(PathPlanner.loadPath("Test Path", new PathConstraints(2.5, 2.5)));
  }

  private Command TestAuto(){
    PathPlannerTrajectory path = PathPlanner.loadPath("Test Path", new PathConstraints(2.5, 2.5));
    return new SequentialCommandGroup(
      new InstantCommand(() -> drivetrain.resetPosition(path.getInitialHolonomicPose())),
      new ParallelCommandGroup(
        new PPSwerveControllerCommand(
          path,
          drivetrain::getPose, // Pose supplier
          drivetrain.getSwerveDriveKinematics(), // SwerveDriveKinematics
          new PIDController(7.5, 0, 0), // X controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
          new PIDController(7.5, 0, 0), // Y controller (usually the same values as X controller)
          new PIDController(0.5, 0, 0.005), // Rotation controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
          drivetrain::setModuleStates, // Module states consumer
          drivetrain // Requires this drive subsystem
        )
      ),
      new InstantCommand(() -> drivetrain.defenseMode())
    );
  }

  //Attempting to pass auto into a separate command has not worked, even with the exact same arguments
  private Command TestAuto2(){
    PathPlannerTrajectory path = PathPlanner.loadPath("Circle Path", new PathConstraints(1, 1));
    return new SequentialCommandGroup(
      new PathPlannerCommand(path,drivetrain,true).configure(),
      new InstantCommand(() -> drivetrain.defenseMode())
    );
  }

  private Command TestAutoBalance(){
    PathPlannerTrajectory path = PathPlanner.loadPath("AutoBalanceTest", new PathConstraints(2.5, 2.5));
    return new SequentialCommandGroup(
      new InstantCommand(() -> drivetrain.resetPosition(path.getInitialHolonomicPose())),
      new ParallelCommandGroup(
        new PPSwerveControllerCommand(
          path,
          drivetrain::getPose, // Pose supplier
          drivetrain.getSwerveDriveKinematics(), // SwerveDriveKinematics
          new PIDController(7.5, 0, 0), // X controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
          new PIDController(7.5, 0, 0), // Y controller (usually the same values as X controller)
          new PIDController(0.5, 0, 0.005), // Rotation controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
          drivetrain::setModuleStates, // Module states consumer
          drivetrain // Requires this drive subsystem
        )
      ),
      new AutoBalance(drivetrain),
      new InstantCommand(() -> drivetrain.defenseMode())
    );
  }

  private Command C_Charge(){
    PathPlannerTrajectory path = PathPlanner.loadPath("C_Charge", new PathConstraints(1, 1));
    return new SequentialCommandGroup(
      new PrintCommand("\n\n" + path.getEndState().toString() + "\n\n"),
      drivetrain.followTrajectory(path),
      new AutoBalance(drivetrain)
    );
  }

  private Command C_Cross_Charge(){
    PathPlannerTrajectory path = PathPlanner.loadPath("C-Cross-Charge", new PathConstraints(1.5,1.5));
    return new SequentialCommandGroup(
      drivetrain.followTrajectory(path),
      new AutoBalance(drivetrain)
    );
  }

  private Command C_1_O(){
    PathPlannerTrajectory path = PathPlanner.loadPath("C-1-O", new PathConstraints(2,2));
    return new SequentialCommandGroup(
      drivetrain.followTrajectory(path)
    );
  }

  private Command C_2_O(){
    PathPlannerTrajectory path = PathPlanner.loadPath("C-2-O", new PathConstraints(1,1));
    return new SequentialCommandGroup(
      drivetrain.followTrajectory(path)
    );
  }

  private Command C_3_I(){
    PathPlannerTrajectory path = PathPlanner.loadPath("C-3-I", new PathConstraints(1,1));
    return new SequentialCommandGroup(
      drivetrain.followTrajectory(path)
    );
  }

  private Command C_4_I(){
    PathPlannerTrajectory path = PathPlanner.loadPath("C-4-I", new PathConstraints(2,2));
    return new SequentialCommandGroup(
      drivetrain.followTrajectory(path)
    );
  }

  private Command C_InnerRoute_Charge(){
    PathPlannerTrajectory path = PathPlanner.loadPath("C-InnerRoute-Charge", new PathConstraints(1,1));
    return new SequentialCommandGroup(
      drivetrain.followTrajectory(path),
      new AutoBalance(drivetrain)
    );
  }

  private Command C_OuterRoute_Charge(){
    PathPlannerTrajectory path = PathPlanner.loadPath("C-OuterRoute-Charge", new PathConstraints(1.5,1.5));
    return new SequentialCommandGroup(
      drivetrain.followTrajectory(path),
      new AutoBalance(drivetrain)
    );
  }

  private Command I_3_I(){
    PathPlannerTrajectory path = PathPlanner.loadPath("I-3-I", new PathConstraints(2,2));
    return new SequentialCommandGroup(
      drivetrain.followTrajectory(path)
    );
  }

  private Command I_4_I(){
    PathPlannerTrajectory path = PathPlanner.loadPath("I-4-I", new PathConstraints(2,2));
    return new SequentialCommandGroup(
      drivetrain.followTrajectory(path)
    );
  }

  private Command I_Charge(){
    PathPlannerTrajectory path = PathPlanner.loadPath("I-Charge", new PathConstraints(1,1));
    return new SequentialCommandGroup(
      drivetrain.followTrajectory(path)
    );
  }

  private Command O_1_O(){
    PathPlannerTrajectory path = PathPlanner.loadPath("O-1-O", new PathConstraints(1,1));
    return new SequentialCommandGroup(
      drivetrain.followTrajectory(path)
    );
  }

  private Command O_2_O(){
    PathPlannerTrajectory path = PathPlanner.loadPath("O-2-O", new PathConstraints(1,1));
    return new SequentialCommandGroup(
      drivetrain.followTrajectory(path)
    );
  }

  private Command O_Charge(){
    PathPlannerTrajectory path = PathPlanner.loadPath("O_Charge", new PathConstraints(1,1));
    return new SequentialCommandGroup(
      drivetrain.followTrajectory(path)
    );
  }
}
