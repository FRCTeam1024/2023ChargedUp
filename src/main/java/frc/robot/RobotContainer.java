// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;

import java.util.Arrays;
import java.util.List;
import java.util.Map;
import java.util.Random;
import java.util.function.BooleanSupplier;

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
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.ProxyCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.AutoAlignAprilTag;
import frc.robot.commands.AutoBalance;
import frc.robot.commands.AutoMoveToAprilTag;
import frc.robot.commands.AutoTurn;
import frc.robot.commands.DriveWithJoysticks;
import frc.robot.commands.HoldEndEffectorPosition;
import frc.robot.commands.PathPlannerCommand;
import frc.robot.commands.TestingPathPlannerCommand;
import frc.robot.commands.TurnWristWithJoysticks;
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
  private final DriveWithJoysticks driveWithController = new DriveWithJoysticks(drivetrain, driverController, .58, arm);
  private final TurnWristWithJoysticks turnWristWithJoysticks = new TurnWristWithJoysticks(endEffector, operatorController);
 
  //Chooser for auto
  SendableChooser<Command> m_AutoChooser = new SendableChooser<>();

  Field2d field = new Field2d();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    resetDriver();
    // Configure the dashboard
    configureDashboard();

    //Assign default commands
    drivetrain.setDefaultCommand(driveWithController);
    endEffector.setDefaultCommand(turnWristWithJoysticks);

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
    driverController.leftTrigger.whileTrue(new DriveWithJoysticks(drivetrain,driverController,0.35, arm));
    driverController.leftBumper.whileTrue(new DriveWithJoysticks(drivetrain,driverController,1, arm));
    driverController.yButton.whileTrue(new ProxyCommand(() -> endEffector.turnWristToAngle(0)));
    //driverController.dPadUp.whileTrue(new AutoTurn(drivetrain, 0));
    //driverController.dPadDown.whileTrue(new AutoTurn(drivetrain, 180));
 

    /**driverController.rightBumper.onTrue(new ProxyCommand(
      () -> drivetrain.followVisionTrajectory()
    ));*/
    

    //OPERATOR CONTROLS


    //controls for arm - could change if hand needs more buttons
    operatorController.dPadUp.whileTrue(new ProxyCommand(() -> arm.moveTo(ArmConstants.highLevel)));  //Use moveTo(ArmConstant.MaxArmAngle)
    operatorController.dPadDown.whileTrue(new ProxyCommand(() -> arm.moveTo(ArmConstants.stowLevel)));  //Use moveTo(ArmConstants.MinArmAngle)

    operatorController.aButton.onTrue(new ProxyCommand(() -> arm.moveTo(ArmConstants.stowLevel)));
    operatorController.xButton.onTrue(new ProxyCommand(() -> arm.moveTo(ArmConstants.pickup)));
    operatorController.bButton.onTrue(new ProxyCommand(() -> arm.moveTo(ArmConstants.midLevel)));
    operatorController.yButton.onTrue(new ProxyCommand(() -> arm.moveTo(5)));

    //need to test and see if these should be instantcommands or proxycommands, as well as if we need an automatic stop after movement
    /**operatorController.dPadLeft.whileTrue(new InstantCommand(() -> endEffector.turnWrist(0.6)));
    operatorController.dPadLeft.onFalse(new InstantCommand(() -> endEffector.stop()));
    operatorController.dPadRight.whileTrue(new InstantCommand(() -> endEffector.turnWrist(-0.6)));
    operatorController.dPadRight.onFalse(new InstantCommand(() -> endEffector.stop()));*/

    //if no more reverse cones, change the below angle to 120 or something higher
    operatorController.leftTrigger.whileTrue(new ProxyCommand(() -> endEffector.turnWristToAngle(95)));
    operatorController.leftBumper.whileTrue(new ProxyCommand(() -> endEffector.turnWristToAngle(-75)));
    operatorController.rightTrigger.whileTrue(new InstantCommand(() -> endEffector.intakeCube()));
    operatorController.rightTrigger.onFalse(new InstantCommand(() -> endEffector.stop()));
    operatorController.rightBumper.whileTrue(new InstantCommand(() -> endEffector.releaseCube()));
    operatorController.rightBumper.onFalse(new InstantCommand(() -> endEffector.stop()));

    //Neo motor stall safety, prevent stalling motor for too long
    Trigger intakeStall = new Trigger(endEffector::intakeStalled).debounce(1);
    intakeStall.onTrue(new InstantCommand(endEffector::stop));

    operatorController.startButton.onTrue(new InstantCommand(() -> endEffector.resetWristAngle()));

    //Arm position calibration
    Trigger atLimit = new Trigger(arm::atCrankLimit).debounce(0.02);
    operatorController.backButton.and(atLimit.negate()).whileTrue(arm.calMove());
    atLimit.whileTrue(new SequentialCommandGroup(
                    new InstantCommand(() -> arm.simpleMove(0),arm),
                    new WaitCommand(0.1),
                    new InstantCommand(arm::resetArmAngle,arm)
    ));
                                                        
            


  }

   /**
   * Use this method to configure the dashboard
   * 
   */
  private void configureDashboard() {

    //Create ShuffleBoard Tabs
    ShuffleboardTab diagnosticsTab = Shuffleboard.getTab("1024Diagnostics");
    ShuffleboardTab driverTab = Shuffleboard.getTab("1024Driver");
    ShuffleboardTab swerveTab = Shuffleboard.getTab("1024Swerve");
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
    //m_AutoChooser.addOption("Test Path", new ProxyCommand(() -> TestAuto()));
    //m_AutoChooser.addOption("Test2", new ProxyCommand(() -> TestAuto2()));
    //m_AutoChooser.addOption("Testing Swerve Auto", new ProxyCommand(() -> returnAutoCommand()));
    //m_AutoChooser.addOption("Testing Auto Balance", new ProxyCommand(() -> TestAutoBalance()));
    //m_AutoChooser.addOption("C-Charge", new ProxyCommand(() -> C_Charge()));
    //m_AutoChooser.addOption("C-Cross-Charge", new ProxyCommand(() -> C_Cross_Charge()));
    //m_AutoChooser.addOption("C-1-O", new ProxyCommand(() -> C_1_O()));
    //m_AutoChooser.addOption("C-2-O", new ProxyCommand(() -> C_2_O()));
    //m_AutoChooser.addOption("C-OuterRoute-Charge", new ProxyCommand(() -> C_OuterRoute_Charge()));
    //m_AutoChooser.addOption("C-4-I", new ProxyCommand(() -> C_4_I()));
    //m_AutoChooser.addOption("C-3-I", new ProxyCommand(() -> C_3_I()));
    //m_AutoChooser.addOption("I-3-I", new ProxyCommand(() -> I_3_I()));
    //m_AutoChooser.addOption("I-4-I", new ProxyCommand(() -> I_4_I()));
    //m_AutoChooser.addOption("O-1-O", new ProxyCommand(() -> O_1_O()));
    //m_AutoChooser.addOption("O-2-O", new ProxyCommand(() -> O_2_O()));
    //m_AutoChooser.addOption("C-InnerRoute-Charge", new ProxyCommand(() -> C_InnerRoute_Charge()));
    //m_AutoChooser.addOption("I-Charge", new ProxyCommand(() -> I_Charge()));
    //m_AutoChooser.addOption("O-Charge", new ProxyCommand(() -> O_Charge()));
    m_AutoChooser.addOption("C-Cone-Cross-Charge", new ProxyCommand(() -> C_Cone_Cross_Charge()));
    m_AutoChooser.addOption("C-Cube-Cross-Charge", new ProxyCommand(() -> C_Cube_Cross_Charge()));
    m_AutoChooser.addOption("OuterConeGrab", new ProxyCommand(() -> OuterConeGrab()));
    m_AutoChooser.addOption("2OuterCones", new ProxyCommand(() -> OuterCones()));
    m_AutoChooser.addOption("2InnerCones", new ProxyCommand(() -> InnerCones()));
    m_AutoChooser.addOption("OuterConesCharge - Testing only", new ProxyCommand(() -> OuterConesCharge()));
    

    //Puts the auto chooser on the dashboard
    driverTab.add("Auto Mode",m_AutoChooser)
       .withSize(3,1)
       .withPosition(0,0);

    //Adds the limelight camera feed
    driverTab.addCamera("limelight", "OV5647", "http://photonvision.local:1182/stream.mjpg")
        .withSize(3,3)
        .withPosition(0,1); 

    driverTab.add("Arm Camera", arm.getFeed())
        .withSize(4,4)
        .withPosition(3,0)
        .withProperties(Map.of("ROTATION", "HALF"));

    driverTab.addNumber("Arm Angle", () -> arm.getArmAngle())
        .withSize(1,1)
        .withPosition(7,0);

    driverTab.addNumber("Raw Wrist Angle", () -> endEffector.getRawWristAngle())
        .withSize(1,1)
        .withPosition(7,1);

    driverTab.addNumber("Wrist Angle", () -> endEffector.getWristAngle())
        .withSize(1,1)
        .withPosition(7,2);

    /**driverTab.addNumber("AprilTag ID", () -> drivetrain.getCamera().getBestTarget().getFiducialId())
        .withSize(1,1)
        .withPosition(0,2);

    driverTab.addBoolean("AprilTag targets", () -> drivetrain.getCamera().hasTargets())
        .withSize(1,1)
        .withPosition(0,3);*/

    diagnosticsTab.addNumber("Robot Yaw", () -> drivetrain.getYawDegrees())
        .withSize(1,1)
        .withPosition(0,0);
        
    //Swerve module angles A-D
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

    diagnosticsTab.addNumber("Neo Temperature", () -> endEffector.getTemperature())
        .withSize(1,1)
        .withPosition(3,2);

    //Swerve module velocity errors A-D
    swerveTab.addNumber("SwerveModule A Vel Error", () -> drivetrain.getModVelError(1))
    .withSize(1,1)
    .withPosition(5,2);

    swerveTab.addNumber("SwerveModule B Vel Error", () -> drivetrain.getModVelError(2))
    .withSize(1,1)
    .withPosition(6,2);

    swerveTab.addNumber("SwerveModule C Vel Error", () -> drivetrain.getModVelError(3))
    .withSize(1,1)
    .withPosition(5,3);

    swerveTab.addNumber("SwerveModule D Vel Error", () -> drivetrain.getModVelError(4))
    .withSize(1,1)
    .withPosition(6,3);

    //Swerve module angle errors A-D
    swerveTab.addNumber("SwerveModule A Angle Error", () -> drivetrain.getModAngError(1))
    .withSize(1,1)
    .withPosition(7,2);

    swerveTab.addNumber("SwerveModule B Angle Error", () -> drivetrain.getModAngError(2))
    .withSize(1,1)
    .withPosition(8,2);

    swerveTab.addNumber("SwerveModule C Angle Error", () -> drivetrain.getModAngError(3))
    .withSize(1,1)
    .withPosition(7,3);

    swerveTab.addNumber("SwerveModule D Angle Error", () -> drivetrain.getModAngError(4))
    .withSize(1,1)
    .withPosition(8,3);

      //Tells if it is practice bot for different swevrve module offsets
    diagnosticsTab.addBoolean("Is Practice Bot", () -> Constants.PracticeBot)
        .withSize(1,1)
        .withPosition(4,1);

    diagnosticsTab.addBoolean("Is Comp Bot", () -> Constants.CompBot)
        .withSize(1,1)
        .withPosition(4,2);

    diagnosticsTab.addNumber("Wrist Angle", () -> endEffector.getWristAngle())
        .withSize(1,1)
        .withPosition(2,3);

    diagnosticsTab.addBoolean("Arm At Limit", () -> arm.atCrankLimit())
        .withSize(1,1)
        .withPosition(4,0);

    /**diagnosticsTab.addNumber("AbsoluteAngle", () -> endEffector.getAbsoluteAngle())
        .withSize(1,1)
        .withPosition(3,3);*/

    /**diagnosticsTab.addNumber("Robot Yaw", () -> drivetrain.getYawDegrees())
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
        .withPosition(7,1); */

    diagnosticsTab.addNumber("CrankAngle", () -> arm.getCrankAngle())
        .withSize(1,1)
        .withPosition(1,1);

    diagnosticsTab.addNumber("GoalVelocity", () -> arm.getGoalVelocity())
        .withSize(1,1)
        .withPosition(3,1);

    diagnosticsTab.addNumber("ArmVoltage", () -> arm.getVoltage())
        .withSize(1,1)
        .withPosition(5,1);


    diagnosticsTab.addNumber("RobotPitch", () -> drivetrain.getPitch())
        .withSize(1,1)
        .withPosition(7,1);

    diagnosticsTab.addNumber("RobotRoll", () -> drivetrain.getRoll())
        .withSize(1,1)
        .withPosition(8,1);

    diagnosticsTab.addNumber("Neo Voltage", () -> endEffector.getNeoVoltage())
        .withSize(1,1)
        .withPosition(4,3);

    /**diagnosticsTab.addNumber("Charging Station Angle", () -> drivetrain.getChargeStationAngle())
        .withSize(1,1)
        .withPosition(7,3);*/

    driverTab.addNumber("Battery Voltage", () -> RobotController.getBatteryVoltage())
        .withSize(1,1)
        .withPosition(7,3);

    diagnosticsTab.addString("Pose:", () -> drivetrain.getPose().toString())
        .withSize(3,1)
        .withPosition(5,2);

    diagnosticsTab.addString("Vision Estimated Pose:", () -> drivetrain.getVisionEstimatedPose().toString())
        .withSize(3,1)
        .withPosition(5,3);
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
  // Test path that moves in a straight line and back, while turning
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
      )
    );
  }

  //Attempting to pass auto into a separate command has not worked, even with the exact same arguments
  // Test path that moves into a circle
  private Command TestAuto2(){
    PathPlannerTrajectory path = PathPlanner.loadPath("Circle Path", new PathConstraints(1, 1));
    return new SequentialCommandGroup(
      new PathPlannerCommand(path,drivetrain,true).configure()
    );
  }
  //Initial test of just driving on to the charging station and running auto balance
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
      new AutoBalance(drivetrain)
    );
  }

  //Moves from the center grid, on to the charge station, then attempts to auto balance
  private Command C_Charge(){
    PathPlannerTrajectory path = PathPlanner.loadPath("C_Charge", new PathConstraints(1, 1));
    /**return new SequentialCommandGroup(
      new ProxyCommand(() -> arm.moveTo(-75)).withTimeout(1),
      new InstantCommand(() -> endEffector.releaseCube()),
      new WaitCommand(1),
      new InstantCommand(() -> endEffector.stop()),
      new ParallelDeadlineGroup(
        drivetrain.followTrajectory(path),
        //arm.moveTo(ArmConstants.stowLevel) just removed for sketchy testing
        new ProxyCommand(() -> arm.moveTo(-120)).withTimeout(1)
      ),
      new PrintCommand("Does this finish these commands?")
      new AutoBalance(drivetrain)
    
    );*/
    return new SequentialCommandGroup(
      new InstantCommand(() -> endEffector.resetWristAngle()),
      drivetrain.followTrajectory(path).withTimeout(6),
      new AutoBalance(drivetrain)
    );
  }

  //Moves from center grid onto and over the charge station, and then moves directly back onto the charge station to auto balance
  private Command C_Cross_Charge(){
    PathPlannerTrajectory path = PathPlanner.loadPath("C-Cross-Charge", new PathConstraints(1.5,1.5));
    return new SequentialCommandGroup(
      /**new PrintCommand("\n\n" + path.getEndState().toString() + "\n\n"),
      arm.moveTo(ArmConstants.lowLevel),
      new InstantCommand(() -> endEffector.releaseCube()),
      new WaitCommand(0.5),
      new InstantCommand(() -> endEffector.stop()),
      new ParallelDeadlineGroup(
        drivetrain.followTrajectory(path),
        arm.moveTo(ArmConstants.stowLevel)
      ),*/
      new InstantCommand(() -> endEffector.resetWristAngle()),
      drivetrain.followTrajectory(path),
      new AutoBalance(drivetrain)
    );
  }

    //Places a high cone Moves from center grid onto and over the charge station, and then moves directly back onto the charge station to auto balance
  private Command C_Cone_Cross_Charge(){
    List<PathPlannerTrajectory> path = PathPlanner.loadPathGroup("C-Cone-Cross-Charge", 
                                          new PathConstraints(1.5,1.5),
                                          new PathConstraints(2,2),
                                          new PathConstraints(2.5,2.5));
    return new SequentialCommandGroup(
      new InstantCommand(() -> endEffector.resetWristAngle()),
      new ParallelCommandGroup(
        arm.moveToAuto(14).withTimeout(2.5),
        new SequentialCommandGroup(
          new WaitCommand(1),
          drivetrain.followTrajectory(path.get(0))
        )
      ),
      arm.moveToAuto(-5).withTimeout(1),
      new InstantCommand(() -> endEffector.runIntakeAuto(-0.3)),
      new ParallelCommandGroup(
        new SequentialCommandGroup(
          drivetrain.followTrajectory(path.get(1)),
          drivetrain.followTrajectory(path.get(2))
        ),
        new SequentialCommandGroup(
          arm.moveToAuto(ArmConstants.highLevel).withTimeout(0.5),
          new InstantCommand(() -> endEffector.stop()),
          arm.moveToAuto(ArmConstants.stowLevel).withTimeout(3)
        )
      ),
      new AutoBalance(drivetrain)
    );
  }

    //Places a high cone Moves from center grid onto and over the charge station, and then moves directly back onto the charge station to auto balance
    //needto rearrange lift up for cube
  private Command C_Cube_Cross_Charge(){
    List<PathPlannerTrajectory> path = PathPlanner.loadPathGroup("C-Cone-Cross-Charge", 
                                          new PathConstraints(1.5,1.5),
                                          new PathConstraints(2,2),
                                          new PathConstraints(2.5,2.5));
    return new SequentialCommandGroup(
      new InstantCommand(() -> endEffector.resetWristAngle()),
      new ParallelCommandGroup(
        arm.moveToAuto(14).withTimeout(2.5),
        new SequentialCommandGroup(
          new WaitCommand(1),
          drivetrain.followTrajectory(path.get(0))
        )
      ),
      arm.moveToAuto(-5).withTimeout(1),
      new InstantCommand(() -> endEffector.runIntakeAuto(0.3)),
      new ParallelCommandGroup(
        new SequentialCommandGroup(
          drivetrain.followTrajectory(path.get(1)),
          drivetrain.followTrajectory(path.get(2))
        ),
        new SequentialCommandGroup(
          arm.moveToAuto(ArmConstants.highLevel).withTimeout(0.5),
          new InstantCommand(() -> endEffector.stop()),
          arm.moveToAuto(ArmConstants.stowLevel).withTimeout(3)
        )
      ),
      new AutoBalance(drivetrain)
    );
  }

  private Command OuterConeGrab(){
    List<PathPlannerTrajectory> path = PathPlanner.loadPathGroup("2OuterCone",
                                      new PathConstraints(1.5,1.5),
                                      new PathConstraints(2,2),
                                      new PathConstraints(0,0), //3rd pathconstraints is unused
                                      new PathConstraints(0,0)); //4th pathconstraints is unused
    return new SequentialCommandGroup(
      new InstantCommand(() -> endEffector.resetWristAngle()),
      new ParallelCommandGroup(
        arm.moveToAuto(ArmConstants.highLevel).withTimeout(2.5),
        new SequentialCommandGroup(
          new WaitCommand(1),
          drivetrain.followTrajectory(path.get(0))
        )
      ),
      arm.moveToAuto(-5).withTimeout(1),
      new InstantCommand(() -> endEffector.intakeCube()),
      arm.moveToAuto(ArmConstants.highLevel).withTimeout(1),
      new ParallelCommandGroup(
        drivetrain.followTrajectory(path.get(1)),
        new InstantCommand(() -> endEffector.intakeCube()),
        new SequentialCommandGroup(
          new WaitCommand(0.25),
          arm.moveToAuto(ArmConstants.pickup).withTimeout(3)
        )
      ),
      new WaitCommand(1),
      new InstantCommand(() -> endEffector.stop())
    );
  }

  private Command OuterCones(){
    List<PathPlannerTrajectory> path = PathPlanner.loadPathGroup("2OuterCone",
                                      new PathConstraints(1.5,1.5),
                                      new PathConstraints(2,2),
                                      new PathConstraints(2,2),
                                      new PathConstraints(0,0)); //4th pathconstraints is unused
    return new SequentialCommandGroup(
      new InstantCommand(() -> endEffector.resetWristAngle()),
      new ParallelCommandGroup(
        arm.moveToAuto(ArmConstants.highLevel).withTimeout(2.5),
        new SequentialCommandGroup(
          new WaitCommand(1),
          drivetrain.followTrajectory(path.get(0))
        )
      ),
      arm.moveToAuto(-5).withTimeout(1),
      new InstantCommand(() -> endEffector.intakeCube()),
      arm.moveToAuto(ArmConstants.highLevel).withTimeout(1),
      new ParallelCommandGroup(
        drivetrain.followTrajectory(path.get(1)),
        new InstantCommand(() -> endEffector.intakeCube()),
        new SequentialCommandGroup(
          new WaitCommand(0.25),
          arm.moveToAuto(ArmConstants.pickup).withTimeout(3)
        )
      ),
      new WaitCommand(0.1),
      new ParallelCommandGroup(
        drivetrain.followTrajectory(path.get(2)),
        new SequentialCommandGroup(
          new WaitCommand(1),
          arm.moveToAuto(ArmConstants.highLevel).withTimeout(3)
        ),
        new SequentialCommandGroup(
          new WaitCommand(1.5),
          new InstantCommand(() -> endEffector.stop())
        )
      ),
      new InstantCommand(() -> endEffector.releaseCube())
    );
  }

  private Command OuterConesCharge(){
    List<PathPlannerTrajectory> path = PathPlanner.loadPathGroup("2OuterCone",
                                      new PathConstraints(1.5,1.5),
                                      new PathConstraints(2,2), //next up this speed, verify functionality
                                      new PathConstraints(2,2),
                                      new PathConstraints(2.5,2.5)); //can up this speed
    return new SequentialCommandGroup(
      new InstantCommand(() -> endEffector.resetWristAngle()),
      new ParallelCommandGroup(
        arm.moveToAuto(ArmConstants.highLevel).withTimeout(2.5),
        new SequentialCommandGroup(
          new WaitCommand(1),
          drivetrain.followTrajectory(path.get(0))
        )
      ),
      arm.moveToAuto(-5).withTimeout(1),
      new InstantCommand(() -> endEffector.intakeCube()),
      arm.moveToAuto(ArmConstants.highLevel).withTimeout(1),
      new ParallelCommandGroup(
        drivetrain.followTrajectory(path.get(1)),
        new InstantCommand(() -> endEffector.intakeCube()),
        new SequentialCommandGroup(
          new WaitCommand(0.25), //might need to decrease this time to get the arm down fast enough
          arm.moveToAuto(ArmConstants.pickup).withTimeout(3) //check whether the arm or path is completed faster
        )
      ),
      new WaitCommand(0.1), //could remove this time fully
      new ParallelCommandGroup(
        drivetrain.followTrajectory(path.get(2)),
        new SequentialCommandGroup(
          new WaitCommand(1), //if we up the speed, this may need to decrease a little bit
          arm.moveToAuto(ArmConstants.highLevel).withTimeout(3)//check whether the arm or path is completed faster
        ),
        new SequentialCommandGroup(
          new WaitCommand(1.5),
          new InstantCommand(() -> endEffector.stop())
        )
      ),
      new InstantCommand(() -> endEffector.releaseCube()),
      new WaitCommand(0.25), //can decrease this time
      new InstantCommand(() -> endEffector.stop()),//cutoff from last path
      new ParallelCommandGroup(
        drivetrain.followTrajectory(path.get(3)),
        new SequentialCommandGroup(
          new WaitCommand(1), //can decrease this time
          arm.moveToAuto(ArmConstants.stowLevel).withTimeout(2.5)
        )
      ),
      new PrintCommand("Starting AutoBalance"),
      new AutoBalance(drivetrain),
      new PrintCommand("AutoBalance Finished")
    );
  }

  private Command InnerCones(){
    List<PathPlannerTrajectory> path = PathPlanner.loadPathGroup("2InnerCone",
                                      new PathConstraints(1.5,1.5),
                                      new PathConstraints(1.5,1.5),
                                      new PathConstraints(2,2),
                                      new PathConstraints(2,2));
    return new SequentialCommandGroup(
      new InstantCommand(() -> endEffector.resetWristAngle()),
      new ParallelCommandGroup(
        arm.moveToAuto(ArmConstants.highLevel).withTimeout(2.5),
        new SequentialCommandGroup(
          new WaitCommand(1),
          drivetrain.followTrajectory(path.get(0))
        )
      ),
      arm.moveToAuto(-5).withTimeout(1),
      new InstantCommand(() -> endEffector.intakeCube()),
      arm.moveToAuto(ArmConstants.highLevel).withTimeout(1),
      new ParallelCommandGroup(
        drivetrain.followTrajectory(path.get(1)),
        new SequentialCommandGroup(
          new WaitCommand(0.5),
          arm.moveToAuto(ArmConstants.stowLevel).withTimeout(2.5)
        ),
        endEffector.turnWristToOffset(-10).withTimeout(1)
      ),
      new ParallelCommandGroup(
        new InstantCommand(() -> endEffector.intakeCube()),
        drivetrain.followTrajectory(path.get(2)),
        new SequentialCommandGroup(
          new WaitCommand(2.3),
          arm.moveToAuto(ArmConstants.pickup).withTimeout(1)
        )
      ),
      new WaitCommand(1),
      new InstantCommand(() -> endEffector.stop()),
      new ParallelCommandGroup(
        new SequentialCommandGroup(      
          arm.moveToAuto(ArmConstants.stowLevel).withTimeout(1),
          new WaitCommand(0.5),
          arm.moveToAuto(ArmConstants.highLevel).withTimeout(2.5)
        ),
        new SequentialCommandGroup(
          new WaitCommand(0.5),
          drivetrain.followTrajectory(path.get(3))
        )
      ),
      new InstantCommand(() -> endEffector.runIntakeAuto(-0.5))
    );
  }

  // Moves from center grid, to the first cube, picks it up, and then moves to the outer grid to score it.
  private Command C_1_O(){
    List<PathPlannerTrajectory> path = PathPlanner.loadPathGroup("C-1-O",
                                      new PathConstraints(1,1),
                                      new PathConstraints(1,1));
    return new SequentialCommandGroup(
      arm.moveTo(ArmConstants.lowLevel),
      new InstantCommand(() -> endEffector.releaseCube()),
      new WaitCommand(0.5),
      new InstantCommand(() -> endEffector.stop()),
      new ParallelDeadlineGroup(
        drivetrain.followTrajectory(path.get(0))
      ),
      new InstantCommand(() -> endEffector.intakeCube()),
      new WaitCommand(0.5),
      new InstantCommand(() -> endEffector.stop()),
      new ParallelDeadlineGroup(
        drivetrain.followTrajectory(path.get(1))
      ),
      new InstantCommand(() -> endEffector.releaseCube()),
      new WaitCommand(0.5),
      new InstantCommand(() -> endEffector.stop())
    );
  }

  // Moves from center grid, to the second cube, picks it up, and then moves to the outer grid to score it.
  private Command C_2_O(){
    PathPlannerTrajectory path = PathPlanner.loadPath("C-2-O", new PathConstraints(1,1));
    return new SequentialCommandGroup(
      drivetrain.followTrajectory(path)
    );
  }
  // Moves from center grid, to the third cube, picks it up, and then moves to the inner grid to score it.
  private Command C_3_I(){
    PathPlannerTrajectory path = PathPlanner.loadPath("C-3-I", new PathConstraints(1,1));
    return new SequentialCommandGroup(
      drivetrain.followTrajectory(path)
    );
  }
  // Moves from center grid, to the fourth cube, picks it up, and then moves to the inner grid to score it.
  private Command C_4_I(){
    PathPlannerTrajectory path = PathPlanner.loadPath("C-4-I", new PathConstraints(2,2));
    return new SequentialCommandGroup(
      drivetrain.followTrajectory(path)
    );
  }
  // Moves from the center grid, towards the inner grid, and then proceeds to go all the way around, drives up onto the charge station and auto balances.
  private Command C_InnerRoute_Charge(){
    PathPlannerTrajectory path = PathPlanner.loadPath("C-InnerRoute-Charge", new PathConstraints(1,1));
    return new SequentialCommandGroup(
      drivetrain.followTrajectory(path),
      new AutoBalance(drivetrain)
    );
  }
  // Moves from the center grid, towards the outer grid, and then proceeds to go all the way around, drives up onto the charge station and auto balances.
  private Command C_OuterRoute_Charge(){
    PathPlannerTrajectory path = PathPlanner.loadPath("C-OuterRoute-Charge", new PathConstraints(1.5,1.5));
    return new SequentialCommandGroup(
      drivetrain.followTrajectory(path),
      new AutoBalance(drivetrain)
    );
  }
  // Moves from inner grid, to the third cube, picks it up, and then moves back to the inner grid to score it.
  private Command I_3_I(){
   List<PathPlannerTrajectory> path = PathPlanner.loadPathGroup("I-3-I", new PathConstraints(2,2),
                                                                              new PathConstraints(2,2));
    return new SequentialCommandGroup(
      new ParallelDeadlineGroup(
        new SequentialCommandGroup(
          drivetrain.followTrajectory(path.get(0)),
          new WaitCommand(1)
        ),
        new ProxyCommand(() -> arm.moveTo(ArmConstants.lowLevel)),
        new InstantCommand(() -> endEffector.intakeCube())
      ),
      new InstantCommand(() -> endEffector.stop()),
      new ParallelDeadlineGroup(
        new WaitCommand(4),
        drivetrain.followTrajectory(path.get(1)),
        new SequentialCommandGroup(
          new WaitCommand(2),
          new ProxyCommand(() -> arm.moveTo(ArmConstants.midLevel))
        )
      ),
      new AutoMoveToAprilTag(drivetrain, drivetrain.getCamera()).withTimeout(1),
      new InstantCommand(() -> endEffector.releaseCube()),
      new WaitCommand(0.5),
      new InstantCommand(() -> endEffector.stop())
    );
  }
  // Moves from inner grid, to the third cube, picks it up, and then moves back to the inner grid to score it.
  private Command I_4_I(){
    List<PathPlannerTrajectory> path = PathPlanner.loadPathGroup("I-4-I", new PathConstraints(2.5,2.5),
                                                                                new PathConstraints(2.5,2.5));
    return new SequentialCommandGroup(
      new ParallelDeadlineGroup(
        new InstantCommand(() -> endEffector.resetWristAngle()),
        new SequentialCommandGroup(
          drivetrain.followTrajectory(path.get(0)),
          new WaitCommand(2)
        ),
        new ProxyCommand(() -> arm.moveTo(ArmConstants.pickup)),
        new InstantCommand(() -> endEffector.intakeCube()),
        new ProxyCommand(endEffector.turnWristToAngle(150))
      ),
      new InstantCommand(() -> endEffector.stop()),
      new ParallelDeadlineGroup(
        new WaitCommand(4),
        drivetrain.followTrajectory(path.get(1)),
        new SequentialCommandGroup(
          new WaitCommand(0.5),
          new ProxyCommand(() -> arm.moveTo(ArmConstants.midLevel))
        )
      ),
      //new AutoMoveToAprilTag(drivetrain, drivetrain.getCamera()).withTimeout(1),
      new InstantCommand(() -> endEffector.releaseCube()),
      new WaitCommand(0.5),
      new InstantCommand(() -> endEffector.stop())
    );
  }
  // Moves from the inner grid, goes around the side, and then moves up onto the charge station to auto balance
  private Command I_Charge(){
    PathPlannerTrajectory path = PathPlanner.loadPath("I-Charge", new PathConstraints(1,1));
    return new SequentialCommandGroup(
      drivetrain.followTrajectory(path)
    );
  }
  // Moves from outer grid, to the first cube, picks it up, and then moves back to the outer grid to score it.
  private Command O_1_O(){
    //PathPlannerTrajectory path = PathPlanner.loadPath("O-1-O", new PathConstraints(1,1));
    List<PathPlannerTrajectory> path = PathPlanner.loadPathGroup("O-1-O", new PathConstraints(2,2), new PathConstraints(2,2));
    return new SequentialCommandGroup(
      new ParallelDeadlineGroup(
        new SequentialCommandGroup(
          new InstantCommand(() -> endEffector.resetWristAngle()),
          drivetrain.followTrajectory(path.get(0)),
          new WaitCommand(2)
        ),
        new SequentialCommandGroup(
          new WaitCommand(1),
          new InstantCommand(() -> endEffector.intakeCube()),
          new ProxyCommand(() -> endEffector.turnWristToAngle(130)).withTimeout(2)
        ),
        new SequentialCommandGroup(
          new ProxyCommand(() -> arm.moveTo(ArmConstants.pickup))
        )
      ),
      new ParallelDeadlineGroup(
        new WaitCommand(5),
        drivetrain.followTrajectory(path.get(1)),
        new SequentialCommandGroup(
          new WaitCommand(1),
          new InstantCommand(() -> endEffector.stop()),
          new ProxyCommand(() -> arm.moveTo(ArmConstants.midLevel))
        )
      ),
      //new ProxyCommand(() -> new AutoMoveToAprilTag(drivetrain, drivetrain.getCamera()).withTimeout(1)),
      new InstantCommand(() -> endEffector.releaseCube()),
      new WaitCommand(0.5),
      new InstantCommand(() -> endEffector.stop())
    );
  }
  // Moves from outer grid, to the second cube, picks it up, and then moves back to the outer grid to score it.
  private Command O_2_O(){
    PathPlannerTrajectory path = PathPlanner.loadPath("O-2-O", new PathConstraints(1,1));
    return new SequentialCommandGroup(
      drivetrain.followTrajectory(path)
    );
  }
  // Moves from the outer grid, goes around the side, then goes onto the charge station to auto balance
  private Command O_Charge(){
    PathPlannerTrajectory path = PathPlanner.loadPath("O_Charge", new PathConstraints(1,1));
    return new SequentialCommandGroup(
      drivetrain.followTrajectory(path)
    );
  }

  public void stopSubsystems(){
    endEffector.stop();
    drivetrain.stop();
  }

  public void resetDriver(){
    drivetrain.setHeading180();
  }
}
