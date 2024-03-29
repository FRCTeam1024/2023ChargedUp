// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

import java.io.File;
import java.io.IOException;
import java.nio.file.Path;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.server.PathPlannerServer;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private static File deployfile = Filesystem.getDeployDirectory();
  private static File pathfile = new File(deployfile, "pathplanner/");
  static String fileList[] = pathfile.list();
  static PathPlannerTrajectory pathList[] = new PathPlannerTrajectory[fileList.length];

  private RobotContainer m_robotContainer;

  private final DigitalInput otherBotJumper = new DigitalInput(8);
  private static boolean compBotState;
  private static boolean otherBotState;
  private final DigitalInput practiceBotJumper = new DigitalInput(9);
  private static boolean practiceBotState;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {

    //Display and log the name and version of the code that is running
    //System.out.println("Running "+BuildConfig.APP_NAME+" "+BuildConfig.APP_VERSION);

    //Pathweaver code from 2022 - may not be needed for pathplanner
    /* 
    for(int i = 0; i < fileList.length; i++) {
      //try {
        Path thePath = Filesystem.getDeployDirectory().toPath().resolve("pathplanner/"+fileList[i]);
        pathList[i] = PathPlanner.loadPath(fileList[i].substring(0,fileList[i].length()-5), 0.5, 3);
        //System.out.println(pathList[i].toString());
      //} catch (IOException ex) {
        //DriverStation.reportError("Unable to open trajectory: " + fileList[i], ex.getStackTrace());
      //}
    }
    */

    // Check whether the current robot is the competition robot or the practice robot:
    // FALSE from the DIO means the jumper is present.
    if(otherBotJumper.get() == false) {
      otherBotState = true;
    } else {
      otherBotState = false;
    }
    if(practiceBotJumper.get() == false){
      practiceBotState = true;
    } else {
      practiceBotState = false;
    }
    if(!otherBotState && !practiceBotState){
      compBotState = true;
    }else if(otherBotState && practiceBotState){
      System.out.println("\n\n\n\n\n ROBOT IS IN QUANTUM SUPERPOSITION STATE: BOTH PRACTICE AND COMP BOT\n   INITIALIZING TO COMP BOT STATE\n\n\n\n\n");
      practiceBotState = false;
      otherBotState = false;
      compBotState = true;
    }else{
      compBotState = false;
    }

    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
    m_robotContainer.resetDriver();
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
        //Call this to diable any PIDSubsytems to avoid integral windup.
        m_robotContainer.disablePIDSubsystems();
        m_robotContainer.stopSubsystems();
  }

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    m_robotContainer.resetDriver();
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }

    //m_robotContainer.resetDriver();
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {}

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}

  /**
  * 
  * @return TRUE if the current robot is the competition robot. Otherwise, false for practice bot.
  */
  public static boolean isCompBot() {
    return compBotState;
  }

  public static boolean isPracticeBot(){
    return practiceBotState;
  }

  public static boolean isOtherBot(){
    return otherBotState;
  }
}
