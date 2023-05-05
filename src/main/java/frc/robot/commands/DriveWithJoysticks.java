// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.oi.Logitech;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.SwerveDrive;
import edu.wpi.first.wpilibj.Timer;

public class DriveWithJoysticks extends CommandBase {
  /** Creates a new DriveWithJoysticks. */
  private Logitech controller;
  private SwerveDrive drivetrain;
  private double speedFactor;
  private Arm arm;
  private double goal;
  private double yaw;

  private double xLockDelay;
  private Timer idleTimer;

  ProfiledPIDController robotTurn = new ProfiledPIDController(0.1, 
                                                              0, 
                                                              0,
                                                              new TrapezoidProfile.Constraints(
                                                                  360, 180));
  public DriveWithJoysticks(SwerveDrive m_swerve, Logitech controllerParam, double speed, Arm theArm) {
    // Use addRequirements() here to declare subsystem dependencies.
    drivetrain = m_swerve;
    controller = controllerParam;
    speedFactor = speed;
    arm = theArm;
    yaw = drivetrain.getYawDegrees();
    goal = yaw;
    xLockDelay = 0.5;
    idleTimer = new Timer();
    robotTurn.enableContinuousInput(-180, 180);
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    idleTimer.reset();
    idleTimer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    // Get Translation and rotation values from the controller
    double xSpeed = -controller.getRightStickY() * DriveConstants.kMaxWheelSpeedMetersPerSecond * speedFactor; //these were negated, changing negation because robot starts at 180
    double ySpeed = -controller.getRightStickX() * DriveConstants.kMaxWheelSpeedMetersPerSecond * speedFactor; //also changed negation
    double rot = controller.getLeftStickX() * DriveConstants.kMaxAngularSpeedRadiansPerSecond * speedFactor;
    
    // Get yaw info from gyro
    yaw = drivetrain.getYawDegrees();

    // If a dpad is selected, override joystick rotation value to hold the dpad direction
    if(controller.dPadUp.getAsBoolean()){
      goal = 0;
      rot = robotTurn.calculate(drivetrain.getYawDegrees(),goal);
    }
    else if(controller.dPadDown.getAsBoolean()){
      goal = 180;
      rot = robotTurn.calculate(drivetrain.getYawDegrees(),goal);
    }
    else if(controller.dPadLeft.getAsBoolean()){
      goal = 90;
      rot = robotTurn.calculate(drivetrain.getYawDegrees(),goal);
    }
    else if(controller.dPadRight.getAsBoolean()){
      goal = -90;
      rot = robotTurn.calculate(drivetrain.getYawDegrees(),goal);
    }
    else {
      robotTurn.reset(drivetrain.getYawDegrees());
    }

    // Limit rotation speed if the arm us up
    if(arm.getArmAngle() >= -60){
      rot *= 0.5;
    }else if(arm.getArmAngle() < -60){
      rot *= 1;
    }

    // Reset the idleTimer if any movement is commanded
    if( xSpeed !=0 || ySpeed != 0 || rot != 0) {
      idleTimer.reset();
    }

    // If we have not been idle and are not overriding xLock then issue a drive command, otherwise xLock
    if( idleTimer.get() < xLockDelay || controller.rightBumper.getAsBoolean()){// || arm.getAngle() >= -60  -> if we decide to stop x-lock based on arm angle
      drivetrain.drive(xSpeed, ySpeed, rot, !controller.rightTrigger.getAsBoolean());
    }
    else {
      drivetrain.defenseMode();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    idleTimer.reset();
    idleTimer.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
