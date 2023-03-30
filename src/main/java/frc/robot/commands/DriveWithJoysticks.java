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

public class DriveWithJoysticks extends CommandBase {
  /** Creates a new DriveWithJoysticks. */
  Logitech controller;
  SwerveDrive drivetrain;
  double speedFactor;
  Arm arm;
  double goal;
  double yaw;
  double autoSpeed;
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
    autoSpeed = 0;
    robotTurn.enableContinuousInput(-180, 180);
    //robotTurn.setTolerance(3);
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double xSpeed = -controller.getRightStickY() * DriveConstants.kMaxWheelSpeedMetersPerSecond * speedFactor; //these were negated, changing negation because robot starts at 180
    double ySpeed = -controller.getRightStickX() * DriveConstants.kMaxWheelSpeedMetersPerSecond * speedFactor; //also changed negation
    double rot = controller.getLeftStickX() * DriveConstants.kMaxAngularSpeedRadiansPerSecond * speedFactor;
    yaw = drivetrain.getYawDegrees();
    if(arm.getArmAngle() >= -60){
      rot *= 0.5;
    }else if(arm.getArmAngle() < -60){
      rot *= 1;
    }

    if(controller.dPadUp.getAsBoolean()){
      goal = 0;
      rot = robotTurn.calculate(drivetrain.getYawDegrees(),goal);
     // System.out.println(rot);
    }
    if(controller.dPadDown.getAsBoolean()){
      goal = 180;
      rot = robotTurn.calculate(drivetrain.getYawDegrees(),goal);
      //System.out.println(rot);
    }
    if(controller.dPadLeft.getAsBoolean()){
      goal = 90;
      rot = robotTurn.calculate(drivetrain.getYawDegrees(),goal);
     // System.out.println(rot);
    }
    if(controller.dPadRight.getAsBoolean()){
      goal = -90;
      rot = robotTurn.calculate(drivetrain.getYawDegrees(),goal);
      //System.out.println(rot);
    }
    if(!controller.dPadUp.getAsBoolean() && !controller.dPadDown.getAsBoolean() && !controller.dPadLeft.getAsBoolean() && !controller.dPadRight.getAsBoolean()){
      robotTurn.reset(drivetrain.getYawDegrees());
    }
    if( xSpeed != 0 || ySpeed != 0 || rot  != 0 || controller.rightBumper.getAsBoolean()){// || arm.getAngle() >= -60  -> if we decide to stop x-lock based on arm angle
      drivetrain.drive(xSpeed, ySpeed, rot, !controller.rightTrigger.getAsBoolean());
    }
    else {drivetrain.defenseMode();}
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
