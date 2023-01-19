// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveDrive;

public class AutoBalance extends CommandBase {

  double roll; //corresponds to x movement for some reason on the practice bot
  double pitch; //corresponds to y movement on the practice bot
  double yaw;
  double goal = 0;
  double xError;
  double yError;
  double averageError;
  double kP = 0.04; //value that turns degrees of error into speed for swerve drive - needs to be tested
  double xSpeed;
  double ySpeed;
  boolean isDone = false;
  int balancedCounter = 0;

  private SwerveDrive swerve;
  /** Creates a new AutoBalance. */
  public AutoBalance(SwerveDrive theSwerve) {
    // Use addRequirements() here to declare subsystem dependencies.
    swerve = theSwerve;
    addRequirements(swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    balancedCounter = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    roll = swerve.getRoll();
    xError = goal - roll;
    pitch = swerve.getPitch();
    yError = goal - pitch;
    yaw = swerve.getYawDegrees();
    averageError = (Math.cos(yaw) * xError + Math.sin(yaw) * yError);
    if(averageError > 2){
      xSpeed = kP * xError;
      ySpeed = kP * yError;
      swerve.drive(xSpeed, ySpeed, 0, false);
      //for now this just uses speed in the x direction - we can make some tweaksif need be to 
      //adjust in both the x and y directions.
      balancedCounter = 0;
    }if(averageError < -2){
      xSpeed = kP * xError;
      ySpeed = kP * yError;
      swerve.drive(xSpeed, ySpeed, 0, false);
    }else{
      balancedCounter++;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    swerve.defenseMode();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(balancedCounter >= 50){
      return true;
    }else{
      return false;
    }
  }
}
