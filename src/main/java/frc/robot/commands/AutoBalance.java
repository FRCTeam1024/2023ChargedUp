// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveDrive;

public class AutoBalance extends CommandBase {

  double currentAngle;
  double goal = 0;
  double error;
  double kP = 0.005; //value that turns degrees of error into speed for swerve drive - needs to be tested
  double speed;
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
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    currentAngle = swerve.getPitch();
    error = goal - currentAngle;
    if(error < 0.1){
      speed = kP * error;
      swerve.drive(speed, 0, 0, true); 
      //for now this just uses speed in the x direction - we can make some tweaksif need be to 
      //adjust in both the x and y directions.
      balancedCounter++;
    }else{
      if(balancedCounter >= 10){
        isDone = true;
      }
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
    return isDone;
  }
}
