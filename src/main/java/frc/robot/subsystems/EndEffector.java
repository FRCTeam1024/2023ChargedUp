// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class EndEffector extends SubsystemBase {
  private final Spark neo = new Spark(Constants.EndEffectorConstants.neoID);
  private final Spark snowblower = new Spark(Constants.EndEffectorConstants.snowblowerID);

  private final CANCoder snowblowerEncoder = new CANCoder(Constants.EndEffectorConstants.snowblowerID);

  /** Creates a new EndEffector. */
  public EndEffector() {
    // Reset the snowblower encoder
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    // If snowblowerSpeed > 0 and encoder >= maxAngle
      // brake intake
    // else if snowblowerSpeed < 0 and encoder <= minAngle
      // brake intake


    // If intakeSpeed > 0 and current is too high
      // brake intake
  }

  public void runIntake(double speed) {
    neo.set(speed);
  }

  public void turnWrist(double speed){
    if(snowblowerEncoder.getAbsolutePosition() >= 90 && speed > 0){
      speed = 0;
    }else if(snowblowerEncoder.getAbsolutePosition() <= -90 && speed < 0){
      speed = 0;
    }
    snowblower.set(speed);
  }

  public PIDCommand turnWristToAngle(double goalAngle){
    double currentAngle = snowblowerEncoder.getAbsolutePosition();
    double error = goalAngle - currentAngle;
    PIDController turnWristController = new PIDController(0.05, 0, 0);
    return new PIDCommand(turnWristController, () -> snowblowerEncoder.getAbsolutePosition(), goalAngle, output -> turnWrist(output), this);
  }

  public void intakeCone(boolean isForward) {
    // if isForward
      // Set snowblower (wrist) to angle
      // Run intake proper direction
      if(isForward){
        turnWristToAngle(-90);
        runIntake(0.5);
      }else{
        turnWristToAngle(90);
        runIntake(-0.5);
      }
    // else
      // Set snowblower (wrist) to angle
      // Run intake proper direction
  }

  public void flipCone() {
    //words words words
    if(snowblowerEncoder.getAbsolutePosition() < -10){
      turnWristToAngle(10);
    }else if(snowblowerEncoder.getAbsolutePosition() > 10){
      turnWristToAngle(-10);
    }
  }

  public void releaseCone() {
    // stuff stuff stuff
    if(snowblowerEncoder.getAbsolutePosition() > 5){
      runIntake(-0.5);
    }else if(snowblowerEncoder.getAbsolutePosition() < 5){
      runIntake(0.5);
    }
  }

  public void intakeCube() {
    // blah blah blah
    runIntake(0.5); //assumption is being made that a positive value intakes and a negative value spits out
  }

  public void releaseCube() {
    // Code code code
    runIntake(-0.5); //assumption is being made that a positive value intakes and a negative value spits out
    //speed may need to be lowered to output more safely
  }
}
