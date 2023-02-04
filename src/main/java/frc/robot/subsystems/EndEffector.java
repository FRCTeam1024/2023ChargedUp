// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.SparkMaxRelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxRelativeEncoder.Type;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj2.command.ProxyCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class EndEffector extends SubsystemBase {
  private final CANSparkMax neo = new CANSparkMax(Constants.EndEffectorConstants.neoID, MotorType.kBrushed); //check if neo is brushed
  private final CANSparkMax snowblower = new CANSparkMax(Constants.EndEffectorConstants.snowblowerID, MotorType.kBrushed);

  private final RelativeEncoder snowblowerEncoder = snowblower.getEncoder(Type.kQuadrature, 8192);

  /** Creates a new EndEffector. */
  public EndEffector() {
    // Reset the snowblower encoder
    snowblower.restoreFactoryDefaults();
    snowblowerEncoder.setPositionConversionFactor(360);
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

  public double getWristAngle(){
    return snowblowerEncoder.getPosition();
  }

  public void runIntake(double speed) {
    neo.set(speed);
  }

  public void turnWrist(double speed){
    if(snowblowerEncoder.getPosition() >= 90 && speed > 0){
      speed = 0;
    }else if(snowblowerEncoder.getPosition() <= -90 && speed < 0){
      speed = 0;
    }
    snowblower.set(speed);
  }

  public PIDCommand turnWristToAngle(double goalAngle){
    double currentAngle = snowblowerEncoder.getPosition();
    double error = goalAngle - currentAngle;
    PIDController turnWristController = new PIDController(0.05, 0, 0);
    return new PIDCommand(turnWristController, () -> snowblowerEncoder.getPosition(), goalAngle, output -> turnWrist(output), this);
  }

  public SequentialCommandGroup intakeCone(boolean isForward) {
    // if isForward
      // Set snowblower (wrist) to angle
      // Run intake proper direction
      if(isForward){
        return new SequentialCommandGroup(
          turnWristToAngle(-90),
          new InstantCommand(() -> runIntake(0.5))
        );
      }else{
        return new SequentialCommandGroup(
          turnWristToAngle(90),
          new InstantCommand(() -> runIntake(-0.5))
        );
      }
    // else
      // Set snowblower (wrist) to angle
      // Run intake proper direction
  }

  public ProxyCommand flipCone() {
    //words words words
    if(snowblowerEncoder.getPosition() < -10){
      return new ProxyCommand(() -> turnWristToAngle(10));
    }else if(snowblowerEncoder.getPosition() > 10){
      return new ProxyCommand(() -> turnWristToAngle(-10));
    }else {
      return new ProxyCommand(() -> turnWristToAngle(snowblowerEncoder.getPosition()));
    }
  }

  public InstantCommand releaseCone() {
    // stuff stuff stuff
    if(snowblowerEncoder.getPosition() > 5){
      return new InstantCommand(() -> runIntake(-0.5));
    }else if(snowblowerEncoder.getPosition() < -5){
      return new InstantCommand(() -> runIntake(0.5));
    }else{
      return new InstantCommand(() -> runIntake(0));
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

  public void stop(){
    runIntake(0);
    turnWrist(0);
  }
}
