// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;

public class Arm extends SubsystemBase {

  private final WPI_TalonFX leftArmMotor;
  private final WPI_TalonFX rightArmMotor;

  private final CANCoder camEncoder;

  /** Creates a new Arm. */
  public Arm() {
    leftArmMotor = new WPI_TalonFX(ArmConstants.leftArmID);
    rightArmMotor = new WPI_TalonFX(ArmConstants.rightArmID);

    camEncoder = new CANCoder(ArmConstants.camEncoderID);

    leftArmMotor.configFactoryDefault();
    rightArmMotor.configFactoryDefault();
    camEncoder.configFactoryDefault();

    leftArmMotor.setInverted(true); //inverting the left side motor for now - may need to be switched later
    rightArmMotor.setInverted(false);

    //rightArmMotor.follow(leftArmMotor);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  //for the joysticks/constant movement
  public void move(double speed){
    leftArmMotor.set(speed);
    rightArmMotor.set(speed);
  }

  public void resetEncoder(){
    camEncoder.setPosition(0);
  }

  public double encoderAngle(){
    return camEncoder.getPosition();
  }

  public void moveTo(double goalAngle){
    double currentAngle = encoderAngle();
    double error = goalAngle - currentAngle;
    //function to turn camAngle into armAngle and then motorpower - need to look at CAD and get measurements to do mathematically
    

    move(error);
  }
}
