// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;

public class Arm extends SubsystemBase {

  private final WPI_TalonFX leftArmMotor;
  private final WPI_TalonFX rightArmMotor;
  private final MotorControllerGroup armMotors;

  private final DutyCycleEncoder camEncoder;

  /** Creates a new Arm. */
  public Arm() {
    leftArmMotor = new WPI_TalonFX(ArmConstants.leftArmID);
    rightArmMotor = new WPI_TalonFX(ArmConstants.rightArmID);

    camEncoder = new DutyCycleEncoder(ArmConstants.camEncoderDIO);

    leftArmMotor.configFactoryDefault();
    rightArmMotor.configFactoryDefault();

    leftArmMotor.setInverted(true); //inverting the left side motor for now - may need to be switched later
    rightArmMotor.setInverted(false);

    armMotors = new MotorControllerGroup(leftArmMotor, rightArmMotor);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  //for the joysticks/constant movement
  /**
   * 
   * @param volts voltage setting for both motors, generally to be provided by the moveTo method.
   */
  public void move(double volts){
    //leftArmMotor.set(speed);
    //rightArmMotor.set(speed);
    armMotors.setVoltage(volts);
  }

  public void resetEncoder(){
    camEncoder.reset();
  }

  /**
   * 
   * @return the angle of the arm in degrees based on the crank angle
   */
  public double getArmAngle(){

    double A = ArmConstants.X3 - ArmConstants.R1*Math.cos(Math.toRadians(getCrankAngle()));
    double B = ArmConstants.Y3 - ArmConstants.R1*Math.sin(Math.toRadians(getCrankAngle()));

    return Math.toDegrees(Math.asin((Math.pow(ArmConstants.R2,2)-Math.pow(ArmConstants.R3,2)-(A*A+B*B))/(2*ArmConstants.R3*Math.sqrt(A*A+B*B))))
          - Math.toDegrees(Math.atan(A/B))
          - ArmConstants.PHI;
  }

  /**
   * 
   * @return the angle of the crank in degrees (0 deg. being horizontal forward)
   */
  private double getCrankAngle(){
    return camEncoder.getAbsolutePosition(); //Not sure if this is returning a 0-360 degrees or a 0-1 value.
  }

  /**
   * 
   * A reverse kinimatics method based on the arm geometry.  All angles are with respect 
   * to 0 degrees being considered horizontal forward and proceed CCW looking at the RH side
   * 
   * @param armAngle  the angle of the arm (degrees) for which the corresponding crank angle is desired
   * @return the crank angle corresponding to the given arm angle (degrees)
   */
  private double armToCrank(double armAngle){
    
    double C = ArmConstants.X3+ArmConstants.R3*Math.cos(Math.toRadians(armAngle+ArmConstants.PHI));
    double D = ArmConstants.Y3+ArmConstants.R3*Math.sin(Math.toRadians(armAngle+ArmConstants.PHI));

    double theAngle = Math.toDegrees(Math.asin((Math.pow(ArmConstants.R1,2)-Math.pow(ArmConstants.R2,2)+C*C+D*D))/(2*ArmConstants.R1*Math.sqrt(C*C+D*D)))
          - Math.toDegrees(Math.atan(C/D))
          + 360;
    
    if (theAngle > ArmConstants.maxCrankAngle) {
      theAngle = ArmConstants.maxCrankAngle - (theAngle - ArmConstants.maxCrankAngle); 
    }

    return theAngle;

  }

  /**
   * 
   * @return The angle og the connecting rod in degrees
   */
  private double getRodAngle() {

    double X1 = ArmConstants.R1*Math.cos(Math.toRadians(getCrankAngle()));
    double Y1 = ArmConstants.R1*Math.sin(Math.toRadians(getCrankAngle()));

    double X2 = ArmConstants.X3+ArmConstants.R3*Math.cos(Math.toRadians(getArmAngle()+ArmConstants.PHI));
    double Y2 = ArmConstants.Y3+ArmConstants.R3*Math.sin(Math.toRadians(getArmAngle()+ArmConstants.PHI));

    return Math.toDegrees(Math.atan((Y2-Y1)/(X2-X1)));
  }

  /**
   * 
   * This method should use a trapezoid motion profile to govern the arm angle 
   * from the current position to the goal angle.  Use the armToCrank method to 
   * determine the crank angle at each time step, then use a proportional gain and a 
   * state based feedforward gain that accounts for the position of the arm.
   * 
   * @param goalAngle The desired arm angle
   */
  public void moveTo(double goalAngle){

    /** 
    double currentAngle = encoderAngle();
    //function to turn camAngle into armAngle and then motorpower - need to look at CAD and get measurements to do mathematically
    double currentArmAngle = -1 * Math.sin(currentAngle) + Math.PI; //very rough transformation - tried to do the actual math, graphed that, this comes pretty close
    currentArmAngle = currentArmAngle * 180 / Math.PI;
    double error = goalAngle - currentArmAngle;
    move(error * 0.05); //random proportional value to test
    //need to test what angles accurately represent each height
    */
  }


  /* DP: Dont think we need these 
  public double encoderToArmAngle(double encoderAngle){
    return (-1 * Math.sin(encoderAngle) + Math.PI * 180)/Math.PI;
  }

  public double getHeight(){
    return Math.sin(encoderToArmAngle(encoderAngle()))*48;//48 is arbitrary length of arm, need to check for accuracy
  }

  */
}
