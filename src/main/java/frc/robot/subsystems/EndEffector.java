// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.SparkMaxRelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxRelativeEncoder.Type;
import com.revrobotics.CANSparkMax.FaultID;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj2.command.ProxyCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.EndEffectorConstants;

/**
 * Check if switch is set correctly on encoder
 * test current code with fudge factor to keep encoder in range.
 * Maybe add periodic thing that checks if encoder has gone past 0 to overflow, and then autocorrects itself.
 */

public class EndEffector extends SubsystemBase {
  private final CANSparkMax neo = new CANSparkMax(Constants.EndEffectorConstants.neoID, MotorType.kBrushless); //check if neo is brushed
  private final CANSparkMax snowblower = new CANSparkMax(Constants.EndEffectorConstants.snowblowerID, MotorType.kBrushed);


  //private final SparkMaxAbsoluteEncoder absoluteEncoder = snowblower.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle);
  private final RelativeEncoder snowblowerEncoder = snowblower.getEncoder(Type.kQuadrature, 8192);
  //private final RelativeEncoder neoEncoder = neo.getEncoder();

  /** Creates a new EndEffector. */
  public EndEffector() {
    // Reset the snowblower encoder
    snowblower.restoreFactoryDefaults();
    neo.restoreFactoryDefaults();
    neo.setSmartCurrentLimit(30);//limit is set to 10 amps, no idea if this is good or not
    neo.setSecondaryCurrentLimit(30);
    snowblower.setSmartCurrentLimit(5);
    snowblower.setSecondaryCurrentLimit(5);
    double conversionFactor = 360/5.333;
    snowblowerEncoder.setPositionConversionFactor(EndEffectorConstants.wristConversionFactor);
    snowblowerEncoder.setPosition(EndEffectorConstants.wristStart);
    //neo.burnFlash();
    //snowblower.burnFlash();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    /**if(snowblowerEncoder.getPosition() >= 6){
      snowblowerEncoder.setPosition(2.667);
    }*/
    // If snowblowerSpeed > 0 and encoder >= maxAngle
      // brake intake
    // else if snowblowerSpeed < 0 and encoder <= minAngle
      // brake intake


    // If intakeSpeed > 0 and current is too high
      // brake intake
    
  }


  public double getWristAngle(){
    //getPosition() returns rotations of motor, which is identical to rotations of encoder since mirrored gearing
    //Rotations of motor * gear ratio (1/2.667) = rotations of wrist
    //rotations of wrist * 360 = degrees

    /**
     * Encoder count seems to max out at 4,500,000
     * 4500000 / 8192 = 549.31640625
     * Hence the encoder maxes out number of revolutions at 549.31640625
     * If set the initial position to 2.667, and then subtract 2.667 from all positions for the return number, 
     * we can get numbers between (2.667,-2.667), so we divide by 2.667 to get from rotations of the motor to rotations of wrist
     */
    /**if(angle > 360){
    return angle%360;
    }else if(angle < 360 && angle > 180){
    }else if(angle < 180 && angle > -180){
      return angle;
    }else if(angle < -180){
    }*/
    return (snowblowerEncoder.getPosition() - EndEffectorConstants.angleBuffer);
  }

  public double getRawWristAngle(){
    return snowblowerEncoder.getPosition();
  }

  /**public double getConsolidatedAngle(){
    //On boot up, relative encoder will read its current position as 0. (Double check this)
    //Relative encoder reading should be added to absolute encoder reading in order to get a more accurate measurement
    return (getWristAngle() + getAbsoluteAngle()); //Maybe, idk exactly what the relative encoder is actually doing
    //return ((getWristAngle() + getAbsoluteAngle())%180);
  }*/

  /**public double getAbsoluteAngle(){
    return absoluteEncoder.getPosition(); //check what exactly this returns - rotations vs degrees
  }*/

  /**
   * Sets intake motor to a specific voltage
   * @param speed - a double between 0 and 1, representing the speed that we want the motor to run at
   */
  public void runIntake(double speed) {
    double voltage = speed * 12;
    neo.setVoltage(voltage);
  }
  /**
   * Turns the wrist mechanism at the input speed, but ideally will automatically stop it if the wrist has moved beyond its limits.
   * @param speed - a double between 0 and 1, representing the speed that we want the wrist to turn at
   */
  public void turnWrist(double speed){
    /**if(snowblowerEncoder.getPosition() >= 120 && speed > 0){ //double check that angle and speed match up correctly
      speed = 0;
    }else if(snowblowerEncoder.getPosition() <= -120 && speed < 0){
      speed = 0;
    }*/
    snowblower.set(speed);
  }

  public void turnWristSetpoints(double speed){
    snowblower.set(-speed);
  }

  /**
   * Automatically turns the wrist to a specified angle.
   * @param goalAngle - input angle that the wrist should turn to
   * @return a PIDCommand for usage with controllers and auto routines
   */
  public PIDCommand turnWristToAngle(double goalAngle){
    double currentAngle = getWristAngle();
    double error = goalAngle - currentAngle;
    PIDController turnWristController = new PIDController(0.05, 0, 0);
    return new PIDCommand(turnWristController, () -> getWristAngle(), goalAngle, output -> turnWristSetpoints(output), this);
  }

  /**
   * More complex command that should automatically intake a cone based on input orientation
   * @param isForward - true if cone is facing forward (point is away from robot), false if reversed
   * @return SeqeuntialCommandGroup for usage with controllers and auto routines
   */
  public SequentialCommandGroup intakeCone(boolean isForward) {
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
  }
  /**
   * If the cone is pointed forwards (and wrist pointed backwards), this command flips the wrist forwards
   * to put the cone upright. Otherwise, it flips the wrist backwards to put a reversed cone upright.
   * @return ProxyCommand
   */
  public ProxyCommand flipCone() {
    if(snowblowerEncoder.getPosition() < -10){
      return new ProxyCommand(() -> turnWristToAngle(10));
    }else if(snowblowerEncoder.getPosition() > 10){
      return new ProxyCommand(() -> turnWristToAngle(-10));
    }else {
      return new ProxyCommand(() -> turnWristToAngle(snowblowerEncoder.getPosition()));
    }
  }

  /**
   * If the wrist is pointed forwards, it reverses the intake to release a cone,
   * and if the wrist is pointed backwards, it runs the intake forwards to release.
   * @return InstantCommand
   */
  public InstantCommand releaseCone() {
    if(snowblowerEncoder.getPosition() > 5){
      return new InstantCommand(() -> runIntake(-0.5));
    }else if(snowblowerEncoder.getPosition() < -5){
      return new InstantCommand(() -> runIntake(0.5));
    }else{
      return new InstantCommand(() -> runIntake(0));
    }
  }

  /**
   * Runs the intake forwards to intake a cube
   */
  public void intakeCube() {
    runIntake(0.6); //assumption is being made that a positive value intakes and a negative value spits out
  }

  /**
   * Runs the intake backwards to release a cube
   */
  public void releaseCube() {
    runIntake(-0.6); //assumption is being made that a positive value intakes and a negative value spits out
  }

  /**
   * Stops all motors on the end effector.
   */
  public void stop(){
    runIntake(0);
    turnWrist(0);
  }

  public void resetWristAngle(){
    snowblowerEncoder.setPosition(EndEffectorConstants.wristStart);
  }

  public double getTemperature(){
    return neo.getMotorTemperature();
  }

  public boolean intakeStalled(){
    return neo.getFault(FaultID.kStall);
  }

  public boolean intakeOvercurrent(){
    return neo.getFault(FaultID.kOvercurrent);
  }

  public double getNeoVoltage(){
    return neo.getBusVoltage();
  }
}
