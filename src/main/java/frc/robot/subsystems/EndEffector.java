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
  private final CANSparkMax neo = new CANSparkMax(Constants.EndEffectorConstants.neoID, MotorType.kBrushless); //check if neo is brushed
  private final CANSparkMax snowblower = new CANSparkMax(Constants.EndEffectorConstants.snowblowerID, MotorType.kBrushed);

  private final RelativeEncoder snowblowerEncoder = snowblower.getEncoder(Type.kQuadrature, 8192);

  /** Creates a new EndEffector. */
  public EndEffector() {
    // Reset the snowblower encoder
    snowblower.restoreFactoryDefaults();
    neo.restoreFactoryDefaults();
    neo.setSmartCurrentLimit(25);//limit is set to 10 amps, no idea if this is good or not
    neo.setSecondaryCurrentLimit(25);
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
   * @param speed  - a double between 0 and 1, representing the speed that we want the wrist to turn at
   */
  public void turnWrist(double speed){
    if(snowblowerEncoder.getPosition() >= 90 && speed > 0){
      speed = 0;
    }else if(snowblowerEncoder.getPosition() <= -90 && speed < 0){
      speed = 0;
    }
    snowblower.set(speed);
  }

  /**
   * Automatically turns the wrist to a specified angle.
   * @param goalAngle - input angle that the wrist should turn to
   * @return a PIDCommand for usage with controllers and auto routines
   */
  public PIDCommand turnWristToAngle(double goalAngle){
    double currentAngle = snowblowerEncoder.getPosition();
    double error = goalAngle - currentAngle;
    PIDController turnWristController = new PIDController(0.05, 0, 0);
    return new PIDCommand(turnWristController, () -> snowblowerEncoder.getPosition(), goalAngle, output -> turnWrist(output), this);
  }

  /**
   * More complex command that should automatically intake a cone based on input orientation
   * @param isForward - true if cone is facing forward (point is away from robot), false if reversed
   * @return SeqeuntialCommandGroup for usage with controllers and auto routines
   */
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
  /**
   * If the cone is pointed forwards (and wrist pointed backwards), this command flips the wrist forwards
   * to put the cone upright. Otherwise, it flips the wrist backwards to put a reversed cone upright.
   * @return ProxyCommand
   */
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

  /**
   * If the wrist is pointed forwards, it reverses the intake to release a cone,
   * and if the wrist is pointed backwards, it runs the intake forwards to release.
   * @return InstantCommand
   */
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

  /**
   * Runs the intake forwards to intake a cube
   */
  public void intakeCube() {
    // blah blah blah
    runIntake(0.5); //assumption is being made that a positive value intakes and a negative value spits out
  }

  /**
   * Runs the intake backwards to release a cube
   */
  public void releaseCube() {
    // Code code code
    runIntake(-0.5); //assumption is being made that a positive value intakes and a negative value spits out
    //speed may need to be lowered to output more safely
  }

  /**
   * Stops all motors on the end effector.
   */
  public void stop(){
    runIntake(0);
    turnWrist(0);
  }
}
