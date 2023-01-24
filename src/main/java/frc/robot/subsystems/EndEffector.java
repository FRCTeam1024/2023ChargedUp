// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
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

  public void intakeCone(boolean isForward) {
    // if isForward
      // Set snowblower (wrist) to angle
      // Run intake proper direction
    // else
      // Set snowblower (wrist) to angle
      // Run intake proper direction
  }

  public void flipCone() {

  }

  public void releaseCone() {
    
  }

  public void intakeCube() {
    // blah blah blah
  }

  public void releaseCube() {
    // 
  }
}
