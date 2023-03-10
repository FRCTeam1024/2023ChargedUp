// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants.ArmConstants;


public class Arm extends SubsystemBase {

  private final UsbCamera armCamera;

  private final WPI_TalonFX leftArmMotor;
  private final WPI_TalonFX rightArmMotor;
  private final MotorControllerGroup armMotors;

  private final DigitalInput armLimit;

  private State goal = new State(-1,-1);
  private double voltage = -1;
  /** Creates a new Arm. */
  public Arm() {
    leftArmMotor = new WPI_TalonFX(ArmConstants.leftArmID);
    rightArmMotor = new WPI_TalonFX(ArmConstants.rightArmID);

    armLimit = new DigitalInput(ArmConstants.camLimitDIO);

    leftArmMotor.configFactoryDefault();
    rightArmMotor.configFactoryDefault();

    leftArmMotor.setInverted(false); //inverting the left side motor for now - may need to be switched later
    rightArmMotor.setInverted(true);

    leftArmMotor.setNeutralMode(NeutralMode.Brake);
    rightArmMotor.setNeutralMode(NeutralMode.Brake);

    armMotors = new MotorControllerGroup(leftArmMotor, rightArmMotor);
    rightArmMotor.setSelectedSensorPosition(0);

    armCamera = CameraServer.startAutomaticCapture();
    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  //for the joysticks/constant movement
  /**
   * 
   * @param volts voltage setting for both motors, generally to be provided by the moveTo method.
   * @param setpoint the setpoint from the profiledPID command
   */
  public void move(double volts, State setpoint){
    
    //Store the setpoint for future reference
    goal = setpoint;

    //point the static gain in the direction that opposes gravity
    double staticGain = ArmConstants.kS;
    if(getArmAngle() < -90) {
      staticGain = staticGain *-1;
    }

    //Add feedforward gains to feedback volts
    voltage = (volts + staticGain + setpoint.velocity*ArmConstants.kV);
  
    //Set the voltage on the motors
    armMotors.setVoltage(voltage);
  }

  public void simpleMove(double volts){
    armMotors.setVoltage(volts);
  }

  /**
   * 
   * @return the angle of the arm in degrees based on the crank angle and limited to the max and min transmissoin angles.
   */
  public double getArmAngle(){

    return crankToArm(MathUtil.clamp(getCrankAngle(),ArmConstants.minCrankAngle,ArmConstants.maxCrankAngle));
  }

  /**
   * 
   * @param  The crank angle in question
   * @return The arm angle corresponding to a given crank angle
   */
  public double crankToArm(double crank){
    double A = ArmConstants.X3 - ArmConstants.R1*Math.cos(Math.toRadians(crank));
    double B = ArmConstants.Y3 - ArmConstants.R1*Math.sin(Math.toRadians(crank));

    return Math.toDegrees(Math.asin((Math.pow(ArmConstants.R2,2)-Math.pow(ArmConstants.R3,2)-(A*A+B*B))/(2*ArmConstants.R3*Math.sqrt(A*A+B*B))))
          - Math.toDegrees(Math.atan(A/B))
          - ArmConstants.PHI;
  }

  /**
   * 
   * @return the angle of the crank in degrees (0 deg. being horizontal forward)
   */
  public double getCrankAngle(){
    return rightArmMotor.getSelectedSensorPosition() * 360 / (2048 * ArmConstants.armGearRatio) + ArmConstants.minCrankAngle; 
    //     need to see left vs right motors for sensor        double check sensor units value
    //     could check if the inversion of the left motor allows us to use an average of the two values
  }

  /**
   * 
   * @return the speed of the crank in degrees per second
   */
  public double getCrankSpeed(){
    return rightArmMotor.getSelectedSensorVelocity() * 360 * 10 / (2048 * ArmConstants.armGearRatio);
  }

  public double getRawCrankAngle(){
    return rightArmMotor.getSelectedSensorPosition();
    //227328 added in code (adding 240 degrees instead as this is more intuitive)
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
  public Command moveTo(double goalAngle){

    //Setup controller
    TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(90,90); //We'll work in degrees here since the arm angle methods return degrees
    ProfiledPIDController crankController = new ProfiledPIDController(.5, 0, 0, constraints);

    //Sanitize the input angle
    goalAngle = MathUtil.clamp(goalAngle,crankToArm(ArmConstants.minCrankAngle),crankToArm(ArmConstants.maxCrankAngle));

    return new ProfiledPIDCommand(crankController, () -> getArmAngle(), goalAngle, (output,setpoint) -> move(output,setpoint), this)
                  .withInterruptBehavior(InterruptionBehavior.kCancelSelf);
  }
  public Command moveToAuto(double goalAngle){

    //Setup controller
    TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(120,120); //We'll work in degrees here since the arm angle methods return degrees
    ProfiledPIDController crankController = new ProfiledPIDController(.5, 0, 0, constraints);

    //Sanitize the input angle
    goalAngle = MathUtil.clamp(goalAngle,crankToArm(ArmConstants.minCrankAngle),crankToArm(ArmConstants.maxCrankAngle));

    return new ProfiledPIDCommand(crankController, () -> getArmAngle(), goalAngle, (output,setpoint) -> move(output,setpoint), this)
                  .withInterruptBehavior(InterruptionBehavior.kCancelSelf);
  }

  public Command calMove(){
    PIDController crankCalController = new PIDController(0.05,0,0);
    return new PIDCommand(crankCalController, () -> getCrankSpeed(), -60.0, (output) -> simpleMove(output),this)
                  .withInterruptBehavior(InterruptionBehavior.kCancelSelf);
  }

  public void resetArmAngle(){
    rightArmMotor.setSelectedSensorPosition(0);
  }

  /**
   * This method returns true when the arm crank is triggered indicating the arm is at its low end setpoint.
   * 
   * @return TRUE if the limit switch is triggered

   */
  public boolean atCrankLimit(){
    return !armLimit.get();
  }

  public double getGoalVelocity(){
    return goal.velocity;
  }

  public double getVoltage(){
    return voltage;
  }

  public UsbCamera getFeed(){
    return armCamera;
  }
}
