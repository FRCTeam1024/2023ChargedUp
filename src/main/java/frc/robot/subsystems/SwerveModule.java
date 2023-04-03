// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.Constants.DriveConstants;
 
public class SwerveModule {

  private final WPI_TalonFX m_angleMotor;
  private final WPI_TalonFX m_driveMotor;

  private final CANCoder m_turnEncoder;

  private final PIDController m_drivePIDController = 
      new PIDController(
          DriveConstants.kPModuleDriveController,
          DriveConstants.kIModuleDriveController,
          DriveConstants.kDModuleDriveController); 

  private final ProfiledPIDController m_turningPIDController =
      new ProfiledPIDController(                      
          DriveConstants.kPModuleTurnController, 
          DriveConstants.kIModuleTurnController, 
          DriveConstants.kDModuleTurnController, 
          new TrapezoidProfile.Constraints(
              DriveConstants.kModuleMaxAngularVelocity, DriveConstants.kModuleMaxAngularAcceleration));

  private final SimpleMotorFeedforward m_driveFeedforward = 
      new SimpleMotorFeedforward(
          DriveConstants.ksVolts, 
          DriveConstants.kvVoltSecondsPerMeter, 
          DriveConstants.kaVoltSecondsSquaredPerMeter);
          
  private final SimpleMotorFeedforward m_turnFeedforward = 
      new SimpleMotorFeedforward(
          DriveConstants.ksTurning, 
          DriveConstants.kvTurning, 
          DriveConstants.kaTurning);

  private SwerveModuleState myState = new SwerveModuleState();

 // private SwerveModuleState state;

  /** Creates a new SwerveModule. */
  public SwerveModule(int angleMotorChannel, int driveMotorChannel, int turnEncoderChannel, 
    double turnOffset, boolean turnReversed, boolean driveReversed) {

    m_angleMotor = new WPI_TalonFX(angleMotorChannel);
    m_driveMotor = new WPI_TalonFX(driveMotorChannel);
    m_turnEncoder = new CANCoder(turnEncoderChannel);

    //Always start from known settings
    m_angleMotor.configFactoryDefault();
    m_driveMotor.configFactoryDefault();
    m_turnEncoder.configFactoryDefault();

    //Set motor directions based on parameters
    m_driveMotor.setInverted(driveReversed);
    m_angleMotor.setInverted(turnReversed);

    //Set brake mode
    m_driveMotor.setNeutralMode(NeutralMode.Brake);
    m_angleMotor.setNeutralMode(NeutralMode.Brake);

    //Set how we want to the encoder to read
    m_turnEncoder.configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180);
    m_turnEncoder.configMagnetOffset(turnOffset);

    //Set CAN rates.  Save CAN bus bandwidth by slowing down some CAN that we don't need.
    m_angleMotor.setStatusFramePeriod(StatusFrame.Status_1_General, 250);
    m_angleMotor.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 250);

    // Limit the PID Controller's input range between -pi and pi and set the input
    // to be continuous.
    m_turningPIDController.enableContinuousInput(-Math.PI, Math.PI);
;  }

  public void setDesiredState(SwerveModuleState moduleState){
    // Optimize the reference state to avoid spinning further than 90 degrees
    SwerveModuleState state = SwerveModuleState.optimize(moduleState, new Rotation2d(getAngleRadians()));

    //Store the current goal state for reference
    myState = state;

    // Calculate the drive output from the drive PID controller.
    final double driveOutput =
        m_drivePIDController.calculate(getDriveVelocity(), state.speedMetersPerSecond);

    final double driveFeedforward = m_driveFeedforward.calculate(state.speedMetersPerSecond);

    // Calculate the turning motor output from the turning PID controller.
    final double turnOutput =
        m_turningPIDController.calculate(getAngleRadians(), state.angle.getRadians());

    final double turnFeedforward =
        m_turnFeedforward.calculate(m_turningPIDController.getSetpoint().velocity);

   m_driveMotor.setVoltage(driveOutput + driveFeedforward);
   m_angleMotor.setVoltage(turnOutput + turnFeedforward);

  }

  public void stop(){
    m_driveMotor.setVoltage(0);
    m_angleMotor.setVoltage(0);
  }

  public double getDriveVelocity(){
    return m_driveMotor.getSelectedSensorVelocity()*(10*DriveConstants.wheelCircumference/(DriveConstants.encoderTicks*DriveConstants.gearRatio));
  }

  public double getVelocityError(){
    return myState.speedMetersPerSecond - getDriveVelocity();
  }

  public double getAngleDegrees(){
    return m_turnEncoder.getAbsolutePosition();
  } 

  public double getAngleErrorDegrees(){
    return myState.angle.getDegrees() - getAngleDegrees();
  }

  public double getAngleRadians() {
    return m_turnEncoder.getAbsolutePosition()*Math.PI/180;
  }

  public double getDriveVoltage() {
    return m_driveMotor.getMotorOutputVoltage();
  }

  public double getTurnVoltage() {
    return m_angleMotor.getMotorOutputVoltage();
  }

  public double getDrivekV() {
    return getDriveVoltage() / getDriveVelocity();
  }

  public double getTurnkV() {
    return getTurnVoltage() / (m_turnEncoder.getVelocity() * Math.PI / 180);
  }

  public SwerveModuleState getDesiredState() {
    return myState;
  }

  public SwerveModulePosition getPosition(){
    return new SwerveModulePosition(
      m_driveMotor.getSelectedSensorPosition()*DriveConstants.wheelCircumference/(DriveConstants.encoderTicks*DriveConstants.gearRatio), new Rotation2d(getAngleRadians())
    );
  }

}
