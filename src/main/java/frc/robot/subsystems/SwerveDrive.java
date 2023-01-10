// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.WPI_PigeonIMU;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class SwerveDrive extends SubsystemBase {

  private final Translation2d m_ALocation = new Translation2d(0.3016, 0.3016); 
  private final Translation2d m_BLocation = new Translation2d(-0.3016, 0.3016);
  private final Translation2d m_CLocation = new Translation2d(-0.3016, -0.3016);
  private final Translation2d m_DLocation = new Translation2d(0.3016, -0.3016);

  private final SwerveModule a = new SwerveModule(DriveConstants.angleMotorA, DriveConstants.driveMotorA, DriveConstants.turnEncoderA, DriveConstants.turnOffsetA, true, true, Math.PI/4);
  private final SwerveModule b = new SwerveModule(DriveConstants.angleMotorB, DriveConstants.driveMotorB, DriveConstants.turnEncoderB, DriveConstants.turnOffsetB, true, false, 3*Math.PI/4);
  private final SwerveModule c = new SwerveModule(DriveConstants.angleMotorC, DriveConstants.driveMotorC, DriveConstants.turnEncoderC, DriveConstants.turnOffsetC, true, false, Math.PI/4);
  private final SwerveModule d = new SwerveModule(DriveConstants.angleMotorD, DriveConstants.driveMotorD, DriveConstants.turnEncoderD, DriveConstants.turnOffsetD, true, true, 3*Math.PI/4);
  
  private final WPI_PigeonIMU pigeon = new WPI_PigeonIMU(DriveConstants.gyroID);
  
  private final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(m_ALocation, m_BLocation, m_CLocation, m_DLocation);

  private final SwerveDriveOdometry m_odometry;

  private double[] yawPitchRoll = new double[3];

  /** Creates a new SwerveDrive. */
  public SwerveDrive() {
    pigeon.setYaw(0);

    m_odometry = new SwerveDriveOdometry(m_kinematics, pigeon.getRotation2d());



  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    m_odometry.update(
            pigeon.getRotation2d(),
            a.getState(),
            b.getState(),
            c.getState(),
            d.getState()
    );

  }

  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    SwerveModuleState[] moduleStates =
        m_kinematics.toSwerveModuleStates(
            fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, pigeon.getRotation2d())
                : new ChassisSpeeds(xSpeed, ySpeed, rot));
    SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, DriveConstants.kMaxWheelSpeedMetersPerSecond);
    a.setDesiredState(moduleStates[0]);
    b.setDesiredState(moduleStates[1]);
    c.setDesiredState(moduleStates[2]);
    d.setDesiredState(moduleStates[3]);
  }

  public double getAngleRad(int id){
    if(id == 1){
      return a.getAngleDegrees();
    }else if(id == 2){
      return b.getAngleDegrees();
    }else if(id == 3){
      return c.getAngleDegrees();
    }else if(id == 4){
      return d.getAngleDegrees();
    }else{
      return Math.PI * 10;
    }
  }


  private double getRawYaw() {
    pigeon.getYawPitchRoll(yawPitchRoll);
    return yawPitchRoll[0];
  }

  public Rotation2d getYawRotation2d() {
    return new Rotation2d(Math.PI * getRawYaw()/180);
  }

  public double getYawDegrees() {
    //return getYawRotation2d().getDegrees();
    return m_odometry.getPoseMeters().getRotation().getDegrees();
  }

  public void zeroHeading(){
    Pose2d pose = new Pose2d(m_odometry.getPoseMeters().getTranslation(), pigeon.getRotation2d());
    pigeon.reset();
    m_odometry.resetPosition(pose, pigeon.getRotation2d());
  }

  public void defenseMode(){
    SwerveModuleState[] moduleStates = {
      new SwerveModuleState(0, new Rotation2d(Math.PI/4)),
      new SwerveModuleState(0, new Rotation2d(3*Math.PI/4)),
      new SwerveModuleState(0, new Rotation2d(Math.PI/4)),
      new SwerveModuleState(0, new Rotation2d(3*Math.PI/4))
    };
    a.setDesiredState(moduleStates[0]);
    b.setDesiredState(moduleStates[1]);
    c.setDesiredState(moduleStates[2]);
    d.setDesiredState(moduleStates[3]);
  }
}
