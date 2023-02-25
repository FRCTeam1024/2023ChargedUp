// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Collections;
import java.util.Comparator;
import java.util.List;

import org.photonvision.PhotonTargetSortMode;
import org.photonvision.targeting.PhotonTrackedTarget;
import com.ctre.phoenix.sensors.WPI_PigeonIMU;
import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.ProxyCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.commands.AutoAlignAprilTag;
import frc.robot.commands.AutoTurn;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class SwerveDrive extends SubsystemBase {
  //0.3016 as magnitude for other robot
  private final Translation2d m_ALocation = new Translation2d(0.2762, 0.2762); 
  private final Translation2d m_BLocation = new Translation2d(-0.2762, 0.2762);
  private final Translation2d m_CLocation = new Translation2d(-0.2762, -0.2762);
  private final Translation2d m_DLocation = new Translation2d(0.2762, -0.2762);

  private final SwerveModule a = new SwerveModule(DriveConstants.angleMotorA, DriveConstants.driveMotorA, DriveConstants.turnEncoderA, DriveConstants.moduleA.turnOffset(), true, true, Math.PI/4);
  private final SwerveModule b = new SwerveModule(DriveConstants.angleMotorB, DriveConstants.driveMotorB, DriveConstants.turnEncoderB, DriveConstants.moduleB.turnOffset(), true, false, 3*Math.PI/4);
  private final SwerveModule c = new SwerveModule(DriveConstants.angleMotorC, DriveConstants.driveMotorC, DriveConstants.turnEncoderC, DriveConstants.moduleC.turnOffset(), true, false, Math.PI/4);
  private final SwerveModule d = new SwerveModule(DriveConstants.angleMotorD, DriveConstants.driveMotorD, DriveConstants.turnEncoderD, DriveConstants.moduleD.turnOffset(), true, true, 3*Math.PI/4);
  
  private final Vision camera = new Vision();

  private final WPI_PigeonIMU pigeon = new WPI_PigeonIMU(DriveConstants.gyroID);
  
  private final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(m_ALocation, m_BLocation, m_CLocation, m_DLocation);

  private final SwerveDriveOdometry m_odometry;

  private double[] yawPitchRoll = new double[3];

  private SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];

  //For vision estimation - according to PhotonVision & WPILIB examples, it should be possible to just drop this in as a replacement for odometry
  private SwerveDrivePoseEstimator m_poseEstimator;

  public double[] velocityErrors = new double[4];
  public double[] angleErrors = new double[4];

  /** Creates a new SwerveDrive. */
  public SwerveDrive() {
    pigeon.setYaw(0);

    modulePositions[0] = a.getPosition();
    modulePositions[1] = b.getPosition();
    modulePositions[2] = c.getPosition();
    modulePositions[3] = d.getPosition();

    m_odometry = new SwerveDriveOdometry(m_kinematics, pigeon.getRotation2d(), modulePositions);

    m_poseEstimator = new SwerveDrivePoseEstimator(m_kinematics, pigeon.getRotation2d(), modulePositions, new Pose2d());

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    modulePositions[0] = a.getPosition();
    modulePositions[1] = b.getPosition();
    modulePositions[2] = c.getPosition();
    modulePositions[3] = d.getPosition();

    m_odometry.update(
            pigeon.getRotation2d(),
            modulePositions
    );

    m_poseEstimator.update(pigeon.getRotation2d(), modulePositions);

    //System.out.println(calculatePathToTag().getEndState().toString());
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

  public String getModulePosition(int id){
    if(id == 1){
      return a.getPosition().toString();
    }else if(id == 2){
      return b.getPosition().toString();
    }else if(id == 3){
      return c.getPosition().toString();
    }else if(id == 4){
      return d.getPosition().toString();
    }else{
      return "uh oh";
    }
  }

  public double getModVelError(int id){
    if(id == 1){
      return a.getVelocityError();
    }else if(id == 2){
      return b.getVelocityError();
    }else if(id == 3){
      return c.getVelocityError();
    }else if(id == 4){
      return d.getVelocityError();
    }else{
      return 999;
    }
  }
  public double getModAngError(int id){
    if(id == 1){
      return a.getAngleErrorDegrees();
    }else if(id == 2){
      return b.getAngleErrorDegrees();
    }else if(id == 3){
      return c.getAngleErrorDegrees();
    }else if(id == 4){
      return d.getAngleErrorDegrees();
    }else{
      return 999;
    }
  }

  public double getYawDegrees() {
    return m_odometry.getPoseMeters().getRotation().getDegrees();
  }

  public void zeroHeading(){
    Pose2d pose = new Pose2d(m_odometry.getPoseMeters().getTranslation(), pigeon.getRotation2d());
    pigeon.reset();
    m_odometry.resetPosition(pigeon.getRotation2d(), modulePositions, pose);
  }

  //might work, didnt have time to test
  public void resetPosition(Pose2d pose){
    
    modulePositions[0] = a.getPosition();
    modulePositions[1] = b.getPosition();
    modulePositions[2] = c.getPosition();
    modulePositions[3] = d.getPosition();

    m_odometry.resetPosition(pigeon.getRotation2d(), modulePositions, pose);
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

  public SwerveDriveOdometry getOdometry(){
    return m_odometry;
  }

  public Rotation2d getRotation2d(){
    return pigeon.getRotation2d();
  }

  public SwerveModulePosition[] getSwerveModulePositions(){
    return modulePositions;
  }

  public SwerveDriveKinematics getSwerveDriveKinematics(){
    return m_kinematics;
  }

  public void setModuleStates(SwerveModuleState[] moduleStates){
    a.setDesiredState(moduleStates[0]);
    b.setDesiredState(moduleStates[1]);
    c.setDesiredState(moduleStates[2]);
    d.setDesiredState(moduleStates[3]);
  }

  public Pose2d getPose(){
    Pose2d pose = m_odometry.getPoseMeters();
    return pose;
  }

  public SwerveModuleState[] getDesiredStates(){
    SwerveModuleState[] moduleStates = {
      a.getDesiredState(),
      b.getDesiredState(),
      c.getDesiredState(),
      d.getDesiredState()
    };
    return moduleStates;
  }

  public double getPitch(){
    return pigeon.getPitch();
  }

  public double getRoll(){
    return pigeon.getRoll();
  }

  public void visionEstimatedPose(){
    Pair<Pose2d,Double> pair = camera.estimateRobotPose(getPose());
    m_poseEstimator.addVisionMeasurement(
      pair.getFirst(), pair.getSecond()
    );
  }

  public Vision getCamera(){
    return camera;
  }

  public double getTargetYaw(){
    if(camera.hasTargets()){
      PhotonTrackedTarget target = camera.getBestTarget();
      if(target != null){
        return target.getYaw();
      }else{
        return 180;
      }
    }else{
      return 180;
    }
  }

  public void sortTargetsByArea(List<PhotonTrackedTarget> targets){
    Collections.sort(targets, new Comparator<PhotonTrackedTarget>() {
      @Override
      public int compare(PhotonTrackedTarget a, PhotonTrackedTarget b){
        double area1 = a.getArea();
        double area2 = b.getArea();
        if(area2 > area1){
          return -1;
        }else if(area1 > area2){
          return 1;
        }else{
          return 0;
        }
      }
    });
  }

  public PathPlannerTrajectory calculatePathToTag() {
    Pose2d robotPose = getPose();
    Translation2d robotPosition = new Translation2d(robotPose.getX(), robotPose.getY());
    Rotation2d robotRotation = robotPose.getRotation();
    if (camera.hasTargets()) {  
      List<PhotonTrackedTarget> targets = camera.getTargets();
      targets.sort(new Comparator<PhotonTrackedTarget>() {
        @Override
        public int compare(PhotonTrackedTarget a, PhotonTrackedTarget b){
          Translation3d translate1 = a.getBestCameraToTarget().getTranslation();
          double dist1 = Math.sqrt(Math.pow(translate1.getX(),2)+Math.pow(translate1.getY(),2));
          Translation3d translate2 = b.getBestCameraToTarget().getTranslation();
          double dist2 = Math.sqrt(Math.pow(translate2.getX(),2)+Math.pow(translate2.getY(),2));
          if(dist2 > dist1){
            return -1;
          }else if(dist1 > dist2){
            return 1;
          }else{
            return 0;
          }
        }
      });
      Transform3d camToTarget = targets.get(0).getBestCameraToTarget();
      
      System.out.println(targets.toString());
      Pose2d targetPose = robotPose.plus(
        new Transform2d(
          new Translation2d(-camToTarget.getX() + 1, -camToTarget.getY()),
          new Rotation2d(camToTarget.getRotation().getZ())
        )
      );
      return PathPlanner.generatePath(
        new PathConstraints(2, 2),
        new PathPoint(robotPosition, robotRotation),
        new PathPoint(targetPose.getTranslation(), targetPose.getRotation())
      );
    } else {
      return PathPlanner.generatePath(
        new PathConstraints(2, 2),
        new PathPoint(robotPosition, robotRotation),
        new PathPoint(robotPosition, robotRotation)
      );
    }
  }

  public Command followTrajectory(PathPlannerTrajectory inputPath){
    //attempts to manually change path planning
    PathPlannerTrajectory path;
    if(DriverStation.getAlliance() == Alliance.Red){
      path = PathPlannerTrajectory.transformTrajectoryForAlliance(inputPath, Alliance.Red);
    }else{
      path = inputPath;
    }
    return new SequentialCommandGroup(
      basicFirstTrajectory(path),
      new InstantCommand(() -> this.defenseMode())
    );
  }

  public Command followVisionTrajectory(){
    PathPlannerTrajectory path = calculatePathToTag();
    return new SequentialCommandGroup(
      basicFirstTrajectory(path),
      new AutoTurn(this, 0),
      new InstantCommand(() -> this.defenseMode())
    );
  }

  /**
   * Encapsulting the PPSwerveController Command generation here so that this code does not need
   * to be repeated for path planner and vision trajectory following.
   * 
   * @param path the path to follow
   * @return the command to follow the path
   */
  public Command basicFirstTrajectory(PathPlannerTrajectory path){
    return new SequentialCommandGroup(
      new PrintCommand("\n\n" + path.getInitialState().toString() + "\n\n" + path.getEndState().toString() + "\n\n"),
      new InstantCommand(() -> this.resetPosition(path.getInitialHolonomicPose())),
      new ParallelCommandGroup(
        new PPSwerveControllerCommand(
          path,
          this::getPose, // Pose supplier
          this.getSwerveDriveKinematics(), // SwerveDriveKinematics
          new PIDController(5, 0, 0.05), // X controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
          new PIDController(5, 0, 0.05), // Y controller (usually the same values as X controller)
          new PIDController(5, 0, 0.005), // Rotation controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
          this::setModuleStates, // Module states consumer
          false,
          this // Requires this drive subsystem
        )
      )
    );

  }

  public double getChargeStationAngle(){
    return (Math.cos(getYawDegrees()) * getRoll() + Math.sin(getYawDegrees()) * getPitch());
  }


}
