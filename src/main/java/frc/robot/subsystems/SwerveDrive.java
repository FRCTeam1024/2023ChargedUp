// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.WPI_PigeonIMU;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.VisionConstants;
import java.util.Collections;
import java.util.Comparator;
import java.util.List;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonTrackedTarget;

public class SwerveDrive extends SubsystemBase {
  // 0.3016 as magnitude for other robot
  private final Translation2d m_ALocation = new Translation2d(0.2762, 0.2762);
  private final Translation2d m_BLocation = new Translation2d(-0.2762, 0.2762);
  private final Translation2d m_CLocation = new Translation2d(-0.2762, -0.2762);
  private final Translation2d m_DLocation = new Translation2d(0.2762, -0.2762);

  private final SwerveModule a =
      new SwerveModule(
          DriveConstants.angleMotorA,
          DriveConstants.driveMotorA,
          DriveConstants.turnEncoderA,
          DriveConstants.moduleA.turnOffset(),
          true,
          true);
  private final SwerveModule b =
      new SwerveModule(
          DriveConstants.angleMotorB,
          DriveConstants.driveMotorB,
          DriveConstants.turnEncoderB,
          DriveConstants.moduleB.turnOffset(),
          true,
          false);
  private final SwerveModule c =
      new SwerveModule(
          DriveConstants.angleMotorC,
          DriveConstants.driveMotorC,
          DriveConstants.turnEncoderC,
          DriveConstants.moduleC.turnOffset(),
          true,
          false);
  private final SwerveModule d =
      new SwerveModule(
          DriveConstants.angleMotorD,
          DriveConstants.driveMotorD,
          DriveConstants.turnEncoderD,
          DriveConstants.moduleD.turnOffset(),
          true,
          true);

  List<PhotonPoseEstimator> cameras =
      List.of(
          new PhotonPoseEstimator(
              Constants.kFieldLayout,
              PoseStrategy.CLOSEST_TO_REFERENCE_POSE,
              new PhotonCamera(VisionConstants.leftCameraName),
              VisionConstants.robotToLeftCam),
          new PhotonPoseEstimator(
              Constants.kFieldLayout,
              PoseStrategy.CLOSEST_TO_REFERENCE_POSE,
              new PhotonCamera(VisionConstants.rightCameraName),
              VisionConstants.robotToRightCam));

  private final WPI_PigeonIMU pigeon = new WPI_PigeonIMU(DriveConstants.gyroID);

  private final SwerveDriveKinematics m_kinematics =
      new SwerveDriveKinematics(m_ALocation, m_BLocation, m_CLocation, m_DLocation);

  private final SwerveDrivePoseEstimator m_odometry;

  private SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];

  // For vision estimation - according to PhotonVision & WPILIB examples, it should be possible to
  // just drop this in as a replacement for odometry

  /** Creates a new SwerveDrive. */
  public SwerveDrive() {
    // pigeon.enterCalibrationMode(CalibrationMode.Temperature);
    pigeon.setStatusFramePeriod(9, 5);
    // System.out.println("Initial Pigeon Yaw: " + pigeon.getYaw());
    pigeon.setYaw(180); // could try pigeon.addYaw(180)
    // pigeon.setFusedHeading(180,10);
    // System.out.println("Changed Pigeon Yaw: " + pigeon.getYaw());

    modulePositions[0] = a.getPosition();
    modulePositions[1] = b.getPosition();
    modulePositions[2] = c.getPosition();
    modulePositions[3] = d.getPosition();

    m_odometry =
        new SwerveDrivePoseEstimator(
            m_kinematics,
            new Rotation2d(Math.PI),
            modulePositions,
            new Pose2d(),
            VecBuilder.fill(0.01, 0.01, 0.05),
            VecBuilder.fill(.05, 0.05, Units.degreesToRadians(5)));
  }

  @Override
  public void periodic() {

    // This method will be called once per scheduler run
    modulePositions[0] = a.getPosition();
    modulePositions[1] = b.getPosition();
    modulePositions[2] = c.getPosition();
    modulePositions[3] = d.getPosition();

    m_odometry.update(pigeon.getRotation2d(), modulePositions);

    for (PhotonPoseEstimator camera : cameras) {
      camera.setReferencePose(getPose());
      var poseEstimate = camera.update();

      if (poseEstimate.isPresent()) {
        m_odometry.addVisionMeasurement(
            poseEstimate.get().estimatedPose.toPose2d(), poseEstimate.get().timestampSeconds);
      }
    }

    // System.out.println(calculatePathToTag().getEndState().toString());
    SmartDashboard.putNumber("OdometryPose/Y", m_odometry.getEstimatedPosition().getY());
    SmartDashboard.putNumber("OdometryPose/X", m_odometry.getEstimatedPosition().getX());
    SmartDashboard.putNumber(
        "OdometryPose/Angle", m_odometry.getEstimatedPosition().getRotation().getDegrees());
    var odoPoseArray =
        new double[] {
          m_odometry.getEstimatedPosition().getX(),
          m_odometry.getEstimatedPosition().getY(),
          m_odometry.getEstimatedPosition().getRotation().getRadians()
        };
    SmartDashboard.putNumberArray("OdometryPose/Pose", odoPoseArray);
  }

  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    var rotation = m_odometry.getEstimatedPosition().getRotation();

    if (DriverStation.getAlliance() == Alliance.Red) {
      rotation = rotation.minus(Rotation2d.fromDegrees(180));
    }
    SwerveModuleState[] moduleStates =
        m_kinematics.toSwerveModuleStates(
            fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, rotation)
                : new ChassisSpeeds(xSpeed, ySpeed, rot));
    SwerveDriveKinematics.desaturateWheelSpeeds(
        moduleStates, DriveConstants.kMaxWheelSpeedMetersPerSecond);
    a.setDesiredState(moduleStates[0]);
    b.setDesiredState(moduleStates[1]);
    c.setDesiredState(moduleStates[2]);
    d.setDesiredState(moduleStates[3]);
  }

  public double getAngleRad(int id) {
    if (id == 1) {
      return a.getAngleDegrees();
    } else if (id == 2) {
      return b.getAngleDegrees();
    } else if (id == 3) {
      return c.getAngleDegrees();
    } else if (id == 4) {
      return d.getAngleDegrees();
    } else {
      return Math.PI * 10;
    }
  }

  public String getModulePosition(int id) {
    if (id == 1) {
      return a.getPosition().toString();
    } else if (id == 2) {
      return b.getPosition().toString();
    } else if (id == 3) {
      return c.getPosition().toString();
    } else if (id == 4) {
      return d.getPosition().toString();
    } else {
      return "uh oh";
    }
  }

  public double getModVelError(int id) {
    if (id == 1) {
      return a.getVelocityError();
    } else if (id == 2) {
      return b.getVelocityError();
    } else if (id == 3) {
      return c.getVelocityError();
    } else if (id == 4) {
      return d.getVelocityError();
    } else {
      return 999;
    }
  }

  public double getModAngError(int id) {
    if (id == 1) {
      return a.getAngleErrorDegrees();
    } else if (id == 2) {
      return b.getAngleErrorDegrees();
    } else if (id == 3) {
      return c.getAngleErrorDegrees();
    } else if (id == 4) {
      return d.getAngleErrorDegrees();
    } else {
      return 999;
    }
  }

  public double getModDrivekV(int id) {
    if (id == 1) {
      return a.getDrivekV();
    } else if (id == 2) {
      return b.getDrivekV();
    } else if (id == 3) {
      return c.getDrivekV();
    } else if (id == 4) {
      return d.getDrivekV();
    } else {
      return 999;
    }
  }

  public double getModDriveVoltage(int id) {
    if (id == 1) {
      return a.getDriveVoltage();
    } else if (id == 2) {
      return b.getDriveVoltage();
    } else if (id == 3) {
      return c.getDriveVoltage();
    } else if (id == 4) {
      return d.getDriveVoltage();
    } else {
      return 999;
    }
  }

  public double getModTurnVoltage(int id) {
    if (id == 1) {
      return a.getTurnVoltage();
    } else if (id == 2) {
      return b.getTurnVoltage();
    } else if (id == 3) {
      return c.getTurnVoltage();
    } else if (id == 4) {
      return d.getTurnVoltage();
    } else {
      return 999;
    }
  }

  public double getModTurnkV(int id) {
    if (id == 1) {
      return a.getTurnkV();
    } else if (id == 2) {
      return b.getTurnkV();
    } else if (id == 3) {
      return c.getTurnkV();
    } else if (id == 4) {
      return d.getTurnkV();
    } else {
      return 999;
    }
  }

  public double getYawDegrees() {
    // System.out.println("\nPigeon yaw: " + pigeon.getYaw() + "\nOdometry yaw: " +
    // m_odometry.getEstimatedPosition().getRotation().getDegrees() + "\n");
    return m_odometry.getEstimatedPosition().getRotation().getDegrees();
  }

  public void zeroHeading() {
    Pose2d pose = new Pose2d(m_odometry.getEstimatedPosition().getTranslation(), new Rotation2d(0));
    // pigeon.reset();
    pigeon.setYaw(0);
    m_odometry.resetPosition(pigeon.getRotation2d(), modulePositions, pose);
  }

  public void setHeading180() {
    Pose2d pose =
        new Pose2d(m_odometry.getEstimatedPosition().getTranslation(), new Rotation2d(Math.PI));
    pigeon.setYaw(180);
    m_odometry.resetPosition(pigeon.getRotation2d(), modulePositions, pose);
  }

  // might work, didnt have time to test
  public void resetPosition(Pose2d pose) {

    modulePositions[0] = a.getPosition();
    modulePositions[1] = b.getPosition();
    modulePositions[2] = c.getPosition();
    modulePositions[3] = d.getPosition();

    m_odometry.resetPosition(pigeon.getRotation2d(), modulePositions, pose);
  }

  public void defenseMode() {
    SwerveModuleState[] moduleStates = {
      new SwerveModuleState(0, new Rotation2d(Math.PI / 4)),
      new SwerveModuleState(0, new Rotation2d(3 * Math.PI / 4)),
      new SwerveModuleState(0, new Rotation2d(Math.PI / 4)),
      new SwerveModuleState(0, new Rotation2d(3 * Math.PI / 4))
    };
    a.setDesiredState(moduleStates[0]);
    b.setDesiredState(moduleStates[1]);
    c.setDesiredState(moduleStates[2]);
    d.setDesiredState(moduleStates[3]);
  }

  public void stop() {
    a.stop();
    b.stop();
    c.stop();
    d.stop();
  }

  public SwerveDrivePoseEstimator getOdometry() {
    return m_odometry;
  }

  public Rotation2d getRotation2d() {
    return pigeon.getRotation2d();
  }

  public double getPigeonHeading() {
    return pigeon.getRotation2d().getDegrees();
  }

  public double getPigeonYaw() {
    return pigeon.getYaw();
  }

  public SwerveModulePosition[] getSwerveModulePositions() {
    return modulePositions;
  }

  public SwerveDriveKinematics getSwerveDriveKinematics() {
    return m_kinematics;
  }

  public void setModuleStates(SwerveModuleState[] moduleStates) {
    a.setDesiredState(moduleStates[0]);
    b.setDesiredState(moduleStates[1]);
    c.setDesiredState(moduleStates[2]);
    d.setDesiredState(moduleStates[3]);
  }

  public Pose2d getPose() {
    return m_odometry.getEstimatedPosition();
  }

  public SwerveModuleState[] getDesiredStates() {
    SwerveModuleState[] moduleStates = {
      a.getDesiredState(), b.getDesiredState(), c.getDesiredState(), d.getDesiredState()
    };
    return moduleStates;
  }

  public double getPitch() {
    return pigeon.getPitch();
  }

  public double getRoll() {
    return pigeon.getRoll();
  }

  public void sortTargetsByArea(List<PhotonTrackedTarget> targets) {
    Collections.sort(
        targets,
        new Comparator<PhotonTrackedTarget>() {
          @Override
          public int compare(PhotonTrackedTarget a, PhotonTrackedTarget b) {
            double area1 = a.getArea();
            double area2 = b.getArea();
            if (area2 > area1) {
              return -1;
            } else if (area1 > area2) {
              return 1;
            } else {
              return 0;
            }
          }
        });
  }

  public Command followTrajectory(PathPlannerTrajectory inputPath) {
    // attempts to manually change path planning
    PathPlannerTrajectory path;
    if (DriverStation.getAlliance() == Alliance.Red) {
      path = PathPlannerTrajectory.transformTrajectoryForAlliance(inputPath, Alliance.Red);
    } else {
      path = inputPath;
    }
    return new SequentialCommandGroup(
        basicFirstTrajectory(path), new InstantCommand(() -> this.stop()));
  }

  /**
   * Encapsulting the PPSwerveController Command generation here so that this code does not need to
   * be repeated for path planner and vision trajectory following.
   *
   * @param path the path to follow
   * @return the command to follow the path
   */
  public Command basicFirstTrajectory(PathPlannerTrajectory path) {
    return new SequentialCommandGroup(
        new PrintCommand(
            "\n\n"
                + path.getInitialState().toString()
                + "\n\n"
                + path.getEndState().toString()
                + "\n\n"),
        new PrintCommand(
            "\n\n"
                + getYawDegrees()
                + "\n\n"
                + path.getInitialPose().getRotation().getDegrees()
                + "\n\n"),
        new InstantCommand(() -> this.resetPosition(path.getInitialHolonomicPose())),
        new PPSwerveControllerCommand(
            path,
            this::getPose, // Pose supplier
            this.getSwerveDriveKinematics(), // SwerveDriveKinematics
            new PIDController(
                5, 0,
                0.05), // X controller. Tune these values for your robot. Leaving them 0 will only
            // use feedforwards.
            new PIDController(5, 0, 0.05), // Y controller (usually the same values as X controller)
            new PIDController(
                5, 0,
                0.005), // Rotation controller. Tune these values for your robot. Leaving them 0
            // will only use feedforwards.
            this::setModuleStates, // Module states consumer
            false,
            this // Requires this drive subsystem
            ));
  }

  public double getChargeStationAngle() {
    return (Math.cos(getYawDegrees()) * getRoll() + Math.sin(getYawDegrees()) * getPitch());
  }
}
