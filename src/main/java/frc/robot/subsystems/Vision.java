// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.Optional;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.EstimatedRobotPose;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.HttpCamera;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableEvent;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPoint;

public class Vision extends SubsystemBase {
  private final PhotonCamera photon = new PhotonCamera("OV5647");
  private ArrayList<Pair<PhotonCamera, Transform3d>> camList = new ArrayList<Pair<PhotonCamera, Transform3d>>();
  private final Transform3d robotToCam = new Transform3d(new Translation3d(0.0, Units.inchesToMeters(10), Units.inchesToMeters(-3)), new Rotation3d(0, 0, 0));

  private final HttpCamera camera = new HttpCamera(
      "limelight", "http://photonvision.local:1182/stream.mjpg?1675300761194", HttpCamera.HttpCameraKind.kMJPGStreamer
  );

  public static final List<AprilTag> aprilTags =
    List.of(
      new AprilTag(1,
        new Pose3d(
          Units.inchesToMeters(610.77),
          Units.inchesToMeters(42.19),
          Units.inchesToMeters(18.22),
          new Rotation3d(0.0, 0.0, Math.PI))),
      new AprilTag(2,
        new Pose3d(
          Units.inchesToMeters(610.77),
          Units.inchesToMeters(108.19),
          Units.inchesToMeters(18.22),
          new Rotation3d(0.0, 0.0, Math.PI))),
      new AprilTag(3,
        new Pose3d(
          Units.inchesToMeters(610.77),
          Units.inchesToMeters(174.19), // FIRST's diagram has a typo (it says 147.19)
          Units.inchesToMeters(18.22),
          new Rotation3d(0.0, 0.0, Math.PI))),
      new AprilTag(4,
        new Pose3d(
          Units.inchesToMeters(636.96),
          Units.inchesToMeters(265.74),
          Units.inchesToMeters(27.38),
          new Rotation3d(0.0, 0.0, Math.PI))),
      new AprilTag(5,
        new Pose3d(
          Units.inchesToMeters(14.25),
          Units.inchesToMeters(265.74),
          Units.inchesToMeters(27.38),
          new Rotation3d())),
      new AprilTag(6,
        new Pose3d(
          Units.inchesToMeters(40.45),
          Units.inchesToMeters(174.19), // FIRST's diagram has a typo (it says 147.19)
          Units.inchesToMeters(18.22),
          new Rotation3d())),
      new AprilTag(7,
        new Pose3d(
          Units.inchesToMeters(40.45),
          Units.inchesToMeters(108.19),
          Units.inchesToMeters(18.22),
          new Rotation3d())),
      new AprilTag(8,
        new Pose3d(
          Units.inchesToMeters(40.45),
          Units.inchesToMeters(42.19),
          Units.inchesToMeters(18.22),
          new Rotation3d())));

  // 2023 field layout is not out yet...
  private final AprilTagFieldLayout fieldLayout2023 = new AprilTagFieldLayout(aprilTags, 16.54, 8.02);
  private PhotonPoseEstimator robotPoseEstimator;

  /** Creates a new Vision. */
  public Vision() {
    camList.add(new Pair<PhotonCamera, Transform3d>(photon, robotToCam));
    robotPoseEstimator = new PhotonPoseEstimator(fieldLayout2023, PoseStrategy.AVERAGE_BEST_TARGETS, photon, robotToCam);
    CameraServer.addCamera(camera);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  // Returns a PhotonPipelineResult which contains all currently visible targets
  public PhotonPipelineResult getResults() {
    return photon.getLatestResult();
  }

  // Checks if photon has targets. Call this to avoid null pointer exceptions.
  // Should it be implemented into getResults()?
  public boolean hasTargets() {
    return getResults().hasTargets();
  }

  public List<PhotonTrackedTarget> getTargets() {
    return getResults().getTargets();
  }

  public PhotonTrackedTarget getBestTarget() {
    return getResults().getBestTarget();
  }

  public Pair<Pose2d, Double> estimateRobotPose(Pose2d prevEstimatedRobotPose) {
    robotPoseEstimator.setReferencePose(prevEstimatedRobotPose);

    double currentTime = Timer.getFPGATimestamp();
    Optional<EstimatedRobotPose> result = robotPoseEstimator.update();
    if(result.isPresent()){
      //System.out.println(result.get().estimatedPose.toPose2d().toString());
      return new Pair<Pose2d, Double>(result.get().estimatedPose.toPose2d(), currentTime - result.get().timestampSeconds);
    }else{
      System.out.println("Uh oh, no vision");
      return new Pair<Pose2d, Double>(null, 0.0);
    }
    
  }

  public PathPlannerTrajectory getPathToAprilTag(Pose2d robotPose) {
    PathPlannerTrajectory trajectoryToTag;
    Translation2d robotPosition = new Translation2d(robotPose.getX(), robotPose.getY());
    Rotation2d robotRotation = robotPose.getRotation();
    if (hasTargets()) {  
      Transform3d camToTarget = getBestTarget().getBestCameraToTarget();
      System.out.println(camToTarget.toString());
      Pose2d targetPose = robotPose.plus(
        new Transform2d(
          new Translation2d(-camToTarget.getX() + 1, -camToTarget.getY()),
          new Rotation2d(camToTarget.getRotation().getZ())
        )
      );
      trajectoryToTag = PathPlanner.generatePath(
        new PathConstraints(1, 1),
        new PathPoint(robotPosition, robotRotation),
        new PathPoint(targetPose.getTranslation(), targetPose.getRotation())
      );
      System.out.println(trajectoryToTag.getInitialState().toString());
      System.out.println(trajectoryToTag.getEndState().toString());
    }else{
      trajectoryToTag = PathPlanner.generatePath(
        new PathConstraints(1, 1),
        new PathPoint(robotPosition, robotRotation),
        new PathPoint(new Translation2d(robotPosition.getX() + 0.1, robotPosition.getY()), robotRotation)
      );
    }

    return trajectoryToTag;
  }

  public HttpCamera getFeed(){
    return camera;
  }
}
