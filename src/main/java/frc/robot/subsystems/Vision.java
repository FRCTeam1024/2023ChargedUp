// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.RobotPoseEstimator;
import org.photonvision.RobotPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase {
  private final PhotonCamera photon = new PhotonCamera("1024-photon");
  private ArrayList<Pair<PhotonCamera, Transform3d>> camList = new ArrayList<Pair<PhotonCamera, Transform3d>>();
  private final Transform3d robotToCam = new Transform3d(new Translation3d(0.0, 0.0, 0.0), new Rotation3d(0, 0, 0));
  // 2023 field layout is not out yet...
  //private final AprilTagFieldLayout fieldLayout2023 = new AprilTagFieldLayout(AprilTagFieldLayout.loadFromResource(AprilTagFields.k2023ChargedUp));
  private RobotPoseEstimator robotPoseEstimator;

  /** Creates a new Vision. */
  public Vision() {
    camList.add(new Pair<PhotonCamera, Transform3d>(photon, robotToCam));
    //robotPoseEstimator = new RobotPoseEstimator(fieldLayout2023, PoseStrategy.AVERAGE_BEST_TARGETS, camList);
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

  /*public Pose3d estimateRobotPose() {
    PhotonTrackedTarget currentTarget = getBestTarget(); 
  }*/
}
