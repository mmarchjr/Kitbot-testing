// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.io.IOException;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Constants.VisionConstants;

public class photonPose extends SubsystemBase {
  
  /** Creates a new PhotonPoseEstimator. */
  PhotonPoseEstimator poseEstimator;
  Field2d field = new Field2d();
  PhotonCamera camera = new PhotonCamera(VisionConstants.APRILTAG_CAMERA_NAME);
  public photonPose() {
     AprilTagFieldLayout layout;
    try {
      layout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2023ChargedUp.m_resourceFile);
      var alliance = DriverStation.getAlliance();
      //layout.setOrigin(alliance.get() == Alliance.Blue ?
      //    OriginPosition.kBlueAllianceWallRightSide : OriginPosition.kRedAllianceWallRightSide);
    } catch(IOException e) {
      DriverStation.reportError("Failed to load AprilTagFieldLayout", e.getStackTrace());
      layout = null;
    }
    this.poseEstimator = new PhotonPoseEstimator(layout, PoseStrategy.AVERAGE_BEST_TARGETS, camera, Constants.VisionConstants.APRILTAG_CAMERA_TO_ROBOT);

  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    poseEstimator.setReferencePose(RobotContainer.m_robotDrive.getPose());
    poseEstimator.update(camera.getLatestResult());
    if (camera.getLatestResult().hasTargets() && poseEstimator.update().isPresent()){
    field.setRobotPose(getPose());
    SmartDashboard.putData("Field2", field);

  }
  }

  private Pose2d getPose() {
return poseEstimator.update(camera.getLatestResult()).get().estimatedPose.toPose2d();
}
}
