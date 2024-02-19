package frc.robot.subsystems;


import static frc.robot.Constants.VisionConstants.APRILTAG_CAMERA_TO_ROBOT;

import java.io.IOException;
import java.util.Optional;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.estimation.TargetModel;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.VisionConstants;

public class PoseEstimatorSubsystem extends SubsystemBase {

  private final DriveSubsystem drivetrainSubsystem;
  private final AprilTagFieldLayout aprilTagFieldLayout;
  private final SUBVision subVision;
  
  // Kalman Filter Configuration. These can be "tuned-to-taste" based on how much
  // you trust your various sensors. Smaller numbers will cause the filter to
  // "trust" the estimate from that particular component more than the others. 
  // This in turn means the particualr component will have a stronger influence
  // on the final pose estimate.

  /**
   * Standard deviations of model states. Increase these numbers to trust your model's state estimates less. This
   * matrix is in the form [x, y, theta]ᵀ, with units in meters and radians, then meters.
   */
  private static final Vector<N3> stateStdDevs = VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5));
  
  /**
   * Standard deviations of the vision measurements. Increase these numbers to trust global measurements from vision
   * less. This matrix is in the form [x, y, theta]ᵀ, with units in meters and radians.
   */
  private static final Vector<N3> visionMeasurementStdDevs = VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(10));

  private final SwerveDrivePoseEstimator poseEstimator;

  private final Field2d field2d = new Field2d();
ShuffleboardTab tab = Shuffleboard.getTab("Vision");

  private double previousPipelineTimestamp = 0;
  private PhotonPoseEstimator poseEstimator2;

  public PoseEstimatorSubsystem( DriveSubsystem drivetrainSubsystem, SUBVision photonCamera) {
    this.drivetrainSubsystem = drivetrainSubsystem;
    AprilTagFieldLayout layout;
    this.subVision = photonCamera;
    try {
      layout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile);
      var alliance = DriverStation.getAlliance();
      //layout.setOrigin(alliance.get() == Alliance.Blue ?
      //    OriginPosition.kBlueAllianceWallRightSide : OriginPosition.kRedAllianceWallRightSide);
    } catch(IOException e) {
      DriverStation.reportError("Failed to load AprilTagFieldLayout", e.getStackTrace());
      layout = null;
    }
    this.aprilTagFieldLayout = layout;


    poseEstimator =  new SwerveDrivePoseEstimator(
        DriveConstants.kDriveKinematics,
        Rotation2d.fromDegrees(drivetrainSubsystem.getHeading()),
        drivetrainSubsystem.getPosition(),
        new Pose2d(),
        stateStdDevs,
        visionMeasurementStdDevs);
       tab.addString("Pose", this::getFomattedPose).withPosition(0, 0).withSize(2, 0);
    
        tab.addBoolean("Has Targets", subVision::HasTargets);
   tab.add("Field", field2d).withPosition(2, 0).withSize(6, 4);
    if (DriverStation.getAlliance().isPresent()) {
    if (DriverStation.getAlliance().get() == DriverStation.Alliance.Red) {
    layout.setOrigin(OriginPosition.kRedAllianceWallRightSide);
  } else {
    layout.setOrigin(OriginPosition.kBlueAllianceWallRightSide);
      }
    } else {layout.setOrigin(OriginPosition.kBlueAllianceWallRightSide);
    }
  }

  @Override
  public void periodic() {
    var pose = subVision.getEstimatedGlobalPose();
    if (pose.isPresent() && !pose.isEmpty()){
      poseEstimator.addVisionMeasurement(pose.get().estimatedPose.toPose2d(), pose.get().timestampSeconds);
    }
    poseEstimator.update(Rotation2d.fromDegrees(drivetrainSubsystem.getHeading()),drivetrainSubsystem.getPosition());
    //Logger.recordOutput("guess pose", poseEstimator.getEstimatedPosition());
    field2d.setRobotPose(new Pose2d(getCurrentPose().getX(),getCurrentPose().getY(),getCurrentPose().getRotation()));
  }

  private String getFomattedPose() {
    var pose = getCurrentPose();
    return String.format("(%.2f, %.2f) %.2f degrees", 
        pose.getX(), 
        pose.getY(),
        pose.getRotation().getDegrees());
  }

  public Pose2d getCurrentPose() {
    
    return poseEstimator.getEstimatedPosition();
  }

  /**
   * Resets the current pose to the specified pose. This should ONLY be called
   * when the robot's position on the field is known, like at the beginning of
   * a match.
   * @param newPose new pose
   */
  public void setCurrentPose(Pose2d newPose) {
    poseEstimator.resetPosition(
      Rotation2d.fromDegrees(drivetrainSubsystem.getHeading()),
      drivetrainSubsystem.getPosition(),
      newPose);
  }

  /**
   * Resets the position on the field to 0,0 0-degrees, with forward being downfield. This resets
   * what "forward" is for field oriented driving.
   */
  public void resetFieldPosition() {
    setCurrentPose(new Pose2d());
  }

}