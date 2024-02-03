package frc.robot.subsystems;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/*
 * TODO: functions based on analyisis of apriltag data (pos data)
 * TODO: function that outputs path data for navigation of current pos to new pos
 * 
 */


public class VisionSubsystem extends SubsystemBase {
    
    private DriveSubsystem m_DriveSubsystem;

    PhotonCamera driverCamera = new PhotonCamera("Driver Camera");
    Transform3d driverTransform = new Transform3d(new Translation3d(0.4, 0, 0.4), new Rotation3d(0,0,0));
    PhotonCamera noteCamera = new PhotonCamera("Note Detection");
    Transform3d noteTransform = new Transform3d(new Translation3d(-0.4,0,0.15), new Rotation3d(0,0,0));

    EstimatedRobotPose robotPose = new EstimatedRobotPose(null, 0, null, null);

    AprilTagFieldLayout aprilTagFieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
    PhotonPoseEstimator poseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, driverCamera, driverTransform );

    public VisionSubsystem(DriveSubsystem drive) {
        m_DriveSubsystem = drive;
    }

    public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
        poseEstimator.setReferencePose(prevEstimatedRobotPose);
        return poseEstimator.update();

    }
    
}
