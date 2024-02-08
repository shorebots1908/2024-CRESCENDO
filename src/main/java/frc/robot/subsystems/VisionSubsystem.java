package frc.robot.subsystems;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.PhotonUtils;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.apriltag.AprilTagPoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/*
 * TODO: functions based on analyisis of apriltag data (pos data)
 * TODO: function that outputs path data for navigation of current pos to new pos
 * 
 */


public class VisionSubsystem extends SubsystemBase {
    
    private static final double CAMERA_HEIGHT_METERS = 0.4;

    private static final double CAMERA_PITCH_RADIANS = 0;

    private static final double TARGET_HEIGHT_METERS = 0.4;

    private DriveSubsystem m_DriveSubsystem;
    private boolean noteVisible = false;
    private boolean apriltagVisible = false;
    PhotonCamera driverCamera = new PhotonCamera("Driver Camera");
    Transform3d driverTransform = new Transform3d(new Translation3d(0.4, 0, 0.4), new Rotation3d(0,0,0));
    PhotonCamera noteCamera = new PhotonCamera("Note Detection");
    Transform3d noteTransform = new Transform3d(new Translation3d(-0.4,0,0.15), new Rotation3d(0,0,0));
    
    EstimatedRobotPose robotPose = new EstimatedRobotPose(null, 0, null, null);

    AprilTagFieldLayout aprilTagFieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
    PhotonPoseEstimator poseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, driverCamera, driverTransform );
    
    public void periodic()
    {

    }



    public VisionSubsystem(DriveSubsystem drive) {
        m_DriveSubsystem = drive;
        
    }

    public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
        poseEstimator.setReferencePose(prevEstimatedRobotPose);
        return poseEstimator.update();

    }

    //need to be implemented
    public boolean noteVisible(){
        var result1 = noteCamera.getLatestResult();
        if (result1.hasTargets()) {
                    // First calculate range
          double range =
                PhotonUtils.calculateDistanceToTargetMeters(
                        CAMERA_HEIGHT_METERS,
                        TARGET_HEIGHT_METERS,
                        CAMERA_PITCH_RADIANS,
                        Units.degreesToRadians(result1.getBestTarget().getPitch()));

         
        }
        return noteVisible = true;
    }
       

    public boolean apriltagVisible(){
        var result2 = driverCamera.getLatestResult();
        if (result2.hasTargets()) {
                    // First calculate range
          double range =
                PhotonUtils.calculateDistanceToTargetMeters(
                        CAMERA_HEIGHT_METERS,
                        TARGET_HEIGHT_METERS,
                        CAMERA_PITCH_RADIANS,
                        Units.degreesToRadians(result2.getBestTarget().getPitch()));

        
        }
        return apriltagVisible = true;
        }
        

        
    

        public Command waitForRing() {
        return Commands.waitUntil(() -> noteVisible());
    }
    





}