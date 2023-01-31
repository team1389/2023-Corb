package frc.subsystems;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap.FieldConstants;

import org.photonvision.PhotonPoseEstimator.PoseStrategy;

public class Vision extends SubsystemBase {
    public PhotonCamera camera = new PhotonCamera("OV5647");
    public double fieldLength = 10; // in meters
    public double fieldWidth = 10;
    public AprilTagFieldLayout aprilTagFieldLayout;

    final AprilTag tag01 =
    new AprilTag(01,
                new Pose3d(new Pose2d(0.0, FieldConstants.FIELD_WIDTH / 2.0, Rotation2d.fromDegrees(0.0))));
    public List<AprilTag> atList = new ArrayList<AprilTag>(); 
    // Cam mounted facing left, 0.197 meters behind center, 0.368 meters left of
    // center, 0.235 meters above center
    public final Transform3d robotToCamTransformation = new Transform3d(new Translation3d(-0.197, -0.368, 0.235),
            new Rotation3d(0, 0, 90));
    private PhotonPoseEstimator photonPoseEstimator;

    public Vision() {

        try {
            aprilTagFieldLayout = new AprilTagFieldLayout("2023-chargedup.json");
            
        } catch (IOException e) {
            // TODO Auto-generated catch block
            aprilTagFieldLayout = new AprilTagFieldLayout(atList, FieldConstants.FIELD_LENGTH, FieldConstants.FIELD_WIDTH);
            e.printStackTrace();
        
        }

        photonPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.AVERAGE_BEST_TARGETS, camera, robotToCamTransformation);
    }

    public void update(Drivetrain drivetrain) {
        Optional<EstimatedRobotPose> pose = photonPoseEstimator.update();

        if(pose.isPresent()) {
            Pose3d estimatedPose = pose.get().estimatedPose;
            double timeStamp = pose.get().timestampSeconds;
            drivetrain.updateOdometryLatency(estimatedPose.toPose2d(), timeStamp);
        }
    }
}
