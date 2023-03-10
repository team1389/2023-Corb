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
    // Cam mounted facing left, 0.1559 meters behind center, 0.1397 meters left of
    // center, 0.536 meters above center
    public final Transform3d robotToCamTransformation = new Transform3d(new Translation3d(-0.1559, -0.1397, 0.535686),
            new Rotation3d(0, 0, 90));
    private PhotonPoseEstimator photonPoseEstimator;

    public Vision() {
        atList.add(tag01);

        try {
            aprilTagFieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2023ChargedUp.m_resourceFile);
            
        } catch (IOException e) {
            aprilTagFieldLayout = new AprilTagFieldLayout(atList, FieldConstants.FIELD_LENGTH, FieldConstants.FIELD_WIDTH);
            e.printStackTrace();
        }

        photonPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.CLOSEST_TO_REFERENCE_POSE, camera, robotToCamTransformation);
    }

    public void updatePose(Drivetrain drivetrain) {
        //Set reference pose of photonSwerveEstimator using swerve odometry, then update photonPoseEstimator
        photonPoseEstimator.setReferencePose(drivetrain.poseEstimator.getEstimatedPosition());
        Optional<EstimatedRobotPose> estimatedPose = photonPoseEstimator.update();

        if(estimatedPose.isPresent()) {
            Pose2d camPose = estimatedPose.get().estimatedPose.toPose2d();
            double timeStamp = estimatedPose.get().timestampSeconds;
            drivetrain.updateOdometryLatency(camPose, timeStamp);
        }
    }
}
