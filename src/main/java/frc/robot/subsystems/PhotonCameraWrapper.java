// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.Optional;

import org.photonvision.PhotonCamera;
import org.photonvision.RobotPoseEstimator;
import org.photonvision.RobotPoseEstimator.PoseStrategy;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.VisionConstants;

/** Add your docs here. */
public class PhotonCameraWrapper {
    public PhotonCamera photonCamera;
    public RobotPoseEstimator poseEstimator;
    
    public PhotonCameraWrapper() {
        final AprilTag tag6 = new AprilTag(6, 
        new Pose3d(new Pose2d(FieldConstants.length, FieldConstants.width / 2.0, 
        Rotation2d.fromDegrees(0))));
        final AprilTag tag7 = new AprilTag(7, 
        new Pose3d(new Pose2d(FieldConstants.length, FieldConstants.width / 2.0, 
        Rotation2d.fromDegrees(0))));
        final AprilTag tag8 = new AprilTag(9, 
        new Pose3d(new Pose2d(FieldConstants.length, FieldConstants.width / 2.0, 
        Rotation2d.fromDegrees(0))));

        ArrayList<AprilTag> atList = new ArrayList<AprilTag>();
        atList.add(tag6);
        atList.add(tag7);
        atList.add(tag8);

        AprilTagFieldLayout atfl = new AprilTagFieldLayout(atList, 
        FieldConstants.length, FieldConstants.width);

        photonCamera = new PhotonCamera(VisionConstants.cameraName);

        var camList = new ArrayList<Pair<PhotonCamera, Transform3d>>();
        camList.add(new Pair<PhotonCamera, Transform3d>(photonCamera, VisionConstants.robotToCam));

        poseEstimator = new RobotPoseEstimator(atfl, 
        PoseStrategy.CLOSEST_TO_REFERENCE_POSE, camList);
    }

    public Pair<Pose2d, Double> getEstimatedGlobalPose(Pose2d prevEstimatedPose){
        poseEstimator.setReferencePose(prevEstimatedPose);

        double currentTime = Timer.getFPGATimestamp();
        Optional<Pair<Pose3d, Double>>result = poseEstimator.update();

        if(result.isPresent()){
            return new Pair<Pose2d, Double>(result.get().getFirst().toPose2d(), 
            currentTime - result.get().getSecond());
        } else{
            return new Pair<Pose2d, Double>(new Pose2d(0, 0, new Rotation2d()), 0.0);
        }
    }
}
