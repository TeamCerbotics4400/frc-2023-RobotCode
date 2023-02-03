// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.VisionConstants;

/** Add your docs here. */
public class PhotonCameraWrapper {
    public PhotonCamera photonCamera;
    //public RobotPoseEstimator poseEstimator;
    public PhotonPoseEstimator photonPoseEstimator;
    
    public PhotonCameraWrapper() {
        final AprilTag tag6 = new AprilTag(6, 
        new Pose3d(new Pose2d(Units.inchesToMeters(40.45), Units.inchesToMeters(174.19), 
        Rotation2d.fromDegrees(0))));
        final AprilTag tag7 = new AprilTag(7, 
        new Pose3d(new Pose2d(Units.inchesToMeters(40.45), Units.inchesToMeters(108.19), 
        Rotation2d.fromDegrees(0))));
        final AprilTag tag8 = new AprilTag(8, 
        new Pose3d(new Pose2d(Units.inchesToMeters(40.45), Units.inchesToMeters(42.19), 
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

        photonPoseEstimator = new PhotonPoseEstimator(atfl, 
        PoseStrategy.CLOSEST_TO_CAMERA_HEIGHT, photonCamera, VisionConstants.robotToCam);
    }

    public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d prevEstimatedPose){
        photonPoseEstimator.setReferencePose(prevEstimatedPose);
        return photonPoseEstimator.update();
    }

    public double getTargetYaw(){
        return photonCamera.getLatestResult().getBestTarget().getYaw();
    }
}
