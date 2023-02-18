// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.io.IOException;
import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants.VisionConstants;

/** Add your docs here. */
public class PhotonCameraWrapper {
    public PhotonCamera photonCamera;
    //public RobotPoseEstimator poseEstimator;
    public PhotonPoseEstimator photonPoseEstimator;
    
    public PhotonCameraWrapper() {

        photonCamera = new PhotonCamera(VisionConstants.cameraName);
        
        try{
            AprilTagFieldLayout fieldLayout = AprilTagFields.k2023ChargedUp.loadAprilTagLayoutField();

            photonPoseEstimator = new PhotonPoseEstimator(fieldLayout, 
                PoseStrategy.MULTI_TAG_PNP, photonCamera, VisionConstants.robotToCam);
            photonPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.CLOSEST_TO_REFERENCE_POSE);
        } catch(IOException e){
            DriverStation.reportError("Failed to load ApriltagFieldLayout", e.getStackTrace());
            photonPoseEstimator = null;
        }
    }

    public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d prevEstimatedPose){
        if(photonPoseEstimator == null){
            return Optional.empty();
        }
        photonPoseEstimator.setReferencePose(prevEstimatedPose);
        return photonPoseEstimator.update();
    }

    public double getTargetYaw(){
        return photonCamera.getLatestResult().getBestTarget().getYaw();
    }

    public PhotonPipelineResult getLatestResult(){
        return photonCamera.getLatestResult();
    }

    public List<PhotonTrackedTarget> getTargets(){
        return getLatestResult().getTargets();
    }
}
