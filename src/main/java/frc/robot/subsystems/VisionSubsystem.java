// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.LimelightHelpers.LimelightResults;
import frc.robot.LimelightHelpers.LimelightTarget_Fiducial;

public class VisionSubsystem extends SubsystemBase {
  /** Creates a new VisionSubsystem. */
  private LimelightResults limelightTagsResults;

  public VisionSubsystem() {
    limelightTagsResults = LimelightHelpers.getLatestResults(VisionConstants.tagLimelightName);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    
  }

  public static LimelightResults getLimelightResults(){
    LimelightResults limelightFudicialResults =
     LimelightHelpers.getLatestResults(VisionConstants.tagLimelightName);
  
    return limelightFudicialResults;
  }

  public static LimelightTarget_Fiducial[] getFudicialTargets(){
    LimelightResults limelightFudicialResults =
     LimelightHelpers.getLatestResults(VisionConstants.tagLimelightName);
    LimelightTarget_Fiducial[] limelightApriltags = 
    limelightFudicialResults.targetingResults.targets_Fiducials;

    return limelightApriltags;
  }

  public static Pose2d getPoseFromAprilTags() {
    Pose2d botPose = LimelightHelpers.getBotPose2d(VisionConstants.tagLimelightName);
    // The origin of botpose is at the center of the field
    double robotX = botPose.getX() + FieldConstants.FIELD_LENGTH_METERS / 2;
    double robotY = botPose.getY() + FieldConstants.FIELD_WIDTH_METERS / 2;
    Rotation2d robotRotation = botPose.getRotation();
    return new Pose2d(robotX, robotY, robotRotation);
  }
}
