// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.LimelightHelpers.LimelightResults;
import frc.robot.LimelightHelpers.LimelightTarget_Fiducial;

public class VisionSubsystem extends SubsystemBase {
  /** Creates a new VisionSubsystem. */
  public static LimelightResults limelightTagsResults;

  public VisionSubsystem() {
    limelightTagsResults = LimelightHelpers.getLatestResults(VisionConstants.tagLimelightName);

    //Set tag Limelight pose
    LimelightHelpers.setCameraPose_RobotSpace(VisionConstants.tagLimelightName, 
                                    -0.04819, -0.133825, 0.4708, 0, 0, 0);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    
  }

  public static LimelightResults getLimelightResults(){
    return limelightTagsResults;
  }

  public static LimelightTarget_Fiducial[] getFudicialTargets(){

    LimelightTarget_Fiducial[] limelightApriltags = 
    getLimelightResults().targetingResults.targets_Fiducials;

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

  public static Pose3d getRobot3dPose(){
    Pose3d botPose3d = LimelightHelpers.getBotPose3d(VisionConstants.tagLimelightName);
    double robotX = botPose3d.getX() + FieldConstants.FIELD_LENGTH_METERS / 2;
    double robotY = botPose3d.getY() + FieldConstants.FIELD_WIDTH_METERS / 2;
    double robotZ = botPose3d.getZ();
    double robotRoll = botPose3d.getRotation().getX();
    double robotPitch = botPose3d.getRotation().getY();
    double robotYaw = botPose3d.getRotation().getZ();

    return new Pose3d(robotX, robotY, robotZ, new Rotation3d(robotRoll, robotPitch, robotYaw));
  }
}
