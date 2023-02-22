// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoCommands;

import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.AutoConstants;
import frc.robot.commands.AutoAlign;
import frc.robot.subsystems.DriveTrain;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class DriveToPoseTest extends SequentialCommandGroup {
  /** Creates a new DriveToPoseTest. */
  private DriveTrain m_drive;
  private Pose2d m_pose;
  public DriveToPoseTest(String m_pose, Alliance alliance, DriveTrain m_drive) {
    this.m_drive = m_drive;
    this.m_pose = AutoConstants.POSE_MAP.get(alliance).get(m_pose);

    addRequirements(m_drive);

    addCommands(new FollowTrajectory(m_drive, this::generateTrajectory, false), new AutoAlign(m_drive)
    .andThen(() -> m_drive.tankDriveVolts(0, 0)));
  }

  private PathPlannerTrajectory generateTrajectory(){
    Pose2d currentPose = m_drive.getPose();
    Translation2d currentTranslation = 
        new Translation2d(currentPose.getX(), currentPose.getY());
    Translation2d desiredTranslation = 
        new Translation2d(m_pose.getX(), m_pose.getY());
    Rotation2d driveAngle = desiredTranslation.minus(currentTranslation).getAngle();
    PathPlannerTrajectory trajectory = 
    PathPlanner.generatePath(new PathConstraints(AutoConstants.kMaxSpeedMetersPerSecond, 
    AutoConstants.kMaxAccelerationMetersPerSecondSquared), 
    List.of(new PathPoint(currentTranslation, driveAngle, m_drive.getPose().getRotation()), 
    new PathPoint(desiredTranslation, driveAngle, m_pose.getRotation())));

    return trajectory;
  }
}
