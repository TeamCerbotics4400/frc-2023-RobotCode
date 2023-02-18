// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Trajectories;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveTrain;

/** Add your docs here. */
public class TestAlignTrajectory {
    
    public static Trajectory generatedTrajectory(DriveTrain m_drive){

        TrajectoryConfig config =
            new TrajectoryConfig(
                    AutoConstants.kMaxSpeedMetersPerSecond,
                    AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                // Add kinematics to ensure max speed is actually obeyed
                .setKinematics(DriveConstants.kDriveKinematics);
    
        var currentPose = new Pose2d(m_drive.getEstimationTranslation(), m_drive.getEstimatioRotation());
    
        var middleGround = new Translation2d(4.00, 4.80);
    
        var goalPose = new Pose2d(1.78, 4.40, new Rotation2d(Units.degreesToRadians(180)));
    
        Trajectory exampleTrajectory =
        TrajectoryGenerator.generateTrajectory(currentPose, List.of(middleGround), goalPose, config);
    
        return exampleTrajectory;
      }
}
