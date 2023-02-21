// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Trajectories;

import java.util.ArrayList;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.DriveTrain;

/** Add your docs here. */
public class TestAlignTrajectory {
    public static Trajectory generatedTrajectory(DriveTrain m_drive){

        var trajectoryWaypoints = new ArrayList<Pose2d>();
    
        var currentPose = new Pose2d(m_drive.getEstimationTranslation(), m_drive.getEstimatioRotation());
        trajectoryWaypoints.add(currentPose);
    
        var middleGround = new Pose2d(3.34, 4.66, new Rotation2d(Units.degreesToRadians(180.00)));
        trajectoryWaypoints.add(middleGround);
    
        var goalPose = new Pose2d(1.78, 4.40, new Rotation2d(Units.degreesToRadians(180)));
        trajectoryWaypoints.add(goalPose);

        TrajectoryConfig config = new TrajectoryConfig(AutoConstants.kMaxSpeedMetersPerSecond, 
                AutoConstants.kMaxAccelerationMetersPerSecondSquared);
    
        var trajectory = TrajectoryGenerator.generateTrajectory(trajectoryWaypoints, config);
    
        return trajectory;
      }
}
