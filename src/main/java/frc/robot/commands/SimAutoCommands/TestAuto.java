// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.SimAutoCommands;

import com.pathplanner.lib.PathPlanner;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.SimulationConstants;
import frc.robot.subsystems.DrivetrainSim;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TestAuto extends SequentialCommandGroup {
  /** Creates a new TestAuto. */
  public TestAuto(DrivetrainSim simDrive) {
    Trajectory trajectory = PathPlanner.loadPath("Test Path", 
    SimulationConstants.kMaxSpeedMetersPerSecond, 
    SimulationConstants.kMaxAccelerationMetersPerSecondSquared, true);

    Command resetOdo = new InstantCommand(() -> simDrive.resetOdometry(trajectory.getInitialPose()));
    
    addCommands(resetOdo, 
    new ParallelRaceGroup(simDrive.createCommandForTrajectory(trajectory, true)));
  }
}
