// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoCommands.AutoRoutinesCommands.Blue;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.DriveTrain;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class BlueLoadingTwoPieces extends SequentialCommandGroup {
  /** Creates a new BlueLoadingTwoPieces. */
  PathPlannerTrajectory loading2 = PathPlanner.loadPath("Loading2", 
  AutoConstants.kMaxSpeedMetersPerSecond, 
  AutoConstants.kMaxAccelerationMetersPerSecondSquared, true);

  public BlueLoadingTwoPieces(DriveTrain m_drive) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());

    InstantCommand resetOdometry = new InstantCommand(() -> 
    m_drive.resetOdometry(loading2.getInitialPose()));
    
    addCommands(resetOdometry, m_drive.createCommandForTrajectory(loading2, false));
  }
}
