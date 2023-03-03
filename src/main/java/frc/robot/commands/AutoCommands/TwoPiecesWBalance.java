// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoCommands;

import com.pathplanner.lib.PathPlanner;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.AutoConstants;
import frc.robot.commands.AutoBalance;
import frc.robot.subsystems.DriveTrain;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TwoPiecesWBalance extends SequentialCommandGroup {
  /** Creates a new TwoPiecesWBalance. */
  Trajectory firstTrajectory = PathPlanner.loadPath("TwoPiecesBalanceOne", 
  AutoConstants.kMaxSpeedMetersPerSecond, 
  AutoConstants.kMaxAccelerationMetersPerSecondSquared, true);

  Trajectory secondTrajectory = PathPlanner.loadPath("TwoPiecesBalanceTwo", 
  AutoConstants.kMaxSpeedMetersPerSecond, 
  AutoConstants.kMaxAccelerationMetersPerSecondSquared, false);

  Trajectory thirdTrajectory = PathPlanner.loadPath("TwoPiecesBalanceThree", 
  2.0, 2.0, false);

  public TwoPiecesWBalance(DriveTrain m_drive) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());

    InstantCommand resetOdometry1 = new InstantCommand(() -> 
    m_drive.resetOdometry(firstTrajectory.getInitialPose()));

    InstantCommand resetOdometry2 = new InstantCommand(() -> 
    m_drive.resetOdometry(secondTrajectory.getInitialPose()));

    InstantCommand resetOdometry3 = new InstantCommand(() -> 
    m_drive.resetOdometry(thirdTrajectory.getInitialPose()));

    addCommands(resetOdometry1, 
    m_drive.createCommandForTrajectory(firstTrajectory, false).andThen(() -> 
    m_drive.tankDriveVolts(0, 0)), new WaitCommand(2), 
    resetOdometry2, 
    m_drive.createCommandForTrajectory(secondTrajectory, false).andThen(() -> 
    m_drive.tankDriveVolts(0, 0)), new WaitCommand(2), 
    resetOdometry3,
    m_drive.createCommandForTrajectory(thirdTrajectory, false), new AutoBalance(m_drive));
  }
}
