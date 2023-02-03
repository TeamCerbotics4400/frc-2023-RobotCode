// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoCommands;

import com.pathplanner.lib.PathPlanner;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.DriveTrain;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SlamTestCommand extends SequentialCommandGroup {
  /** Creates a new SlamTestCommand. */
  Trajectory slamTrajectory = PathPlanner.loadPath("Slam Test",
   AutoConstants.kMaxSpeedMetersPerSecond, AutoConstants.kMaxAccelerationMetersPerSecondSquared);

  public SlamTestCommand(DriveTrain m_drive) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    InstantCommand resetOdometry = new InstantCommand(
      () -> m_drive.resetOdometry(slamTrajectory.getInitialPose()));

    addCommands(resetOdometry, 
    m_drive.createCommandForTrajectory(slamTrajectory, true).andThen(
      () -> m_drive.tankDriveVolts(0, 0)));
  }
}
