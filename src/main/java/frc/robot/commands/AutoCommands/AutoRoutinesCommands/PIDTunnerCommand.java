// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoCommands.AutoRoutinesCommands;

import com.pathplanner.lib.PathPlanner;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.AutoConstants;
import frc.robot.commands.AutoCommands.IdleArm;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.WristSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class PIDTunnerCommand extends SequentialCommandGroup {
  /** Creates a new PIDTunnerCommand. */
  DriveTrain m_drive;
  ArmSubsystem m_arm;
  WristSubsystem m_wrist;

  Trajectory pidTunerTrajectory = PathPlanner.loadPath("PIDTuner", AutoConstants.kMaxSpeedMetersPerSecond, 
  AutoConstants.kMaxAccelerationMetersPerSecondSquared);

  public PIDTunnerCommand(DriveTrain m_drive, ArmSubsystem m_arm, WristSubsystem m_wrist) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    InstantCommand resetOdometry = new InstantCommand(() -> m_drive.resetOdometry(pidTunerTrajectory.getInitialPose()));

    addCommands(resetOdometry, new IdleArm(m_arm, m_wrist), 
    m_drive.createCommandForTrajectory(pidTunerTrajectory, false)
    .andThen(() -> m_drive.tankDriveVolts(0, 0)));
  }
}
