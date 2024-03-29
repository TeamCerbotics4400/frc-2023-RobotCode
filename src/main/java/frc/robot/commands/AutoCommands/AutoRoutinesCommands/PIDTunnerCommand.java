// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoCommands.AutoRoutinesCommands;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
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

  PathPlannerTrajectory pidTunerTrajectory = PathPlanner.loadPath("PIDTuner", 4.0, 
  2.0, false);

  public PIDTunnerCommand(DriveTrain m_drive, ArmSubsystem m_arm, WristSubsystem m_wrist) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    InstantCommand resetOdometry = new InstantCommand(() -> 
    m_drive.resetOdometry(pidTunerTrajectory.getInitialPose()));

    addCommands( new IdleArm(m_arm, m_wrist), 
    m_drive.createCommandForTrajectoryVision(pidTunerTrajectory)
    .andThen(() -> m_drive.tankDriveVolts(0, 0)));
  }
}
