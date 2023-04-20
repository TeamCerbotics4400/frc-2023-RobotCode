// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoCommands.AutoRoutinesCommands.Blue;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.AutoCommands.AutoBalance;
import frc.robot.commands.AutoCommands.IdleArm;
import frc.robot.commands.AutoCommands.ShootCube;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.FalconShooter;
import frc.robot.subsystems.NodeSelector;
import frc.robot.subsystems.WristSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class BluePieceBalance extends SequentialCommandGroup {
  /** Creates a new OnlyBalanceAutoCommand. */
  PathPlannerTrajectory onlyBalanceTrajectory = PathPlanner.loadPath("OnlyBalanceBlue", 
    2.0, 
    0.75, true);
    DriveTrain m_drive;
    ArmSubsystem m_arm;
    WristSubsystem m_wrist;
    FalconShooter m_shooter;
    NodeSelector m_selector;

  public BluePieceBalance(DriveTrain m_drive, ArmSubsystem m_arm, WristSubsystem m_wrist,
   FalconShooter m_shooter, NodeSelector m_selector) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    this.m_drive = m_drive;
    this.m_arm = m_arm;
    this.m_wrist = m_wrist;
    this.m_shooter = m_shooter;
    this.m_selector = m_selector;

    InstantCommand resetOdometry = 
    new InstantCommand(() -> m_drive.resetOdometry(onlyBalanceTrajectory.getInitialPose()));

    InstantCommand setCurrent = new InstantCommand(() -> m_drive.setDriveCurrentLimit(75));

    InstantCommand shootHigh = new InstantCommand(() -> m_selector.selectLevel(2));

    addCommands(setCurrent, resetOdometry, shootHigh,
    new ShootCube(m_shooter, m_arm, m_wrist, m_selector), 
      new IdleArm(m_arm, m_wrist),
      m_drive.createCommandForTrajectoryVision(onlyBalanceTrajectory),
      new AutoBalance(m_drive));
  }
}