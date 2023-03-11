// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoCommands.AutoRoutinesCommands;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.WristConstants;
import frc.robot.commands.AutoCommands.IdleArm;
import frc.robot.commands.AutoCommands.IntakeCube;
import frc.robot.commands.AutoCommands.ShootCone;
import frc.robot.commands.AutoCommands.ShootCube;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.FalconShooter;
import frc.robot.subsystems.WristSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TwoPiecesCommand extends SequentialCommandGroup {
  /** Creates a new TwoPiecesWBalanceCommand. */
  PathPlannerTrajectory twoPiecesTrajectory = PathPlanner.loadPath("TwoPieces", 
  AutoConstants.kMaxSpeedMetersPerSecond,
  AutoConstants.kMaxAccelerationMetersPerSecondSquared, true);

  DriveTrain m_drive;
  ArmSubsystem m_arm;
  WristSubsystem m_wrist;

  public TwoPiecesCommand(DriveTrain m_drive, ArmSubsystem m_arm, 
  WristSubsystem m_wrist, FalconShooter m_shooter) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());

    PathPlannerTrajectory.transformTrajectoryForAlliance(twoPiecesTrajectory, DriverStation.getAlliance());

    InstantCommand resetOdometry = new InstantCommand(() ->
     m_drive.resetOdometry(twoPiecesTrajectory.getInitialPose()));

    addCommands(resetOdometry, new ShootCone(m_shooter, m_arm, m_wrist).raceWith(new WaitCommand(4)),
     m_arm.goToPosition(ArmConstants.IDLE_POSITION).alongWith(m_wrist.goToPosition(WristConstants.IDLE_POSITION)),
    m_drive.createCommandForTrajectory(twoPiecesTrajectory, false).alongWith(
    new IntakeCube(m_shooter, m_arm, m_wrist))
     .andThen(() -> m_drive.tankDriveVolts(0, 0)), 
     new ShootCube(m_shooter, m_arm, m_wrist).raceWith(new WaitCommand(4)).andThen(new IdleArm(m_arm, m_wrist)));
  }
}
