// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoCommands;

import java.util.function.BooleanSupplier;

import com.pathplanner.lib.PathPlanner;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.WristConstants;
import frc.robot.commands.AutoBalance;
import frc.robot.commands.CubeShooter;
import frc.robot.commands.ShootCube;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.FalconShooter;
import frc.robot.subsystems.WristSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class PieceWBalance extends SequentialCommandGroup {
  /** Creates a new OnlyBalanceAutoCommand. */
  Trajectory onlyBalanceTrajectory = PathPlanner.loadPath("Only Balance", 
    AutoConstants.kMaxSpeedMetersPerSecond, 
    AutoConstants.kMaxAccelerationMetersPerSecondSquared, true);
    DriveTrain m_drive;
    ArmSubsystem m_arm;
    WristSubsystem m_wrist;
    FalconShooter m_shooter;

  public PieceWBalance(DriveTrain m_drive, ArmSubsystem m_arm, WristSubsystem m_wrist, FalconShooter m_shooter) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    this.m_drive = m_drive;
    this.m_arm = m_arm;
    this.m_wrist = m_wrist;
    this.m_shooter = m_shooter;

    BooleanSupplier noPiece = new BooleanSupplier() {
      @Override
      public boolean getAsBoolean() {
          return m_shooter.isShooterOcuppiedCube();
      }
  };

    InstantCommand resetOdometry = 
    new InstantCommand(() -> m_drive.resetOdometry(onlyBalanceTrajectory.getInitialPose()));

    addCommands(resetOdometry,
    new ShootCube(m_shooter, m_arm, m_wrist).raceWith(new WaitCommand(4)), m_arm.goToPosition(ArmConstants.IDLE_POSITION).alongWith(
    m_wrist.goToPosition(WristConstants.IDLE_POSITION)),
      m_drive.createCommandForTrajectory(onlyBalanceTrajectory, false), 
      new AutoBalance(m_drive).andThen(() -> m_drive.tankDriveVolts(0, 0)));
  }
}
