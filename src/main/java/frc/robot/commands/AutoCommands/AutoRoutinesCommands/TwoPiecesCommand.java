// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoCommands.AutoRoutinesCommands;

import java.util.HashMap;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.FollowPathWithEvents;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
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
import frc.robot.subsystems.NodeSelector;
import frc.robot.subsystems.WristSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TwoPiecesCommand extends SequentialCommandGroup {
  /** Creates a new TwoPiecesWBalanceCommand. */
  PathPlannerTrajectory twoPiecesTrajectory = PathPlanner.loadPath("TwoPieces", 
  AutoConstants.kMaxSpeedMetersPerSecond,
  AutoConstants.kMaxAccelerationMetersPerSecondSquared, true);

  HashMap<String, Command> eventMap = new HashMap<>();

  public TwoPiecesCommand(DriveTrain m_drive, ArmSubsystem m_arm, 
  WristSubsystem m_wrist, FalconShooter m_shooter, NodeSelector m_selector) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());

    InstantCommand resetOdometry = new InstantCommand(() ->
     m_drive.resetOdometry(twoPiecesTrajectory.getInitialPose()));

    InstantCommand setLevelLow = new InstantCommand(() -> m_selector.selectLevel(0));
    InstantCommand setLevelMid = new InstantCommand(() -> m_selector.selectLevel(1));

    eventMap.put("Shoot", new ShootCube(m_shooter, m_arm, m_wrist, m_selector));
    eventMap.put("Idle", new IdleArm(m_arm, m_wrist));
    eventMap.put("Intake", new IntakeCube(m_shooter, m_arm, m_wrist));
    eventMap.put("Mid", setLevelMid);

    addCommands(resetOdometry, 
     new ShootCone(m_shooter, m_arm, m_wrist).andThen(new IdleArm(m_arm, m_wrist)), 
     new FollowPathWithEvents(m_drive.createCommandForTrajectory(twoPiecesTrajectory, 
     false), twoPiecesTrajectory.getMarkers(), eventMap), 
     new ShootCube(m_shooter, m_arm, m_wrist, m_selector).andThen(new IdleArm(m_arm, m_wrist)));
  }
}
