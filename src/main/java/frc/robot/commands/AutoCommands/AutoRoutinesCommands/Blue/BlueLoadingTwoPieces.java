// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoCommands.AutoRoutinesCommands.Blue;

import java.util.HashMap;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.FollowPathWithEvents;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.AutoConstants;
import frc.robot.commands.AutoCommands.AveMariaShoot;
import frc.robot.commands.AutoCommands.IdleArm;
import frc.robot.commands.AutoCommands.IntakeCube;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.FalconShooter;
import frc.robot.subsystems.NodeSelector;
import frc.robot.subsystems.WristSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class BlueLoadingTwoPieces extends SequentialCommandGroup {
  /** Creates a new BlueLoadingTwoPieces. */
  PathPlannerTrajectory loading2 = PathPlanner.loadPath("Loading2", 
  //5.0, 
  2.0,
  1.0,
  //3.0,
   true);

  HashMap<String, Command> eventMap = new HashMap<>();

  public BlueLoadingTwoPieces(DriveTrain m_drive, ArmSubsystem m_arm, WristSubsystem m_wrist, 
  FalconShooter m_shooter, NodeSelector m_selector) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());

    InstantCommand resetOdometry = new InstantCommand(() -> 
    m_drive.resetOdometry(loading2.getInitialPose()));

    InstantCommand aveMaria = new InstantCommand(
      () -> m_selector.selectLevel(3));

    eventMap.put("Shoot", new AveMariaShoot(m_shooter, m_arm, m_wrist, m_selector));
    eventMap.put("Idle", new IdleArm(m_arm, m_wrist));
    eventMap.put("Intake", new IntakeCube(m_shooter, m_arm, m_wrist));
    
    addCommands(resetOdometry, aveMaria, 
    new AveMariaShoot(m_shooter, m_arm, m_wrist, m_selector),
    new IdleArm(m_arm, m_wrist), //m_drive.createCommandForTrajectory(loading2));
    new FollowPathWithEvents(m_drive.createCommandForTrajectoryVision(loading2), 
                                                        loading2.getMarkers(), eventMap));
  }
}
