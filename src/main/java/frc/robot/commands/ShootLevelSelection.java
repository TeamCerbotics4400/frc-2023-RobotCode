// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.WristConstants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.NodeSelector;
import frc.robot.subsystems.WristSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ShootLevelSelection extends ParallelCommandGroup {
  /** Creates a new ShootLevelSelection. */
  ArmSubsystem m_arm;
  WristSubsystem m_wrist;
  NodeSelector m_nodeSelector;

  public ShootLevelSelection(ArmSubsystem m_arm, WristSubsystem m_wrist, NodeSelector m_nodeSelector) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    this.m_arm = m_arm;
    this.m_wrist = m_wrist;
    this.m_nodeSelector = m_nodeSelector;

    switch(m_nodeSelector.getLevelName()){
      case "Low":
       addCommands(m_arm.goToPosition(ArmConstants.FRONT_FLOOR_POSITION), 
       m_wrist.goToPosition(WristConstants.LEFT_POSITION));
      break;
  
      case "Mid":
      addCommands(m_arm.goToPosition(ArmConstants.SCORING_POSITION), 
       m_wrist.goToPosition(WristConstants.LEFT_POSITION));
      break;
  
      case "High":
      addCommands(m_arm.goToPosition(ArmConstants.SCORING_POSITION), 
       m_wrist.goToPosition(WristConstants.LEFT_POSITION));
      break;
    }
  }
}
