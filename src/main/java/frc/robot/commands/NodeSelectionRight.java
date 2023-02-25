// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants.FieldConstants;
import frc.robot.subsystems.NodeSelector;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class NodeSelectionRight extends InstantCommand {
  NodeSelector m_nodeSelector;

  public NodeSelectionRight(NodeSelector m_nodeSelector) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_nodeSelector = m_nodeSelector;
    
    addRequirements(m_nodeSelector);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_nodeSelector.updateSelectionRight();
  }
}
