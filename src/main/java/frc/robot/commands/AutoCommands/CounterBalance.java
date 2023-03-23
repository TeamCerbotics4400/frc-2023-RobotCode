// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoCommands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.WristConstants;
import frc.robot.commands.CubeShooter;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.FalconShooter;
import frc.robot.subsystems.NodeSelector;
import frc.robot.subsystems.WristSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class CounterBalance extends ParallelCommandGroup {
  /** Creates a new ShootCube. */
  ArmSubsystem m_arm;
  WristSubsystem m_wrist;
  NodeSelector m_selector;

  public CounterBalance(ArmSubsystem m_arm, WristSubsystem m_wrist, 
  NodeSelector m_selector) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    this.m_arm = m_arm;
    this.m_wrist = m_wrist;
    this.m_selector = m_selector;

    addCommands(m_arm.goToPosition(ArmConstants.COUNTER_BALANCE_POSITION), 
    m_wrist.goToPosition(WristConstants.RIGHT_POSITION));
  }
}