// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoCommands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.WristConstants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.WristSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class IdleArm extends ParallelCommandGroup {
  /** Creates a new IdleArm. */
  ArmSubsystem m_arm;
  WristSubsystem m_wrist;
  public IdleArm(ArmSubsystem m_arm, WristSubsystem m_wrist) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    this.m_arm = m_arm;
    this.m_wrist = m_wrist;
    addCommands(m_arm.goToPosition(ArmConstants.IDLE_POSITION), m_wrist.goToPosition(WristConstants.IDLE_POSITION));
  }
}
