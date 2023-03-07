// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoCommands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.WristConstants;
import frc.robot.commands.CubeShooter;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.FalconShooter;
import frc.robot.subsystems.WristSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ShootCube extends ParallelCommandGroup {
  /** Creates a new ShootCube. */
  FalconShooter m_shooter;
  ArmSubsystem m_arm;
  WristSubsystem m_wrist;

  public ShootCube(FalconShooter m_shooter, ArmSubsystem m_arm, WristSubsystem m_wrist) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    this.m_arm = m_arm;
    this.m_shooter = m_shooter;
    this.m_wrist = m_wrist;

    addCommands(m_arm.goToPosition(ArmConstants.SCORING_POSITION), 
    m_wrist.goToPosition(WristConstants.LEFT_POSITION), 
    new CubeShooter(m_shooter, m_arm, m_wrist));
  }
}
