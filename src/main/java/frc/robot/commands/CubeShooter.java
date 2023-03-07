// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.WristConstants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.FalconShooter;
import frc.robot.subsystems.WristSubsystem;

public class CubeShooter extends CommandBase {
  /** Creates a new DefaultShooter. */
  private FalconShooter shooter;
  private ArmSubsystem m_arm;
  private WristSubsystem m_wrist;

  public CubeShooter(FalconShooter shooter, ArmSubsystem m_arm, WristSubsystem m_wrist) {
    this.shooter = shooter;
    this.m_arm = m_arm;
    this.m_wrist = m_wrist;
    
    addRequirements(shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //shooter.goToDashboardVelocity();
    //1500 RPM HIGH
    if(m_arm.isReady() && m_wrist.isReady()){
    //shooter.goToDashboardVelocity();
    shooter.leftSetpoint(1500);
    shooter.rightSetpoint(1500);
     } else {
      shooter.setMotorsPower(0, 0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.setMotorsPower(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(!shooter.isShooterOcuppiedCube()){
    return true;
    } else{
      return false;
    }
  }
}
