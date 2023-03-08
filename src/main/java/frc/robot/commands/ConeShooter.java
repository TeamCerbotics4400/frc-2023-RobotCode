// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.FalconShooter;
import frc.robot.subsystems.WristSubsystem;

public class ConeShooter extends CommandBase {
  /** Creates a new ConeShooter. */
  FalconShooter m_shooter;
  ArmSubsystem m_arm;
  WristSubsystem m_wrist;
  public ConeShooter(FalconShooter m_shooter, ArmSubsystem m_arm, WristSubsystem m_wrist) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_shooter = m_shooter;
    this.m_arm = m_arm;
    this.m_wrist = m_wrist;

    addRequirements(m_shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(m_arm.isReady() && m_wrist.isReady()){
      //shooter.goToDashboardVelocity();
      m_shooter.leftSetpoint(1075);
    m_shooter.rightSetpoint(1075);
       } else {
        m_shooter.setMotorsPower(0, 0);
      }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_shooter.setMotorsPower(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    //if(DriverStation.isAutonomous()){
      if(!m_shooter.isShooterOcuppiedCube()){
        return true;
        } else{
          return false;
        }
   // }
    //return false;
  }
}
