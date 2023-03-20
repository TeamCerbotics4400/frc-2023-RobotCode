// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.FalconShooter;
import frc.robot.subsystems.NodeSelector;
import frc.robot.subsystems.WristSubsystem;

public class CubeShooter extends CommandBase {
  /** Creates a new DefaultShooter. */
  private FalconShooter shooter;
  private ArmSubsystem m_arm;
  private WristSubsystem m_wrist;
  private NodeSelector m_selector;

  public CubeShooter(FalconShooter shooter, ArmSubsystem m_arm, WristSubsystem m_wrist, 
  NodeSelector m_selector) {
    this.shooter = shooter;
    this.m_arm = m_arm;
    this.m_wrist = m_wrist;
    this.m_selector = m_selector;
    
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
      switch(m_selector.getLevelName()){
        case "Low":
          shooter.leftSetpoint(500);
          shooter.rightSetpoint(500);
        break;
        
        case "Mid":
          shooter.leftSetpoint(800);
          shooter.rightSetpoint(800);
        break;
        
        case "High":
          shooter.leftSetpoint(1200);
          shooter.rightSetpoint(1200);
      }
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
    if(DriverStation.isAutonomous()){
      if(!shooter.isShooterOcuppiedCube()){
        return true;
      } else{
        return false;
      }
    } 
    return false;
  }
}
