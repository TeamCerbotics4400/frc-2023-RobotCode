// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.ArrayList;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.FalconShooter;
import frc.robot.subsystems.NodeSelector;
import frc.robot.subsystems.WristSubsystem;

public class CombinedShooter extends CommandBase {
  /** Creates a new CombinedShooter. */
  ArmSubsystem m_arm;
  WristSubsystem m_wrist;
  FalconShooter m_shooter;
  NodeSelector m_selector;

  private ArrayList<String> cubeNodes;
  private ArrayList<String> coneNodes;

  public CombinedShooter(ArmSubsystem m_arm, WristSubsystem m_wrist, FalconShooter m_shooter, 
  NodeSelector m_selector) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_arm = m_arm;
    this.m_wrist = m_wrist;
    this.m_shooter = m_shooter;
    this.m_selector = m_selector;

    cubeNodes = new ArrayList<String>();
    coneNodes = new ArrayList<String>();

    cubeNodes.add("Node 2");
    cubeNodes.add("Node 5");
    cubeNodes.add("Node 8");

    coneNodes.add("Node 1");
    coneNodes.add("Node 3");
    coneNodes.add("Node 4");
    coneNodes.add("Node 6");
    coneNodes.add("Node 7");
    coneNodes.add("Node 9");

    addRequirements(m_shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    switch(m_selector.getLevelName()){
      case "Low":
        m_shooter.setMotorsPower(0.35, 0.35);
      break;

      case "Mid":
        if(cubeNodes.contains(m_selector.getAlignName())){
      
          m_shooter.leftSetpoint(1150);
          m_shooter.rightSetpoint(1150);
        
        } else{
      
          m_shooter.leftSetpoint(1045);
          m_shooter.rightSetpoint(1045);
        }
      break;

      case "High":
      if(cubeNodes.contains(m_selector.getAlignName())){
      
        m_shooter.leftSetpoint(1500);
        m_shooter.rightSetpoint(1500);
      
      } else{
    
        m_shooter.leftSetpoint(1045);
        m_shooter.rightSetpoint(1045);
      }
      break;
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
    return false;
  }

  public boolean suitableToShoot(){
    if(RobotController.getBatteryVoltage() <= 12.0){
      return false;
    } else {
      return true;
    }
  }
}


