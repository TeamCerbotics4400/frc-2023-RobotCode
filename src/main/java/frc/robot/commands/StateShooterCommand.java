// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.FalconShooter;
import frc.robot.subsystems.NodeSelector;
import frc.robot.subsystems.WristSubsystem;
import team4400.StateMachines;
import team4400.StateMachines.IntakeState;

public class StateShooterCommand extends CommandBase {
  /** Creates a new StateIntakeCommand. */
  FalconShooter m_shooter;
  ArmSubsystem m_arm;
  WristSubsystem m_wrist;
  NodeSelector m_selector;
  IntakeState state;

  Timer rumbleTimer = new Timer();

  public StateShooterCommand(FalconShooter m_shooter, ArmSubsystem m_arm, WristSubsystem m_wrist,
  IntakeState state, NodeSelector m_selector) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_shooter = m_shooter;
    this.m_wrist = m_wrist;
    this.m_selector = m_selector;
    this.m_arm = m_arm;
    this.state = state;
    addRequirements(m_shooter);
  } 

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(DriverStation.isAutonomous()){
     if(m_arm.isReady() && m_wrist.isReady()){
      StateMachines.setIntakeState(state);

    switch(StateMachines.getIntakeState().toString()){
      case "SHOOTING":
       //new CombinedShooter(m_shooter, m_selector);
       switch(m_selector.getLevelName()){
        case "Low":
            m_shooter.leftSetpoint(500);
            m_shooter.rightSetpoint(500);
            m_shooter.horizontalSetpoint(650);
        break;
  
        case "Mid":
            //800 RPM for cube
            //m_shooter.goToDashboardVelocity();
            m_shooter.leftSetpoint(1200);
            m_shooter.rightSetpoint(1200);
            m_shooter.horizontalSetpoint(1200);
        break;
  
        case "High":
          //m_shooter.goToDashboardVelocity();
          m_shooter.leftSetpoint(1600);
          m_shooter.rightSetpoint(1600);
          m_shooter.horizontalSetpoint(2100);
        break;
  
        case "Ave Maria":
          m_shooter.leftSetpoint(6000);
          m_shooter.rightSetpoint(6000);
          m_shooter.horizontalSetpoint(6000);
      }
      break;
    }
     } else {
      m_shooter.setMotorsPower(0, 0, 0);
     }
    } else {
      StateMachines.setIntakeState(state);

    switch(StateMachines.getIntakeState().toString()){
      case "SHOOTING":
       //new CombinedShooter(m_shooter, m_selector);
       switch(m_selector.getLevelName()){
        case "Low":
            m_shooter.leftSetpoint(500);
            m_shooter.rightSetpoint(500);
            m_shooter.horizontalSetpoint(650);
        break;
  
        case "Mid":
            //800 RPM for cube
            //m_shooter.goToDashboardVelocity();
            m_shooter.leftSetpoint(1200);
            m_shooter.rightSetpoint(1200);
            m_shooter.horizontalSetpoint(1200);
        break;
  
        case "High":
          //m_shooter.goToDashboardVelocity();
          m_shooter.leftSetpoint(1600);//2300
          m_shooter.rightSetpoint(1600);//2300
          m_shooter.horizontalSetpoint(2100);//2800
        break;
  
        case "Ave Maria":
          m_shooter.leftSetpoint(6000);
          m_shooter.rightSetpoint(6000);
          m_shooter.horizontalSetpoint(6000);
      }
      break;
     }
    }
    
    SmartDashboard.putString("Current Intake State", StateMachines.getIntakeState().toString());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_shooter.setMotorsPower(0, 0, 0);
    StateMachines.setIntakeIdle();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(DriverStation.isAutonomous()){
      //Timer.delay(0.5);
      if(m_arm.isInShootingPos() && m_shooter.hasAlreadyShot() && StateMachines.isShooting()){
        return true;
      } else {
        return false;
      }
    } 
    return false;
  }
}
