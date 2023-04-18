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
import team4400.StateMachines;
import team4400.StateMachines.IntakeState;

public class StateIntake extends CommandBase {
  /** Creates a new StateIntakeCommand. */
  FalconShooter m_shooter;
  ArmSubsystem m_arm;
  NodeSelector m_selector;
  IntakeState state;
  //Joystick intakeJoystick;
  //Joystick shootingJoystick;

  Timer rumbleTimer = new Timer();

  public StateIntake(FalconShooter m_shooter, ArmSubsystem m_arm, IntakeState state) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_shooter = m_shooter;
    this.m_arm = m_arm;
    this.state = state;

    addRequirements(m_shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_shooter.setCurrentLimit(16, 0.1);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if(StateMachines.getIntakeState() != IntakeState.FULL){
      StateMachines.setIntakeState(state);

    if(m_shooter.needToStop() && m_arm.isInIntakingPos()){
      StateMachines.setIntakeFull();
    }
  }

    switch(StateMachines.getIntakeState().toString()){
      case "IDLE":
        m_shooter.setMotorsPower(0, 0, 0);
        rumbleTimer.stop();
        rumbleTimer.reset();
      break;

      case "INTAKING":
      m_shooter.leftSetpoint(-6000);
      m_shooter.rightSetpoint(-6000);
      m_shooter.horizontalSetpoint(-6000);
       //m_shooter.setMotorsPower(-1, -1);
      break;

      case "FULL":
       m_shooter.setMotorsPower(0, 0, 0);
       rumbleTimer.start();

       /*if(rumbleTimer.get() < 1){
        intakeJoystick.setRumble(RumbleType.kBothRumble, 1);
       } else {
        intakeJoystick.setRumble(RumbleType.kBothRumble, 0);
       }*/
      break;
      
    }

    //SmartDashboard.putString("Current Intake State", StateMachines.getIntakeState().toString());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_shooter.setMotorsPower(0, 0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(DriverStation.isAutonomous()){
      if(StateMachines.getIntakeState() == IntakeState.FULL){
        return true;
      } else{
        return false;
      }
    } 
    return false;
  }
}
