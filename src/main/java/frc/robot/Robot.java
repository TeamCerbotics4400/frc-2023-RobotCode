// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.server.PathPlannerServer;

import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import team4400.StateMachines;
import team4400.StateMachines.IntakeState;

/*
 * 
_________             ___.           __  .__               
\_   ___ \  __________\_ |__   _____/  |_|__| ____   ______
/    \  \/_/ __ \_  __ \ __ \ /  _ \   __\  |/ ___\ /  ___/
\     \___\  ___/|  | \/ \_\ (  <_> )  | |  \  \___ \___ \ 
 \______  /\___  >__|  |___  /\____/|__| |__|\___  >____  >
        \/     \/          \/                    \/     \/ 
            _____    _____  _______  _______               
           /  |  |  /  |  | \   _  \ \   _  \              
          /   |  |_/   |  |_/  /_\  \/  /_\  \             
         /    ^   /    ^   /\  \_/   \  \_/   \            
         \____   |\____   |  \_____  /\_____  /            
              |__|     |__|        \/       \/             

        (Ascii art inspired by Botbusters 4635)
 *
 */

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;
  DoubleLogEntry batteryVoltageLog;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
    
    DataLog log = DataLogManager.getLog();
    
    if(Constants.needToLog){
      DataLogManager.start();
      DriverStation.startDataLog(log);
    }
    
    PathPlannerServer.startServer(5811);

    batteryVoltageLog = new DoubleLogEntry(log, "Battery Voltage");
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run(); 
    batteryVoltageLog.append(RobotController.getBatteryVoltage());
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
    //LimelightHelpers.setLEDMode_PipelineControl(VisionConstants.tapeLimelight);
  }

  @Override
  public void disabledPeriodic() {
  }

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }

    //m_robotContainer.getDrivetrain().setDriveCurrentLimit(75);

    m_robotContainer.getDrivetrain().setAllianceForVision(DriverStation.getAlliance());
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    m_robotContainer.getDrivetrain().setDriveCurrentLimit(75);
    m_robotContainer.getDrivetrain().setAllianceForVision(DriverStation.getAlliance());
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    if(StateMachines.getIntakeState() == IntakeState.FULL){
      m_robotContainer.setIntakeRumble(); 
    } else {
      m_robotContainer.getRumbleTimer().stop();
      m_robotContainer.getRumbleTimer().reset();
    }
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
