// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.WristConstants;
import frc.robot.commands.TeleOpCommands.TeleOpControl;
import frc.robot.commands.StateIntake;
import frc.robot.commands.StateShooterCommand;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.FalconShooter;
import frc.robot.subsystems.WristSubsystem;
import team4400.StateMachines;
import team4400.StateMachines.IntakeState;
import frc.robot.subsystems.NodeSelector;
/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */

public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  Joystick chassisDriver = new Joystick(0);
  Joystick subsystemsDriver = new Joystick(1);
  private DriveTrain m_drive = new DriveTrain();
  private FalconShooter m_shooter = new FalconShooter();
  private NodeSelector m_nodeSelector = new NodeSelector(subsystemsDriver);
  private WristSubsystem m_wrist = new WristSubsystem();
  private ArmSubsystem m_arm = new ArmSubsystem();

  Timer rumbleTimer = new Timer();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings

    configureBindings();
  }

  public DriveTrain getDrivetrain(){
    return m_drive;
  }

  public ArmSubsystem getArm(){
    return m_arm;
  }

  public Timer getRumbleTimer(){
    return rumbleTimer;
  }
  
  public void setIntakeRumble(){
    rumbleTimer.start();
    if(rumbleTimer.get() < 1){
      chassisDriver.setRumble(RumbleType.kBothRumble, 1);
    } else {
      chassisDriver.setRumble(RumbleType.kBothRumble, 0);
    }
  }
  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {

  //Controller 1
  //Left and right sticks
   m_drive.setDefaultCommand(new TeleOpControl(m_drive, 
   chassisDriver));
      
   //Left bumper
   new JoystickButton(chassisDriver, 5)
    .onTrue(m_arm.goToPosition(ArmConstants.BACK_FLOOR_POSITION))
   .whileTrue(m_wrist.goToPosition(WristConstants.RIGHT_POSITION))
   .whileTrue(new StateIntake(m_shooter, m_arm, IntakeState.INTAKING))
   .whileFalse(m_arm.goToPosition(ArmConstants.IDLE_POSITION))
   .whileFalse(m_wrist.goToPosition(WristConstants.IDLE_POSITION));

   //Right bumper
   new JoystickButton(chassisDriver, 6)
   .onTrue(m_arm.goToPosition(ArmConstants.FRONT_FLOOR_POSITION))
   .whileTrue(m_wrist.goToPosition(WristConstants.LEFT_POSITION))
   .whileTrue(new StateIntake(m_shooter, m_arm, IntakeState.INTAKING))
   .whileFalse(m_arm.goToPosition(ArmConstants.IDLE_POSITION))
   .whileFalse(m_wrist.goToPosition(WristConstants.IDLE_POSITION));

   //Controller 2
   //Pov up
   new POVButton(subsystemsDriver, 0).onTrue(new InstantCommand(() -> m_nodeSelector.updateSelectionUp()));

   //Pov down
   new POVButton(subsystemsDriver, 180).onTrue(new InstantCommand(() -> m_nodeSelector.updateSelectionDown()));

   //Left bumper
    new JoystickButton(subsystemsDriver, 5)
   .onTrue(m_arm.goToPosition(ArmConstants.FRONT_FLOOR_POSITION))
   .whileTrue(m_wrist.goToPosition(WristConstants.LEFT_POSITION))
   .whileFalse(m_arm.goToPosition(ArmConstants.IDLE_POSITION))
   .whileFalse(m_wrist.goToPosition(WristConstants.IDLE_POSITION));

   //Right bumper
   new JoystickButton(subsystemsDriver, 6)
   .onTrue(m_arm.goToPosition(ArmConstants.SCORING_POSITION))
   .whileTrue(m_wrist.goToPosition(WristConstants.LEFT_POSITION))
   .whileFalse(m_arm.goToPosition(ArmConstants.IDLE_POSITION))
   .whileFalse(m_wrist.goToPosition(WristConstants.IDLE_POSITION));

   new JoystickButton(subsystemsDriver, 4)
   .whileTrue(new StateShooterCommand(m_shooter, m_arm, m_wrist, IntakeState.SHOOTING, 
                                                                            m_nodeSelector));

   new JoystickButton(subsystemsDriver, 2).toggleOnTrue(new InstantCommand(() -> StateMachines.setIntakeIdle()));
  }    

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return null;
  }
}
