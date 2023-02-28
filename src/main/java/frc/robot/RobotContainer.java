// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.AutoAlign;
import frc.robot.commands.AutoBalance;
import frc.robot.commands.ConeShooter;
import frc.robot.commands.TeleOpControl;
import frc.robot.commands.AutoCommands.DriveToNode;
import frc.robot.commands.CubeShooter;
import frc.robot.commands.NodeSelectionLeft;
import frc.robot.commands.NodeSelectionRight;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.FalconShooter;
import frc.robot.subsystems.NodeSelector;
/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  Joystick joy0 = new Joystick(0);
  Joystick joy1 = new Joystick(1);
  private DriveTrain m_drive = new DriveTrain();
  private FalconShooter m_shooter = new FalconShooter();
  private NodeSelector m_nodeSelector = new NodeSelector(joy0);
  
  private final SendableChooser<String> m_autoChooser = new SendableChooser<>(); 
  private final String m_DefaultAuto = "NO AUTO";
  private String m_autoSelected;
  private final String[] m_autoNames = {"NO AUTO", "STRAIGH LINE"};

  private AutoParser autoParser = new AutoParser(m_drive);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings

    m_autoChooser.setDefaultOption("Default Auto", m_DefaultAuto);
    
    configureBindings();
  }

  public DriveTrain getDrivetrain(){
    return m_drive;
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
   //Control teleOp
   m_drive.setDefaultCommand(new TeleOpControl(m_drive, 
   joy0));

   new JoystickButton(joy0, 1).whileTrue(new 
                                            DriveToNode(m_drive, m_nodeSelector, joy0));
   //Autobalance
   new JoystickButton(joy0, 6).whileTrue(new AutoBalance(m_drive));

   //Reset Imu
   new JoystickButton(joy0, 2).whileTrue(new AutoAlign(m_drive));

   new JoystickButton(joy1, 1).whileTrue(new CubeShooter(m_shooter));

   new JoystickButton(joy1, 2).whileTrue(new ConeShooter(m_shooter));

   new POVButton(joy0, 90).onTrue(new NodeSelectionRight(m_nodeSelector));

   new POVButton(joy0, 270).onTrue(new NodeSelectionLeft(m_nodeSelector));

   //new JoystickButton(joy0, 5).whileTrue(new ShooterTrigger(m_feeder, m_shooter));
   
   //new JoystickButton(joy0, 6).whileTrue(new IntakePieces(m_intake));

   //new JoystickButton(joy1, 3).whileTrue(new DefaultIndexer(m_indexer));
   //Shooter
   //new JoystickButton(joy0, 2).whileTrue(new ShooterTrigger(m_feeder));
  }    
  

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    Command autoCommand = null;
    m_autoSelected = m_autoChooser.getSelected();

    return autoCommand;
  }
}
