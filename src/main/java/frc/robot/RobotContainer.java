// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.AutoBalance;
import frc.robot.commands.ConeShooter;
import frc.robot.commands.ResetImuCommand;
import frc.robot.commands.ShooterTrigger;
//import frc.robot.commands.ShooterTrigger;
import frc.robot.commands.TeleOpControl;
import frc.robot.commands.AutoCommands.AlignWithTag;
import frc.robot.commands.AutoCommands.DriveToPoseTest;
import frc.robot.commands.AutoCommands.StraightLineAutoCommand;
import frc.robot.commands.CubeShooter;
import frc.robot.commands.IntakePieces;
import frc.robot.commands.DefaultIndexer;
import edu.wpi.first.wpilibj.DriverStation;
//import frc.robot.commands.IntakePieces;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.FalconShooter;
import frc.robot.subsystems.FeederLinkage;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeLinkage;

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
  private IntakeLinkage m_intake = new IntakeLinkage();
  private FeederLinkage m_feeder = new FeederLinkage();
  private FalconShooter m_shooter = new FalconShooter();
  private IndexerSubsystem m_indexer = new IndexerSubsystem();

  private AutoParser autoParser = new AutoParser(m_drive);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    
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

   //Autobalance
   new JoystickButton(joy1, 7).toggleOnTrue(new AutoBalance(m_drive));

   //Reset Imu
   //new JoystickButton(joy0, 1).onTrue(new DriveToPoseTest(m_drive));

   new JoystickButton(joy1, 1).whileTrue(new CubeShooter(m_shooter));

   new JoystickButton(joy1, 2).whileTrue(new ConeShooter(m_shooter));

   new JoystickButton(joy0, 5).whileTrue(new ShooterTrigger(m_feeder, m_shooter));
   
   new JoystickButton(joy0, 6).whileTrue(new IntakePieces(m_intake));

   new JoystickButton(joy1, 3).whileTrue(new DefaultIndexer(m_indexer));
   //Shooter
   //new JoystickButton(joy0, 2).whileTrue(new ShooterTrigger(m_feeder));

   //Intake y feeder

  
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
  }    
  

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return autoParser.getAutoCommand();
  }

  public void parseAuto() {
    String autoText = SmartDashboard.getString("AutoCode", "");
    String parserOutput = "";
    try {
      parserOutput = autoParser.parse(autoText, DriverStation.getAlliance());
    } catch (Exception e) {
      parserOutput = e.getMessage();
      e.printStackTrace();
    } finally {
      SmartDashboard.putString("Compiler Message", parserOutput);
    }
  }

  /*public DrivetrainSim getSimDrive(){
    return driveSim;
  }*/
}
