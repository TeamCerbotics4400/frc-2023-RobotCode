// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.WristConstants;
import frc.robot.commands.AutoCommands.LimelightAutoAlign;
import frc.robot.commands.AutoCommands.AutoRoutinesCommands.PieceWBalance;
import frc.robot.commands.AutoCommands.AutoRoutinesCommands.StraightAuto;
import frc.robot.commands.AutoCommands.AutoRoutinesCommands.Blue.BlueTwoPiecesWBalance;
import frc.robot.commands.AutoCommands.AutoRoutinesCommands.Blue.BlueTwoWorking;
import frc.robot.commands.AutoCommands.AutoRoutinesCommands.Red.RedTwoBalance;
import frc.robot.commands.AutoCommands.AutoRoutinesCommands.Red.RedTwoWorking;
import frc.robot.commands.TeleOpCommands.LimelightToggle;
import frc.robot.commands.TeleOpCommands.TeleOpControl;
import frc.robot.commands.AlignToNode;
import frc.robot.commands.StateIntakeCommand;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
import frc.robot.subsystems.NodeSelector;
/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
import frc.robot.subsystems.VisionSubsystem;

public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  Joystick chassisDriver = new Joystick(0);
  Joystick subsystemsDriver = new Joystick(1);
  private DriveTrain m_drive = new DriveTrain();
  private FalconShooter m_shooter = new FalconShooter();
  private NodeSelector m_nodeSelector = new NodeSelector(subsystemsDriver);
  private WristSubsystem m_wrist = new WristSubsystem();
  private ArmSubsystem m_arm = new ArmSubsystem();
  private VisionSubsystem m_vision = new VisionSubsystem();
  
  private final SendableChooser<String> m_autoChooser = new SendableChooser<>(); 
  private final String m_DefaultAuto = "PIECE AND BALANCE";//"NO AUTO";
  private String m_autoSelected;
  private final String[] m_autoNames = {"NO AUTO", "PID TUNER", "STRAIGHT AUTO", 
      "PIECE AND BALANCE", "BLUE TWO WORKING", "BLUE TWO AND BALANCE", "THREE PIECES", "THREE BALANCE", "RED TWO WORKING",
      "RED TWO AND BALANCE"};

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    m_autoChooser.setDefaultOption("PieceBalance Default", m_DefaultAuto);
    //m_autoChooser.addOption("PID Tuner", m_autoNames[1]);
    m_autoChooser.addOption("Straight Auto", m_autoNames[2]);
    //m_autoChooser.addOption("Piece and balance", m_autoNames[3]);
    m_autoChooser.addOption("BLUE Two Working", m_autoNames[4]);
    m_autoChooser.addOption("BLUE Two and Balance", m_autoNames[5]);
    //m_autoChooser.addOption("Three pieces", m_autoNames[6]);
    //m_autoChooser.addOption("Three and Balance", m_autoNames[7]);
    m_autoChooser.addOption("RED Two Working", m_autoNames[8]);
    m_autoChooser.addOption("RED Two and Balance", m_autoNames[9]);

    SmartDashboard.putData("Auto Choices", m_autoChooser);

    configureBindings();
  }

  public DriveTrain getDrivetrain(){
    return m_drive;
  }

  public VisionSubsystem getVision(){
    return m_vision;
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
   m_drive.setDefaultCommand(new TeleOpControl(m_drive, 
   chassisDriver));

   m_shooter.setDefaultCommand(new StateIntakeCommand(m_shooter, m_arm, chassisDriver, 
                                                      subsystemsDriver, m_nodeSelector));

    new JoystickButton(chassisDriver, 1).whileTrue(new AlignToNode(m_drive, m_nodeSelector, chassisDriver));
                                            //DriveToNode(m_drive, m_nodeSelector, chassisDriver));

    new JoystickButton(chassisDriver, 2).whileTrue(new LimelightAutoAlign(m_drive, chassisDriver));

    new JoystickButton(chassisDriver, 5).onTrue(m_arm.goToPosition(ArmConstants.BACK_FLOOR_POSITION))
   .whileTrue(m_wrist.goToPosition(WristConstants.RIGHT_POSITION))
   //.whileTrue(new StateIntakeCommand(m_shooter))
   .whileFalse(m_arm.goToPosition(ArmConstants.IDLE_POSITION))
   .whileFalse(m_wrist.goToPosition(WristConstants.IDLE_POSITION));
   //.whileFalse(new StateIntakeCommand(m_shooter));

   new JoystickButton(chassisDriver, 6).onTrue(m_arm.goToPosition(ArmConstants.FRONT_FLOOR_POSITION))
   .whileTrue(m_wrist.goToPosition(WristConstants.LEFT_POSITION))
   //.whileTrue(new StateIntakeCommand(m_shooter))
   .whileFalse(m_arm.goToPosition(ArmConstants.IDLE_POSITION))
   .whileFalse(m_wrist.goToPosition(WristConstants.IDLE_POSITION));
   //.whileFalse(new StateIntakeCommand(m_shooter));

   //Controller 2
   //new POVButton(subsystemsDriver, 90).onTrue(new NodeSelectionRight(m_nodeSelector));

   new POVButton(subsystemsDriver, 90).onTrue(new InstantCommand(() -> m_nodeSelector.updateSelectionRight()));

   new POVButton(subsystemsDriver, 270).onTrue(new InstantCommand(() -> m_nodeSelector.updateSelectionLeft()));

   new POVButton(subsystemsDriver, 0).onTrue(new InstantCommand(() -> m_nodeSelector.updateSelectionUp()));

   new POVButton(subsystemsDriver, 180).onTrue(new InstantCommand(() -> m_nodeSelector.updateSelectionDown()));

   /*new JoystickButton(subsystemsDriver, 3).onTrue(m_arm.goToPosition(ArmConstants.SUBSTATION_POSITION))
   .whileTrue(m_wrist.goToPosition(WristConstants.LEFT_POSITION))
   .whileTrue(new IntakeCubes(m_shooter))
   .whileFalse(m_arm.goToPosition(ArmConstants.IDLE_POSITION))
   .whileFalse(m_wrist.goToPosition(WristConstants.IDLE_POSITION));*/

   new JoystickButton(subsystemsDriver, 5).onTrue(m_arm.goToPosition(ArmConstants.FRONT_FLOOR_POSITION))
   .whileTrue(m_wrist.goToPosition(WristConstants.LEFT_POSITION))
   .whileFalse(m_arm.goToPosition(ArmConstants.IDLE_POSITION))
   .whileFalse(m_wrist.goToPosition(WristConstants.IDLE_POSITION));

   new JoystickButton(subsystemsDriver, 6).onTrue(m_arm.goToPosition(ArmConstants.SCORING_POSITION))
   .whileTrue(m_wrist.goToPosition(WristConstants.LEFT_POSITION))
   .whileFalse(m_arm.goToPosition(ArmConstants.IDLE_POSITION))
   .whileFalse(m_wrist.goToPosition(WristConstants.IDLE_POSITION));

   //new JoystickButton(subsystemsDriver, 4).whileTrue(new CombinedShooter(m_arm, m_wrist, m_shooter, m_nodeSelector));

   new JoystickButton(subsystemsDriver, 10).whileTrue(new LimelightToggle());
  }    

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    Command autonomousCommand = null;
    m_autoSelected = m_autoChooser.getSelected();

    System.out.println("Auto Selected: " + m_autoSelected);
    switch(m_autoSelected){
      /*case "PID TUNER":
       autonomousCommand = new PIDTunnerCommand(m_drive, m_arm, m_wrist);
      break;*/

      case "STRAIGHT AUTO":
        autonomousCommand = new StraightAuto(m_drive, m_arm, m_wrist, m_shooter, m_nodeSelector);
      break;

      case "PIECE AND BALANCE":
        autonomousCommand = new PieceWBalance(m_drive, m_arm, m_wrist, m_shooter, m_nodeSelector);
      break;

      case "BLUE TWO WORKING":
        autonomousCommand = new BlueTwoWorking(m_drive, m_arm, m_wrist, m_shooter, m_nodeSelector);
      break;

      case "BLUE TWO AND BALANCE":
        autonomousCommand = new BlueTwoPiecesWBalance(m_drive, m_arm, m_wrist, m_shooter, m_nodeSelector);
      break;

      /*case "THREE PIECES":
       autonomousCommand = new ThreePieces(m_drive, m_arm, m_wrist, m_shooter, m_nodeSelector);
      break;

      case "THREE BALANCE":
       autonomousCommand = new ThreePiecesBalance(m_drive, m_arm, m_wrist, m_shooter, m_nodeSelector);
      break;*/

      case "RED TWO WORKING":
       autonomousCommand = new RedTwoWorking(m_drive, m_arm, m_wrist, m_shooter, m_nodeSelector);
      break;

      case "RED TWO AND BALANCE":
       autonomousCommand = new RedTwoBalance(m_drive, m_arm, m_wrist, m_shooter, m_nodeSelector);
      break;
    }

    return autonomousCommand;
  }
}
