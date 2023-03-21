// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.WristConstants;
import frc.robot.commands.CombinedShooter;
import frc.robot.commands.AutoCommands.DriveToNode;
import frc.robot.commands.AutoCommands.LimelightAutoAlign;
import frc.robot.commands.AutoCommands.AutoRoutinesCommands.PIDTunnerCommand;
import frc.robot.commands.AutoCommands.AutoRoutinesCommands.PieceWBalance;
import frc.robot.commands.AutoCommands.AutoRoutinesCommands.TwoWorking;
import frc.robot.commands.AutoCommands.AutoRoutinesCommands.StraightAuto;
import frc.robot.commands.AutoCommands.AutoRoutinesCommands.ThreePieces;
import frc.robot.commands.AutoCommands.AutoRoutinesCommands.TwoPiecesWBalance;
import frc.robot.commands.TeleOpCommands.NodeSelectionDown;
import frc.robot.commands.TeleOpCommands.NodeSelectionLeft;
import frc.robot.commands.TeleOpCommands.NodeSelectionRight;
import frc.robot.commands.TeleOpCommands.NodeSelectionUp;
import frc.robot.commands.TeleOpCommands.TeleOpControl;
import frc.robot.commands.IntakeCubes;
import frc.robot.commands.LimelightToggle;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.FalconShooter;
import frc.robot.subsystems.NodeSelector;
/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
import frc.robot.subsystems.WristSubsystem;
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  Joystick chassisDriver = new Joystick(0);
  Joystick subsystemsDriver = new Joystick(1);
  private DriveTrain m_drive = new DriveTrain();
  private FalconShooter m_shooter = new FalconShooter();
  private NodeSelector m_nodeSelector = new NodeSelector(subsystemsDriver);
  private WristSubsystem m_wrist = new WristSubsystem();
  private ArmSubsystem m_arm = new ArmSubsystem();
  
  private final SendableChooser<String> m_autoChooser = new SendableChooser<>(); 
  private final String m_DefaultAuto = "PIECE AND BALANCE";//"NO AUTO";
  private String m_autoSelected;
  private final String[] m_autoNames = {"NO AUTO", "PID TUNER", "STRAIGHT AUTO", 
      "PIECE AND BALANCE", "TWO WORKING", "TWO PIECES AND BALANCE", "THREE PIECES"};

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    m_autoChooser.setDefaultOption("PieceBalance Default", m_DefaultAuto);
    m_autoChooser.addOption("PID Tuner", m_autoNames[1]);
    m_autoChooser.addOption("Straight Auto", m_autoNames[2]);
    m_autoChooser.addOption("Piece and balance", m_autoNames[3]);
    m_autoChooser.addOption("Two Working", m_autoNames[4]);
    m_autoChooser.addOption("Two and Balance", m_autoNames[5]);
    m_autoChooser.addOption("Three pieces", m_autoNames[5]);

    SmartDashboard.putData("Auto Choices", m_autoChooser);

    configureBindings();
  }

  public DriveTrain getDrivetrain(){
    return m_drive;
  }

  public ArmSubsystem getArm(){
    return m_arm;
  }

  public WristSubsystem getWrist(){
    return m_wrist;
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

   m_drive.setDefaultCommand(new TeleOpControl(m_drive, 
   chassisDriver));

   if(Constants.twoControllerMode){

    new JoystickButton(chassisDriver, 1).whileTrue(new 
                                            DriveToNode(m_drive, m_nodeSelector, chassisDriver));

    new JoystickButton(chassisDriver, 2).whileTrue(new LimelightAutoAlign(m_drive, chassisDriver));

    new POVButton(subsystemsDriver, 90).onTrue(new NodeSelectionRight(m_nodeSelector));

    new POVButton(subsystemsDriver, 270).onTrue(new NodeSelectionLeft(m_nodeSelector));

    new POVButton(subsystemsDriver, 0).onTrue(new NodeSelectionUp(m_nodeSelector));

    new POVButton(subsystemsDriver, 180).onTrue(new NodeSelectionDown(m_nodeSelector));

    new JoystickButton(chassisDriver, 5).onTrue(m_arm.goToPosition(ArmConstants.BACK_FLOOR_POSITION))
   .whileTrue(m_wrist.goToPosition(WristConstants.RIGHT_POSITION))
   .whileTrue(new IntakeCubes(m_shooter))
   .whileFalse(m_arm.goToPosition(ArmConstants.IDLE_POSITION))
   .whileFalse(m_wrist.goToPosition(WristConstants.IDLE_POSITION));

   new JoystickButton(chassisDriver, 6).onTrue(m_arm.goToPosition(ArmConstants.FRONT_FLOOR_POSITION))
   .whileTrue(m_wrist.goToPosition(WristConstants.LEFT_POSITION))
   .whileTrue(new IntakeCubes(m_shooter))
   .whileFalse(m_arm.goToPosition(ArmConstants.IDLE_POSITION))
   .whileFalse(m_wrist.goToPosition(WristConstants.IDLE_POSITION));

   new JoystickButton(subsystemsDriver, 3).onTrue(m_arm.goToPosition(ArmConstants.SUBSTATION_POSITION))
   .whileTrue(m_wrist.goToPosition(WristConstants.LEFT_POSITION))
   .whileTrue(new IntakeCubes(m_shooter))
   .whileFalse(m_arm.goToPosition(ArmConstants.IDLE_POSITION))
   .whileFalse(m_wrist.goToPosition(WristConstants.IDLE_POSITION));

   new JoystickButton(subsystemsDriver, 5).onTrue(m_arm.goToPosition(ArmConstants.FRONT_FLOOR_POSITION))
   .whileTrue(m_wrist.goToPosition(WristConstants.LEFT_POSITION))
   .whileFalse(m_arm.goToPosition(ArmConstants.IDLE_POSITION))
   .whileFalse(m_wrist.goToPosition(WristConstants.IDLE_POSITION));

   new JoystickButton(subsystemsDriver, 6).onTrue(m_arm.goToPosition(ArmConstants.AVE_MARIA_SHOOT_POSITION))
   .whileTrue(m_wrist.goToPosition(WristConstants.LEFT_POSITION))
   .whileFalse(m_arm.goToPosition(ArmConstants.IDLE_POSITION))
   .whileFalse(m_wrist.goToPosition(WristConstants.IDLE_POSITION));

   new JoystickButton(subsystemsDriver, 4).whileTrue(new CombinedShooter(m_arm, m_wrist, m_shooter, m_nodeSelector));

   new JoystickButton(subsystemsDriver, 10).whileTrue(new LimelightToggle());

   } else {

    new JoystickButton(chassisDriver, 1).whileTrue(new 
                                            DriveToNode(m_drive, m_nodeSelector, chassisDriver));

    new JoystickButton(chassisDriver, 9).whileTrue(new LimelightAutoAlign(m_drive, chassisDriver));

    new POVButton(chassisDriver, 90).onTrue(new NodeSelectionRight(m_nodeSelector));

    new POVButton(chassisDriver, 270).onTrue(new NodeSelectionLeft(m_nodeSelector));

    new POVButton(chassisDriver, 0).onTrue(new NodeSelectionUp(m_nodeSelector));

    new POVButton(chassisDriver, 180).onTrue(new NodeSelectionDown(m_nodeSelector));

    new JoystickButton(chassisDriver, 5).onTrue(m_arm.goToPosition(ArmConstants.BACK_FLOOR_POSITION))
   .whileTrue(m_wrist.goToPosition(WristConstants.RIGHT_POSITION))
   .whileTrue(new IntakeCubes(m_shooter))
   .whileFalse(m_arm.goToPosition(ArmConstants.IDLE_POSITION))
   .whileFalse(m_wrist.goToPosition(WristConstants.IDLE_POSITION));

   new JoystickButton(chassisDriver, 3).onTrue(m_arm.goToPosition(ArmConstants.SUBSTATION_POSITION))
   .whileTrue(m_wrist.goToPosition(WristConstants.LEFT_POSITION))
   .whileTrue(new IntakeCubes(m_shooter))
   .whileFalse(m_arm.goToPosition(ArmConstants.IDLE_POSITION))
   .whileFalse(m_wrist.goToPosition(WristConstants.IDLE_POSITION));

   new JoystickButton(chassisDriver, 6).onTrue(m_arm.goToPosition(ArmConstants.SCORING_POSITION))
   .whileTrue(m_wrist.goToPosition(WristConstants.LEFT_POSITION))
   .whileFalse(m_arm.goToPosition(ArmConstants.IDLE_POSITION))
   .whileFalse(m_wrist.goToPosition(WristConstants.IDLE_POSITION));

   new JoystickButton(chassisDriver, 4).whileTrue(new CombinedShooter(m_arm, m_wrist, m_shooter, m_nodeSelector));
   }
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
      case "PID TUNER":
       autonomousCommand = new PIDTunnerCommand(m_drive, m_arm, m_wrist);
      break;

      case "STRAIGHT AUTO":
        autonomousCommand = new StraightAuto(m_drive, m_arm, m_wrist, m_shooter, m_nodeSelector);
      break;

      case "PIECE AND BALANCE":
        autonomousCommand = new PieceWBalance(m_drive, m_arm, m_wrist, m_shooter, m_nodeSelector);
      break;

      case "TWO WORKING":
        autonomousCommand = new TwoWorking(m_drive, m_arm, m_wrist, m_shooter, m_nodeSelector);
      break;

      case "TWO PIECES AND BALANCE":
        autonomousCommand = new TwoPiecesWBalance(m_drive, m_arm, m_wrist, m_shooter, m_nodeSelector);
      break;

      case "THREE PIECES":
       autonomousCommand = new ThreePieces(m_drive, m_arm, m_wrist, m_shooter, m_nodeSelector);
      break;
    }

    return autonomousCommand;
  }
}
