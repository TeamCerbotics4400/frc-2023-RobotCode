// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.AutoBalance;
import frc.robot.commands.ResetImuCommand;
import frc.robot.commands.TeleOpControl;
import frc.robot.commands.AutoCommands.StraightLineAutoCommand;
import frc.robot.commands.DefaultShooter;
import frc.robot.commands.FeedShooter;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Shooter;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  Joystick joy0 = new Joystick(0);
  private DriveTrain m_drive = new DriveTrain();
  JoystickButton rightBumper = new JoystickButton(joy0, 6);
// private final Shooter shooter1 = new Shooter();
  //private final NeoIntake neointake = new NeoIntake();
  private Shooter m_shooter = new Shooter();
  //JoystickButton FerBestoProgra = new JoystickButton(joy0, 1);
  //JoystickButton Citrus1678BestoFRCTeam = new JoystickButton(joy0, 2);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */

  public RobotContainer() {
    // Configure the trigger bindings

    /*driveSim.setDefaultCommand(new SimTeleOp(driveSim, 
    () -> joy0.getRawAxis(2), //4 para joystick, 0 para teclado
    () -> joy0.getRawAxis(1)));*/
    


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
   new JoystickButton(joy0, 1).toggleOnTrue(new AutoBalance(m_drive));

   //Reset Imu
   new JoystickButton(joy0, 2).onTrue(new ResetImuCommand(m_drive));

   //Servos Shooter
   new JoystickButton(joy0, 3).whileTrue(new FeedShooter(m_shooter));

   new JoystickButton(joy0, 1).whileTrue(new DefaultShooter(m_shooter));
  
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`

    /*Citrus1678BestoFRCTeam.toggleOnTrue(new Intake(neointake));
    FerBestoProgra.toggleOnTrue(new DefaultShooter(shooter1));*/

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
    return new StraightLineAutoCommand(m_drive);
  }

  /*public DrivetrainSim getSimDrive(){
    return driveSim;
  }*/
}
