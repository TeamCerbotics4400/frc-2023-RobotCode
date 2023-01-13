// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.DefaultShooter;
import frc.robot.subsystems.Shooter;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.SimTeleOp;
import frc.robot.subsystems.DrivetrainSim;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...

  Joystick joystick = new Joystick(0);
  JoystickButton FerBestoPrograON = new JoystickButton(joystick, 1);
  JoystickButton FerBestoPrograOFF = new JoystickButton(joystick, 2);
  JoystickButton buttonX = new JoystickButton(joystick, 3);
  JoystickButton buttonY = new JoystickButton(joystick, 4);
  JoystickButton leftBumper = new JoystickButton(joystick, 5);
  JoystickButton rightBumper = new JoystickButton(joystick, 6);
  private final Shooter shooter1 = new Shooter();

  DrivetrainSim driveSim = new DrivetrainSim();
  Joystick joy0 = new Joystick(0);


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();

    driveSim.setDefaultCommand(new SimTeleOp(driveSim, 
    () -> joy0.getRawAxis(1), 
    () -> joy0.getRawAxis(4)));
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
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    FerBestoPrograON.toggleOnTrue(new DefaultShooter(shooter1));
    

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
    return null;
  }

  public DrivetrainSim getSimDrive(){
    return driveSim;
  }
}
