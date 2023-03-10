// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.robot.Constants.ArmConstants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;

/** A robot arm subsystem that moves with a motion profile. */

/*
 * Why not use Rev's motion profiling if we have Neo 500 motors?
 * Well we have an Absolute Encoder that returns the current Arm angle but is not connected to the
 * Spark Max data port but to the RoboRIO DIO port 2, so we decided that it was better to use the
 * ProfiedPIDSubsystem class for our Arm so we can have a Motion Profiling PID with the absolute
 * Encoder as a feedback Device.
 */
public class ArmSubsystem extends ProfiledPIDSubsystem {
  private final CANSparkMax leftMotor = new CANSparkMax(ArmConstants.LEFT_ARM_ID, MotorType.kBrushless);
  private final CANSparkMax rightMotor = new CANSparkMax(ArmConstants.RIGHT_ARM_ID, MotorType.kBrushless);
  private final DutyCycleEncoder m_encoder =
      new DutyCycleEncoder(2);
  private final ArmFeedforward m_feedforward =
      new ArmFeedforward(
          ArmConstants.kS, ArmConstants.kG,
          ArmConstants.kV, ArmConstants.kA);

  boolean onTarget;

  /** Create a new ArmSubsystem. */
  public ArmSubsystem() {
    super(
        new ProfiledPIDController(
            ArmConstants.kP,
            0,
            ArmConstants.kD,
            new TrapezoidProfile.Constraints(
                ArmConstants.kMaxVelocityRadPerSecond,
                ArmConstants.kMaxAccelerationMetersPerSecondSquared)),
        90.3);
    
    //Makes the Arm absolute Encoder return every rotation as angles
    m_encoder.setDistancePerRotation(360.0);
    // Start arm at rest in neutral position
    setGoal(90.3);

    leftMotor.restoreFactoryDefaults();
    rightMotor.restoreFactoryDefaults();

    leftMotor.setInverted(false);
    rightMotor.follow(leftMotor, true);

    leftMotor.setSmartCurrentLimit(80);
    rightMotor.setSmartCurrentLimit(80);

    /*SmartDashboard.putNumber("Arm P", this.m_controller.getP());
    SmartDashboard.putNumber("Arm D", this.m_controller.getD());*/
  }


  @Override
  public void periodic() {
      super.periodic();
      SmartDashboard.putNumber("Angulo Encoder", getMeasurement());

      //SmartDashboard.putBoolean("Arm ready", isReady());
      //SmartDashboard.putNumber("Target Verdadero ", targetAngle);
      //SmartDashboard.putNumber("Goal Objetivo",this.getController().getGoal().position);
      //SmartDashboard.putNumber("Goal Velocidad", this.getController().getGoal().velocity);
      //SmartDashboard.putNumber("Error de posicion", this.getController().getPositionError());
      //SmartDashboard.putNumber("Velocidad ErrorBrazo", this.m_controller.getVelocityError());
      //SmartDashboard.putNumber("Consumo motor derecho:", rightMotor.getOutputCurrent());
      //SmartDashboard.putNumber("Consumo motor izq:", leftMotor.getOutputCurrent());

      /*double desiredAngle = SmartDashboard.getNumber("Desired Angle", targetAngle);
      if((desiredAngle != targetAngle)){desiredAngle = targetAngle;}

      double p = SmartDashboard.getNumber("Arm P", this.m_controller.getP());
      double d = SmartDashboard.getNumber("Arm D", this.m_controller.getD());

      if((p != ArmConstants.kP)){this.m_controller.setP(p); p = ArmConstants.kP;}
      if((d != ArmConstants.kD)){this.m_controller.setD(d); d = ArmConstants.kD;}*/
  }

  @Override
  public void useOutput(double output, TrapezoidProfile.State setpoint) {
    // Calculate the feedforward from the sepoint
    double feedforward = m_feedforward.calculate(setpoint.position, setpoint.velocity);
    // Add the feedforward to the PID output to get the motor output
    leftMotor.setVoltage(output + feedforward);
  }

  @Override
  public double getMeasurement() {
    //Minus 70.5 because that gives us a range betwueen 0-180 degrees, 0 being the left position
    //and 180 the right position while 90 degrees is the idle vertical position
    return m_encoder.getDistance() - 70.5;
  }

  public Command goToPosition(double position){
    Command ejecutable = Commands.runOnce(
                () -> {
                  this.setGoal(position);
                  this.enable();
                },
                this);
    return ejecutable;
  }

  public void goToPositionMethod(double goalPosition) {
    double pidVal = m_controller.calculate(getMeasurement(), goalPosition);
    double acceleration = (m_controller.getSetpoint().velocity );
    leftMotor.setVoltage(
        pidVal
        + m_controller.calculate(m_controller.getSetpoint().velocity, acceleration));
  }

  //For use in autonomous methods to shoot after the Arm is in position
  public boolean isReady(){
    if(getMeasurement() > getController().getGoal().position - ArmConstants.ARM_THRESHOLD 
    && getMeasurement() < getController().getGoal().position + ArmConstants.ARM_THRESHOLD){
      return true;
    } else {
      return false;
    }
  }
}

