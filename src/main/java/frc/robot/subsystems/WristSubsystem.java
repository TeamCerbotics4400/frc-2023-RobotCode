// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants.WristConstants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;

/** A robot arm subsystem that moves with a motion profile. */
public class WristSubsystem extends ProfiledPIDSubsystem {
  private final CANSparkMax wristMotor = new CANSparkMax(WristConstants.WRIST_ID, MotorType.kBrushless);

  private RelativeEncoder wristEncoder = wristMotor.getEncoder();

  private DigitalInput hallEffectSensor = new DigitalInput(1);

  private final ArmFeedforward m_feedforward =
      new ArmFeedforward(
          WristConstants.kS, WristConstants.kG,
          WristConstants.kV, WristConstants.kA);

  /** Create a new ArmSubsystem. */
  //Relacion: 210.0 : 1.00
  public WristSubsystem() {
    super(
        new ProfiledPIDController(
            WristConstants.kP,
            0,
            WristConstants.kD,
            new TrapezoidProfile.Constraints(
                WristConstants.kMaxVelocityRadPerSecond,
                WristConstants.kMaxAccelerationMetersPerSecondSquared)),
        0);
    // Start arm at rest in neutral position
    setGoal(0);

    wristMotor.restoreFactoryDefaults();

    wristMotor.setInverted(false); //Mirandolo desde un lado gira a la derecha

    wristMotor.setSmartCurrentLimit(5);

    /*SmartDashboard.putNumber("Arm P", this.m_controller.getP());
    SmartDashboard.putNumber("Arm D", this.m_controller.getD());*/

    resetEncoder();
  }


  @Override
  public void periodic() {
      // TODO Auto-generated method stub
      super.periodic();
      //SmartDashboard.putBoolean("Wrist ready", isReady());
      //SmartDashboard.putNumber("Angulo Encoder", getMeasurement());
      //SmartDashboard.putNumber("Angulo Objetivo",this.getController().getSetpoint().position);
      //SmartDashboard.putNumber("Objetivo Velocidad", this.getController().getSetpoint().velocity);
      //SmartDashboard.putNumber("Wrist Angle", getDegrees().getDegrees());

      //SmartDashboard.putBoolean("Hall effect activado", hallEffectSensor.get());

      //resetEncoderSensor();

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
    wristMotor.setVoltage(output + feedforward);
  }

  public synchronized Rotation2d getDegrees(){
    return Rotation2d.fromRadians(wristEncoder.getPosition() / 210.0 * 2 * Math.PI);
  }

  public void resetEncoder(){
    wristEncoder.setPosition(0);
  }

  public void resetEncoderSensor(){
    if(!hallEffectSensor.get() && wristEncoder.getVelocity() < 0.1){
      wristEncoder.setPosition(0);
    }
  }

  @Override
  public double getMeasurement() {
    return getDegrees().getDegrees();
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

  public void goToPositionMethod(double position){
    this.setGoal(position);
    this.enable();
  }

  public boolean isReady(){
    if(getMeasurement() > getController().getGoal().position - WristConstants.WRIST_THRESHOLD
    && getMeasurement() < getController().getGoal().position + WristConstants.WRIST_THRESHOLD){
      return true;
    } else {
      return false;
    }
  }
}