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
public class ArmSubsystem extends ProfiledPIDSubsystem {
  private final CANSparkMax leftMotor = new CANSparkMax(ArmConstants.LEFT_ARM_ID, MotorType.kBrushless);
  private final CANSparkMax rightMotor = new CANSparkMax(ArmConstants.RIGHT_ARM_ID, MotorType.kBrushless);
  private final DutyCycleEncoder m_encoder =
      new DutyCycleEncoder(2);
  private final ArmFeedforward m_feedforward =
      new ArmFeedforward(
          ArmConstants.kS, ArmConstants.kG,
          ArmConstants.kV, ArmConstants.kA);

  double targetAngle = 0.0;

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
    
    m_encoder.setDistancePerRotation(360.0);
    // Start arm at rest in neutral position
    setGoal(90.3);

    leftMotor.restoreFactoryDefaults();
    rightMotor.restoreFactoryDefaults();

    leftMotor.setInverted(false);
    rightMotor.follow(leftMotor, true);

    leftMotor.setSmartCurrentLimit(40);
    rightMotor.setSmartCurrentLimit(40);

    //SmartDashboard.putNumber("Desired Angle", targetAngle);

    /*SmartDashboard.putNumber("Arm P", this.m_controller.getP());
    SmartDashboard.putNumber("Arm D", this.m_controller.getD());*/
  }


  @Override
  public void periodic() {
      // TODO Auto-generated method stub
      super.periodic();
      SmartDashboard.putNumber("Angulo Encoder", getMeasurement());
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

  public void gotoPositionMethod(double position){
    this.setGoal(position);
    this.enable();
  }

  public boolean isReady(){
    if(getMeasurement() > getController().getGoal().position - ArmConstants.ARM_THRESHOLD 
    && getMeasurement() < getController().getGoal().position + ArmConstants.ARM_THRESHOLD){
      return true;
    } else {
      return false;
    }
  }

  public double getTargetAngle(){
    return targetAngle;
  }
}

