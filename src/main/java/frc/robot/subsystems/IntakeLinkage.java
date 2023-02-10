// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LinkageConstants;

public class IntakeLinkage extends SubsystemBase {
  /** Creates a new Linkage. */
  CANSparkMax intakeLinkage = new CANSparkMax(LinkageConstants.INTAKE_LINKAGE_ID, MotorType.kBrushless);
  CANSparkMax intakeLinkageWheel = new CANSparkMax(LinkageConstants.INTAKE_WHEEL_ID, MotorType.kBrushless);
  
  RelativeEncoder intakeEncoder = intakeLinkage.getEncoder();

  SparkMaxPIDController intakeController = intakeLinkage.getPIDController();

  public IntakeLinkage() {
    intakeLinkage.restoreFactoryDefaults();
    intakeLinkageWheel.restoreFactoryDefaults();

    intakeLinkage.setIdleMode(IdleMode.kBrake);
    intakeLinkageWheel.setIdleMode(IdleMode.kBrake);
  
    intakeEncoder.setPosition(0);

    intakeController.setP(LinkageConstants.IkP);
    intakeController.setI(LinkageConstants.IkI);
    intakeController.setD(LinkageConstants.IkD);
    intakeController.setFF(LinkageConstants.IkFF);

    if(LinkageConstants.linkageTuningMode){
      SmartDashboard.putNumber("ILinkage P", LinkageConstants.IkP);
      SmartDashboard.putNumber("ILinkage I", LinkageConstants.IkI);
      SmartDashboard.putNumber("ILinkage D", LinkageConstants.IkD);
      SmartDashboard.putNumber("ILinkage FF", LinkageConstants.IkFF);
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    SmartDashboard.putNumber("Intake Encoder", intakeEncoder.getPosition());

    if(LinkageConstants.linkageTuningMode){
      double intakeP = SmartDashboard.getNumber("ILinkage P", LinkageConstants.IkP);
      double intakeI = SmartDashboard.getNumber("ILinkage I", LinkageConstants.IkI);
      double intakeD = SmartDashboard.getNumber("ILinkage D", LinkageConstants.IkD);
      double intakeFF = SmartDashboard.getNumber("ILinkage FF", LinkageConstants.IkFF);

      if((intakeP != LinkageConstants.IkP)) {intakeController.setP(intakeP); LinkageConstants.IkP = intakeP;}
      if((intakeI != LinkageConstants.IkI)) {intakeController.setI(intakeI); LinkageConstants.IkP = intakeI;}
      if((intakeD != LinkageConstants.IkD)) {intakeController.setD(intakeD); LinkageConstants.IkP = intakeD;}
      if((intakeFF != LinkageConstants.IkFF)) {intakeController.setFF(intakeFF); LinkageConstants.IkP = intakeFF;}
    }
  }

  public void setIntakePose(double pose){
    intakeController.setReference(pose, ControlType.kPosition);
  }

  public void setIntakePower(double power){
    intakeLinkageWheel.set(power);
  }
}
