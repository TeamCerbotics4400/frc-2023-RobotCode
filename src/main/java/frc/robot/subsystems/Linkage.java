// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LinkageConstants;

public class Linkage extends SubsystemBase {
  /** Creates a new Linkage. */
  CANSparkMax intakeLinkage = new CANSparkMax(LinkageConstants.INTAKE_LINKAGE_ID, MotorType.kBrushless);
  CANSparkMax intakeLinkageWheel = new CANSparkMax(0, MotorType.kBrushless);
  CANSparkMax feederLinkage = new CANSparkMax(0, MotorType.kBrushless);
  CANSparkMax feederLinkageWheel = new CANSparkMax(LinkageConstants.FEEDER_LINKAGE_ID, MotorType.kBrushless);

  RelativeEncoder intakeEncoder = intakeLinkage.getEncoder();
  RelativeEncoder feederEncoder = feederLinkage.getEncoder();

  SparkMaxPIDController intakeController = intakeLinkage.getPIDController();
  SparkMaxPIDController feederController = feederLinkage.getPIDController();

  public Linkage() {
    intakeLinkage.restoreFactoryDefaults();
    intakeLinkageWheel.restoreFactoryDefaults();
    feederLinkage.restoreFactoryDefaults();
    feederLinkageWheel.restoreFactoryDefaults();

    intakeLinkage.setIdleMode(IdleMode.kBrake);
    intakeLinkageWheel.setIdleMode(IdleMode.kBrake);
    feederLinkage.setIdleMode(IdleMode.kBrake);
    feederLinkageWheel.setIdleMode(IdleMode.kBrake);

    intakeController.setP(LinkageConstants.IkP);
    intakeController.setI(LinkageConstants.IkI);
    intakeController.setD(LinkageConstants.IkD);
    intakeController.setFF(LinkageConstants.IkFF);

    feederController.setP(LinkageConstants.FkP);
    feederController.setI(LinkageConstants.FkI);
    feederController.setD(LinkageConstants.FkD);
    feederController.setFF(LinkageConstants.IkFF);

    if(LinkageConstants.linkageTuningMode){
      SmartDashboard.putNumber("ILinkage P", LinkageConstants.IkP);
      SmartDashboard.putNumber("ILinkage I", LinkageConstants.IkI);
      SmartDashboard.putNumber("ILinkage D", LinkageConstants.IkD);
      SmartDashboard.putNumber("ILinkage FF", LinkageConstants.IkFF);

      SmartDashboard.putNumber("FLinkage P", LinkageConstants.FkP);
      SmartDashboard.putNumber("FLinkage P", LinkageConstants.FkI);
      SmartDashboard.putNumber("FLinkage P", LinkageConstants.FkD);
      SmartDashboard.putNumber("FLinkage P", LinkageConstants.FkFF);
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if(LinkageConstants.linkageTuningMode){
      double intakeP = SmartDashboard.getNumber("ILinkage P", LinkageConstants.IkP);
      double intakeI = SmartDashboard.getNumber("ILinkage I", LinkageConstants.IkI);
      double intakeD = SmartDashboard.getNumber("ILinkage D", LinkageConstants.IkD);
      double intakeFF = SmartDashboard.getNumber("ILinkage FF", LinkageConstants.IkFF);

      double feederP = SmartDashboard.getNumber("FLinkage P", LinkageConstants.FkP);
      double feederI = SmartDashboard.getNumber("FLinkage I", LinkageConstants.FkI);
      double feederD = SmartDashboard.getNumber("FLinkage D", LinkageConstants.FkD);
      double feederFF = SmartDashboard.getNumber("FLinkage FF", LinkageConstants.FkFF);

      if((intakeP != LinkageConstants.IkP)) {intakeController.setP(intakeP); LinkageConstants.IkP = intakeP;}
      if((intakeI != LinkageConstants.IkI)) {intakeController.setI(intakeI); LinkageConstants.IkP = intakeI;}
      if((intakeD != LinkageConstants.IkD)) {intakeController.setD(intakeD); LinkageConstants.IkP = intakeD;}
      if((intakeFF != LinkageConstants.IkFF)) {intakeController.setFF(intakeFF); LinkageConstants.IkP = intakeFF;}

      if((feederP != LinkageConstants.FkP)) {feederController.setP(feederP); LinkageConstants.FkP = feederP;}
      if((feederI != LinkageConstants.FkI)) {feederController.setI(feederI); LinkageConstants.FkP = feederI;}
      if((feederD != LinkageConstants.FkD)) {feederController.setD(feederD); LinkageConstants.FkP = feederD;}
      if((feederFF != LinkageConstants.FkFF)) {feederController.setFF(feederFF); LinkageConstants.FkP = feederFF;}
    }
  }
}
