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

public class FeederLinkage extends SubsystemBase {
  /** Creates a new FeederLinkage. */
  CANSparkMax feederLinkage = new CANSparkMax(LinkageConstants.FEEDER_LINKAGE_ID, MotorType.kBrushless);
  CANSparkMax feederLinkageWheel = new CANSparkMax(LinkageConstants.FEEDER_WHEEL_ID, MotorType.kBrushless);

  RelativeEncoder feederEncoder = feederLinkage.getEncoder();

  SparkMaxPIDController feederController = feederLinkage.getPIDController();
  
  public FeederLinkage() {
    feederLinkage.restoreFactoryDefaults();
    feederLinkageWheel.restoreFactoryDefaults();

    feederLinkage.setIdleMode(IdleMode.kBrake);
    feederLinkageWheel.setIdleMode(IdleMode.kBrake);

    feederEncoder.setPosition(0);

    feederController.setP(LinkageConstants.FkP);
    feederController.setI(LinkageConstants.FkI);
    feederController.setD(LinkageConstants.FkD);
    feederController.setFF(LinkageConstants.IkFF);

    if(LinkageConstants.linkageTuningMode){
      SmartDashboard.putNumber("FLinkage P", LinkageConstants.FkP);
      SmartDashboard.putNumber("FLinkage I", LinkageConstants.FkI);
      SmartDashboard.putNumber("FLinkage D", LinkageConstants.FkD);
      SmartDashboard.putNumber("FLinkage FF", LinkageConstants.FkFF);
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    SmartDashboard.putNumber("Feeder Encoder", feederEncoder.getPosition());

    if(LinkageConstants.linkageTuningMode){
      double feederP = SmartDashboard.getNumber("FLinkage P", LinkageConstants.FkP);
      double feederI = SmartDashboard.getNumber("FLinkage I", LinkageConstants.FkI);
      double feederD = SmartDashboard.getNumber("FLinkage D", LinkageConstants.FkD);
      double feederFF = SmartDashboard.getNumber("FLinkage FF", LinkageConstants.FkFF);

      if((feederP != LinkageConstants.FkP)) {feederController.setP(feederP); LinkageConstants.FkP = feederP;}
      if((feederI != LinkageConstants.FkI)) {feederController.setI(feederI); LinkageConstants.FkP = feederI;}
      if((feederD != LinkageConstants.FkD)) {feederController.setD(feederD); LinkageConstants.FkP = feederD;}
      if((feederFF != LinkageConstants.FkFF)) {feederController.setFF(feederFF); LinkageConstants.FkP = feederFF;}
    }
  }

  public void setFeederPose(double pose){
    feederController.setReference(pose, ControlType.kPosition);
  }

  public void setFeederPower(double power){
    feederLinkageWheel.set(power);
  }
}
