// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static class SimulationConstants{
    public static final double kTrackwidthMeters = 0.69;
    public static final DifferentialDriveKinematics kDriveKinematics =
        new DifferentialDriveKinematics(kTrackwidthMeters);

    public static final int kEncoderCPR = 1024;
    public static final double kWheelDiameterMeters = 0.15;
    public static final double kEncoderDistancePerPulse =
        // Assumes the encoders are directly mounted on the wheel shafts
        (kWheelDiameterMeters * Math.PI) / (double) kEncoderCPR;

    public static final boolean kGyroReversed = true;

    public static final double ksVolts = 0.22;
    public static final double kvVoltSecondsPerMeter = 1.98;
    public static final double kaVoltSecondsSquaredPerMeter = 0.2;

    public static final double kvVoltSecondsPerRadian = 1.5;
    public static final double kaVoltSecondsSquaredPerRadian = 0.3;

    public static final LinearSystem<N2, N2, N2> kDrivetrainPlant =
        LinearSystemId.identifyDrivetrainSystem(
            kvVoltSecondsPerMeter,
            kaVoltSecondsSquaredPerMeter,
            kvVoltSecondsPerRadian,
            kaVoltSecondsSquaredPerRadian);

    public static final DCMotor kDriveGearbox = DCMotor.getCIM(2);
    public static final double kDriveGearing = 8;

    public static final double kPDriveVel = 8.5;

    public static final double kP = 0.1, 
    kI = 0, 
    kD = 0;

    public static final double kS = 0.22, 
    kV = 1.98, 
    kA = 0.2;

    public static final double kRamseteB = 2;
    public static final double kRamseteZeta = 0.7;

    public static final double kMaxSpeedMetersPerSecond = 4.0; 
    public static final double kMaxAccelerationMetersPerSecondSquared = 3.0; 

    public static final int k100msPerSecond = 10;
  }

  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static final class DriveConstants{
    public static final int LeftMaster_ID = 9; //3
    public static final int LeftSlave_ID = 8; //4

    public static final int RightMaster_ID = 7; //1
    public static final int RightSlave_ID = 6;  //2

    public static final double kP = 0//3.9356
    ,  kI = 0
    , kD = 03 ;

    public static final double kS = 0,//0.2131,
                               kV = 0,//2.8451, 
                               kA = 0;//0.64907;

    public static final double WHEEL_DIAMETER = 0;
    public static final double GEAR_RATIO_ONE = 0;//7 : 1
    public static final double GEAR_RATIO_TWO = 0;//9.22 : 1
    public static final double TRACK_WIDTH = 0; 
    public static final double ENCODER_CPR = 0;
    public static final DifferentialDriveKinematics kDriveKinematics =
        new DifferentialDriveKinematics(TRACK_WIDTH);

    public static final double TRACK_WIDTH_INCHES = Units.metersToInches(TRACK_WIDTH);

    public static final double TRACK_SCRUB_FACTOR = 0;//Para un giro mas cerrado del cheezy drive

    public static final int Solenoid_Shifter_ID = 0;
}
public static final class ShooterConstants{
  /** ---PROTOTYPE--- */
  public static final byte FLY_WHEEL_LOWER_ID = 1; 
  public static final byte FLY_WHEEL_UPPER_ID = 2;

  public static double kP= 0, //0.00001
                       kI = 0,
                       kD = 0, //0.0001
                       kIz = 0,
                       kFF = 0,
                       kMaxOutput = 0,
                       kMinOutput = 0, 
                       maxRPM = 0, 
                       maxVel = 0, //---------
                       minVel = 0, //---------
                       maxAcc = 0, 
                       allowedErr = 0;

}
}

