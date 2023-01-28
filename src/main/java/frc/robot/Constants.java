// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Map;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.Util.Alert;
import frc.robot.Util.Alert.AlertType;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  private static final RobotType robot = RobotType.ROBOT_2023P;
  public static final double loopPeriodSecs = 0.02;

  private static final Alert invalidRobotAlert =
      new Alert("Invalid robot selected, using competition robot as default.", AlertType.ERROR);

  public static RobotType getRobot() {
    if (RobotBase.isReal()) {
      if (robot == RobotType.ROBOT_SIMBOT) { // Invalid robot selected
        invalidRobotAlert.set(true);
        return RobotType.ROBOT_2023C;
      } else {
        return robot;
      }
    } else {
      return robot;
    }
  }

  public static Mode getMode() {
    switch (getRobot()) {
      case ROBOT_2023C:
      case ROBOT_2023P:
        return RobotBase.isReal() ? Mode.REAL : Mode.REPLAY;

      case ROBOT_SIMBOT:
        return Mode.SIM;

      default:
        return Mode.REAL;
    }
  }

  public static final Map<RobotType, String> logFolders =
      Map.of(RobotType.ROBOT_2023P, "/media/sda2/");

  public static enum RobotType {
    ROBOT_2023C,
    ROBOT_2023P,
    ROBOT_SIMBOT
  }

  public static enum Mode {
    REAL,
    REPLAY,
    SIM
  }


  public static final boolean tuningMode = false;
  public static class SimulationConstants{
    public static final double kTrackwidthMeters = 0.546;
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
    public static final double kDriveGearing = 10.71;

    public static final double kWheelRadiusInches = 3;

    public static final double kPDriveVel = 8.5;

    public static final double kP = 0, 
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
    public static final int LeftMaster_ID = 2; //3
    public static final int LeftSlave_ID = 3; //4

    public static final int RightMaster_ID = 4; //1
    public static final int RightSlave_ID = 5;  //2

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

public static final class AutoConstants{
  public static final double kMaxSpeedMetersPerSecond = 0;
  public static final double kMaxAccelerationMetersPerSecondSquared = 0;
}

public static final class FieldConstants{
  public static final double length = Units.feetToMeters(54);
  public static final double width = Units.feetToMeters(27);
}

public static final class ShooterConstants{
  /** ---PROTOTYPE--- */
  public static final byte FLY_WHEEL_LOWER_ID = 8; 
  public static final byte FLY_WHEEL_UPPER_ID = 9;

  public static double kP= 0.000031,
                       kI = 0,
                       kD = 0.0001,
                       kIz = 0,
                       kFF = 0.0001678,
                       kMaxOutput = 0,
                       kMinOutput = 0, 
                       maxRPM = 0, 
                       maxVel = 0, //---------
                       minVel = 0, //---------
                       maxAcc = 0, 
                       allowedErr = 0;

                       public static double targetVelocity = 0;

}

public static final class VisionConstants {
  /** ---PROTOTYPE--- */
  public static double HEIGHT_OF_OUTER_PORT = 2.64;//Altura del target
  public static double LIMELIGHT_FLOOR_CLEREANCE= 0.79;//Altura de la limelight
  public static double LIMELIGHT_VERTICAL_ANGLE = 36; //Angulo de la limelight

  public static final Transform3d robotToCam =
          new Transform3d(
                  new Translation3d(0.33, 0.03, 0.53),
                  new Rotation3d(
                          0, 0,
                          0)); // Cam mounted facing forward, half a meter forward of center, half a meter up
  // from center.
  public static final String cameraName = "Limelight";
}

public static final class IntakeConstants{
  public static final byte IntakePoseMotor_ID = 6;
  public static final byte RapidWheeel_ID = 7;
  public static double kP= 0,
                       kI = 0,
                       kD = 0,
                       kIz = 0,
                       kFF = 0,
                       kMaxOutput = 0,
                       kMinOutput = 0, 
                       maxRPM = 0, 
                       maxVel = 100, //---------
                       minVel = 0, //---------
                       maxAcc = 10, 
                       allowedErr = 5;



}
  /* vDelCodigo = "1.5"; */
  /* Cosas Por Hacer:
  *  boolean PID_Intake = false ;
   * boolean Sim = false;
   * 
   */
}
  


