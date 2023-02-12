// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Map;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import team4400.Alert;
import team4400.Alert.AlertType;

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

  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static final class LinkageConstants {

    public static final boolean linkageTuningMode = true;

    public static final byte INTAKE_LINKAGE_ID = 5;
    public static final byte INTAKE_WHEEL_ID = 8;
    public static final byte FEEDER_LINKAGE_ID = 7;
    public static final byte FEEDER_WHEEL_ID = 6;

    public static final double intakeExtended = 2.40;

    public static final double feederIntaking = 0.28571423;
    public static final double feederShooting = -0.642857;

    public static double IkP= 0.2,
                         IkI = 0,
                         IkIz = 0,
                         IkD = 0.01, 
                         IkFF = 0;
    
    public static double FkP= 0,
                         FkI = 0,
                         FkIz = 0,
                         FkD = 0, 
                         FkFF = 0.05,
                         FkMaxOutput = 1,
                         FkMinOutput = -1,
                         FMaxAcc = 1500;



                         
  }

  public static final class ShooterConstants{
    /** ---PROTOTYPE--- */
    public static final byte LEFT_FLYWHEEL_ID = 9; 
    public static final byte RIGHT_FLYWHEEL_ID = 10;

    public static double kP= 0.096,//0.1,//0.000031, //con bandas = 0.098
                         kI = 0.001,//0.001,//0,
                         kD = 0.001,//5,//0.0001,
                         kIz = 300,
                         kFF = 0.40,//1023.0/20660.0,//0.00018, //con bandas = 0.42
                         kMaxOutput = 1,
                         kMinOutput = -1, 
                         maxRPM = 0, 
                         maxVel = 0, //---------
                         minVel = 0, //---------
                         maxAcc = 0, 
                         allowedErr = 0;

    public static double targetVelocity = 0;

    public static final double shooterTreshold = 300;

}
  public static final class DriveConstants{
    public static final int LeftLeader_ID = 1; //3
    public static final int LeftFollower_ID = 2; //4

    public static final int RightLeader_ID = 3; //1
    public static final int RightFollower_ID = 4;  //2

    public static final double kP = 0.013687//3.9356
    ,  kI = 0
    , kD = 0.0010903;

    public static final double kS = 0.15862,//0.15862
                               kV = 2.8101,//2.8101, 
                               kA = 0.39782;//0.64907;

    public static final double WHEEL_DIAMETER = 0.1524;
    public static final double GEAR_RATIO = 0;//7 : 1
    public static final double TRACK_WIDTH = 0.69; 
    public static final double ENCODER_CPR = 0;
    public static final DifferentialDriveKinematics kDriveKinematics =
        new DifferentialDriveKinematics(TRACK_WIDTH);

    public static final double TRACK_WIDTH_INCHES = Units.metersToInches(TRACK_WIDTH);

    public static final double TRACK_SCRUB_FACTOR = 0;//Para un giro mas cerrado del cheezy drive

    public static final int Solenoid_Shifter_ID = 0;

    
}

public static final class AutoConstants{
  public static final double kMaxSpeedMetersPerSecond = 2.0;
  public static final double kMaxAccelerationMetersPerSecondSquared = 2.0;
}

public static final class FieldConstants{
  public static final double length = Units.feetToMeters(54);
  public static final double width = Units.feetToMeters(27);
}

public static final class VisionConstants {
  /** ---PROTOTYPE--- */
  public static double HEIGHT_OF_OUTER_PORT = 2.64;//Altura del target
  public static double LIMELIGHT_FLOOR_CLEREANCE= 0.79;//Altura de la limelight
  public static double LIMELIGHT_VERTICAL_ANGLE = 36; //Angulo de la limelight

  public static final Transform3d robotToCam =
          new Transform3d(
                  new Translation3d(0.32, 0.03, 0.53),
                  new Rotation3d(
                          0, 0,
                          0)); // Cam mounted facing forward, half a meter forward of center, half a meter up
  // from center.
  public static final String cameraName = "Limelight";
}
  /* vDelCodigo = "1.5"; */
  /* Cosas Por Hacer:
  *  boolean PID_Intake = false ;
   * boolean Sim = false;
   * 
   */
}
  


