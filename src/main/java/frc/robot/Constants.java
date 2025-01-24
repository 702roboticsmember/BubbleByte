package frc.robot;

import static frc.lib.util.COTSTalonFXSwerveConstants.SDS.MK4i.Falcon500;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

//import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
//import com.pathplanner.lib.util.PIDConstants;
// import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import frc.lib.util.COTSTalonFXSwerveConstants;
import frc.lib.util.COTSTalonFXSwerveConstants.SDS.MK4i.driveRatios;
import frc.lib.util.SwerveModuleConstants;

public final class Constants {
    public static final double CONTROLLER_DEADBAND = 0.1;

    

    /**
     * Corresponds to port zero on the Roborio DIO. 
     */
    public static final int LIMIT_SWITCH_INTAKE = 0;

    public static final class Swerve {
        /**
         * Whether gyroscope values should be inverted.
         */
        public static final boolean INVERT_GYRO = true;

        /**
         * Constants for the motor setup that we're using.
         */
        public static final COTSTalonFXSwerveConstants FALCON_500_CONSTANTS = Falcon500(driveRatios.L2);

        /**
         * Units: Meters
         */
        public static final double TRACK_WIDTH = Units.inchesToMeters(23.5);

        /**
         * Units: Meters
         */
        public static final double BASE_WIDTH = Units.inchesToMeters(23.5);

        /**
         * Units: Meters
         */
        public static final double DRIVEBASE_DIAMETER = Math.sqrt(TRACK_WIDTH * TRACK_WIDTH + BASE_WIDTH * BASE_WIDTH);

        /**
         * Units: Meters
         */
        public static final double DRIVEBASE_RADIUS = DRIVEBASE_DIAMETER / 2f;

        public static final double WHEEL_CIRCUMFERENCE = FALCON_500_CONSTANTS.wheelCircumference;

        public static final SwerveDriveKinematics KINEMATICS = new SwerveDriveKinematics(
                new Translation2d(BASE_WIDTH / 2.0, TRACK_WIDTH / 2.0),
                new Translation2d(BASE_WIDTH / 2.0, -TRACK_WIDTH / 2.0),
                new Translation2d(-BASE_WIDTH / 2.0, TRACK_WIDTH / 2.0),
                new Translation2d(-BASE_WIDTH / 2.0, -TRACK_WIDTH / 2.0));

        /* Module Gear Ratios */
        public static final double DRIVE_GEAR_RATIO = FALCON_500_CONSTANTS.driveGearRatio;
        public static final double ANGLE_GEAR_RATIO = FALCON_500_CONSTANTS.angleGearRatio;

        public static final InvertedValue ANGLE_MOTOR_INVERT = FALCON_500_CONSTANTS.angleMotorInvert;
        public static final InvertedValue DRIVE_MOTOR_INVERT = FALCON_500_CONSTANTS.driveMotorInvert;

        public static final SensorDirectionValue CANCODER_INVERT = FALCON_500_CONSTANTS.cancoderInvert;

        /**
         * Units: Volts
         */
        public static final int ANGLE_STATOR_CURRENT_LIMIT = 40;
        public static final int ANGLE_CURRENT_LIMIT = 25;
        public static final int ANGLE_CURRENT_THRESHOLD = 40;
        public static final double ANGLE_CURRENT_THRESHOLD_TIME = 0.1;
        public static final boolean ANGLE_ENABLE_CURRENT_LIMIT = true;
        public static final boolean ANGLE_ENABLE_STATOR_CURRENT_LIMIT = false;

        public static final int DRIVE_STATOR_CURRENT_LIMIT = 50;
        public static final int DRIVE_CURRENT_LIMIT = 35;//35
        public static final int DRIVE_CURRENT_THRESHOLD = 50;//60
        public static final double DRIVE_CURRENT_THRESHOLD_TIME = 0.1;
        public static final boolean DRIVE_ENABLE_CURRENT_LIMIT = true;
        public static final boolean DRIVE_ENABLE_STATOR_CURRENT_LIMIT = false;

        /*
         * These values are used by the drive falcon to ramp in open loop and closed
         * loop driving.
         * We found a small open loop ramp (0.25) helps with tread wear, tipping, etc
         */
        public static final double OPEN_LOOP_RAMP = 0.45;
        public static final double CLOSED_LOOP_RAMP = 0;

        public static final PIDConstants ANGLE_PID = new PIDConstants(FALCON_500_CONSTANTS.angleKP,
                FALCON_500_CONSTANTS.angleKI, FALCON_500_CONSTANTS.angleKD);
        public static final PIDConstants DRIVE_PID = new PIDConstants(0.12, 0.0, 0.0);

        /* Drive Motor Characterization Values From SYSID */
        public static final double DRIVE_KS = 0.32;
        public static final double DRIVE_KV = 1.51;
        public static final double DRIVE_KA = 0.27;

        /** Units: m/s */
        public static final double MAX_SPEED = 10;
        /** Units: radians/s */
        public static final double MAX_ANGULAR_VELOCITY = 10.0;

        /* Neutral Modes */
        public static final NeutralModeValue ANGLE_NEUTRAL_MODE = NeutralModeValue.Coast;
        public static final NeutralModeValue DRIVE_NEUTRAL_MODE = NeutralModeValue.Brake;

        /* Module Specific Constants */
        /* Front Left Module - Module 0 */
        public static final class Mod0 {
            public static final int driveMotorID = 1;
            public static final int angleMotorID = 2;
            public static final int canCoderID = 3;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(-177.178);
            public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
                    canCoderID, angleOffset);
        }

        /* Front Right Module - Module 1 */
        public static final class Mod1 {
            public static final int driveMotorID = 4;
            public static final int angleMotorID = 5;
            public static final int canCoderID = 6;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(-166.87);
            public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
                    canCoderID, angleOffset);
        }

        /* Back Left Module - Module 2 */
        public static final class Mod2 {
            public static final int driveMotorID = 7;
            public static final int angleMotorID = 8;
            public static final int canCoderID = 9;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(165.8407);
            public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
                    canCoderID, angleOffset);
        }

        /* Back Right Module - Module 3 */
        public static final class Mod3 {
            public static final int driveMotorID = 10;
            public static final int angleMotorID = 11;
            public static final int canCoderID = 12;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(127.798);
            public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
                    canCoderID, angleOffset);
        }
        public static final PPHolonomicDriveController PATHPLANNER_FOLLOWER_CONFIG = new PPHolonomicDriveController(
                new PIDConstants(5, 0, 0), 
                new PIDConstants(3, 0, 0)
                // MAX_SPEED,
                // DRIVEBASE_RADIUS,
                // new ReplanningConfig()
                );

    }

    public static final class ClimberConstants {
        public static final int MotorID = 0;

        public static final InvertedValue MotorInverted = InvertedValue.Clockwise_Positive;
        public static final NeutralModeValue LiftMotorMode = NeutralModeValue.Brake;
        
        public static final double MaxLiftSpeed = 4.0;

        public static final double kP = 1.3;
        public static final double kI = 0.0023;
        public static final double kD = 0.00147;

        public static final double Tolerance = 0.1;

        // public static boolean LiftLimitEnable = true;
        // public static final double LiftPIDTolerance = .5;
        // public static final double LeftLiftPosInValue = -41.7;
        // public static final double LeftLiftPosOutValue = 0;   
        // public static final double RightLiftPosInValue = -13;
        // public static final double RightLiftPosOutValue = 0;     

        public static final int STATOR_CURRENT_LIMIT = 35;
        public static final int CURRENT_LIMIT = 30;
        public static final boolean ENABLE_CURRENT_LIMIT = true;
        public static final boolean ENABLE_STATOR_CURRENT_LIMIT = true;

        public static final double ForwardLimit = 0;

        public static final boolean LimitEnable = true;

        public static final double ReverseLimit = 0;

        public static final double DefaultPose = 0;

        // public static final int CURRENT_THRESHOLD = 35;
        // public static final double CURRENT_THRESHOLD_TIME = 0.1;
    }


    public static final class AutoAimConstants {
        public static final double kP = 0.004537;
        public static final double kI = 0.0000;
        public static final double kD = 0.000;

        public static final double AutoAimPIDTolerance = 1.0;
        // public static final double DeflectorPosInValue = 0.0;
        // public static final double DeflectorPosOutValue = 0.0;

    }

    public static final class AutoFollowConstants {
        public static final double kP = 0.271;
        public static final double kI = 0;
        public static final double kD = 0.0;

       

        public static final double AutoFollowPIDTolerance = 1.0;
        // public static final double DeflectorPosInValue = 0.0;
        // public static final double DeflectorPosOutValue = 0.0;

    }

    public static final class AutoRotateConstants {
        public static final double kP = 0.002037;
        public static final double kI = 0.0000665;
        public static final double kD = 0.0003333;

        public static final double Tolerance = 6.0;
        // public static final double DeflectorPosInValue = 0.0;
        // public static final double DeflectorPosOutValue = 0.0;

    }

    public static final class AutoTranslateConstants {
        public static final double kP = 0.05471;
        public static final double kI = 0.000665;
        public static final double kD = 0.001333;

       

        public static final double Tolerance = 0.3;
        public static final double Setpoint = 1.2;
        // public static final double DeflectorPosInValue = 0.0;
        // public static final double DeflectorPosOutValue = 0.0;

    }

    public static final class AutoStrafeConstants {
        public static final double kP = 0.05471;
        public static final double kI = 0.000665;
        public static final double kD = 0.001333;

       

        public static final double Tolerance = 1;
        

    }

    public static final class AutoConstants { 

        public static final double kHeadingOffset = 90;
                                              // tuned to specific robot
        public static final double kMaxSpeedMetersPerSecond = 7.5;
        public static final double kMaxAccelerationMetersPerSecondSquared = 5.2;
        public static final double kMaxAngularSpeedRadiansPerSecond = 3 * Math.PI;
        public static final double kMaxAngularSpeedRadiansPerSecondSquared = 4* Math.PI;

        public static final double kPXController = 1;
        public static final double kPYController = 1;
        public static final double kPThetaController = 1;

        
        /**
         * Config for PathPlanner to follow auto paths
         */

        /* Constraint for the motion profilied robot angle controller */
        // public static final TrapezoidProfile.Constraints kThetaControllerConstraints
        // =
        // new TrapezoidProfile.Constraints(
        // kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
    }

    public static final class LEDConstants {
        public static final int LED_1_PwmID = 9;
        public static final int LED_1_Length = 28;
    }

    public static final class ElevatorConstants{
        public static final int Motor1ID = 0;
        public static final int Motor2ID = 1;
        public static final double ConversionConstant = 0.0;

        public static final InvertedValue MotorInverted = InvertedValue.Clockwise_Positive;
        public static final NeutralModeValue MotorMode = NeutralModeValue.Brake;

        public static final int STATOR_CURRENT_LIMIT = 35;
        public static final int CURRENT_LIMIT = 30;
        public static final boolean ENABLE_CURRENT_LIMIT = true;
        public static final boolean ENABLE_STATOR_CURRENT_LIMIT = true;
        public static final int CURRENT_THRESHOLD = 35;
        public static final double CURRENT_THRESHOLD_TIME = 0.1;

        public static final double Radius = 1;


        public static final double kP = 0;
        public static final double kI = 0;
        public static final double kD = 0;
        public static final double Tolerance = 0;
        public static final boolean LimitEnable = false;
        public static final double ForwardLimit = 0;
        public static final double ReverseLimit = 0;
        public static final double DefaultPose = 0;
    }

    public static final class AlgaeIntakeConstants{
        public static final int MotorID = 0;
        public static final double ConversionConstant = 0.0;

        public static final InvertedValue MotorInverted = InvertedValue.Clockwise_Positive;
        public static final NeutralModeValue MotorMode = NeutralModeValue.Brake;

        public static final int STATOR_CURRENT_LIMIT = 35;
        public static final int CURRENT_LIMIT = 30;
        public static final boolean ENABLE_CURRENT_LIMIT = true;
        public static final boolean ENABLE_STATOR_CURRENT_LIMIT = true;
    }

    public static final class CoralIntakeConstants {
        public static final int LeftMotorID = 0;
        public static final int RightMotorID = 0;
        public static final double ConversionConstant = 0.0;
        public static final boolean LeftMotorInverted = true;
        public static final boolean RightMotorInverted = false;
        public static final NeutralModeValue MotorMode = NeutralModeValue.Brake;
        

        public static final int STATOR_CURRENT_LIMIT = 35;
        public static final int CURRENT_LIMIT = 30;
        public static final boolean ENABLE_CURRENT_LIMIT = true;
        public static final boolean ENABLE_STATOR_CURRENT_LIMIT = true;
    }
    public static final class AlgaeArmConstants{
        public static final int MotorID = 0;
        public static final InvertedValue MotorInverted = InvertedValue.Clockwise_Positive;
        public static final NeutralModeValue MotorMode = NeutralModeValue.Brake;
        public static final double EncoderConversion = 0.0;
        public static final int STATOR_CURRENT_LIMIT = 35;
        public static final int CURRENT_LIMIT = 30;
        public static final boolean ENABLE_CURRENT_LIMIT = true;
        public static final boolean ENABLE_STATOR_CURRENT_LIMIT = true;
        public static final boolean LimitEnable = false;
        public static final double ForwardLimit = 0;
        public static final double ReverseLimit = 0;
        public static final double DefaultPose = 0;
    }


}
