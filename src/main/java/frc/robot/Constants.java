// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    public static final double PI = 3.14159;
    public static final boolean CompBot = Robot.isCompBot();
    public static final boolean PracticeBot = Robot.isPracticeBot();
    public static final boolean OtherBot = Robot.isOtherBot();

    public static final int PCMID = 3;  // CAN ID for PCM

    // IDs for physical input devices
    // Make sure order matches that of DriverStation
    public static final class Inputs {
        public static final int driverControllerID = 0; // ID for Xbox/Logitech controller
        public static final int operatorControllerID = 1;
    }

    //Swerve Drive Drivetrain Related Constants
    public static final class DriveConstants {

        public static final double gearRatio = 6.75; //SDS mk4i L2
        public static final double wheelCircumference = 0.309;  //SDS mk4i L2
        public static final double encoderTicks = 2048;  //CANCoder

        //These are feedforward controllers for within swerve modules
        public static final double ksVolts = 0.73394;  //all need to be characterized by the SysId tool
        public static final double kvVoltSecondsPerMeter = 2.4068;
        public static final double kaVoltSecondsSquaredPerMeter = 0.28749;

        public static final double ksTurning = 0.75; //Borrowed these values
        public static final double kvTurning = 0.75; //.384 to 1.27 (1.27 aligns 12V to 3pi max angular velocity)
        public static final double kaTurning = 0;

        //These are PID controllers for within swerve modules
        public static final double kPModuleDriveController = 3;  //Borrowed these values
        public static final double kIModuleDriveController = 0; 
        public static final double kDModuleDriveController = 0;

        public static final double kPModuleTurnController = 10;  //Borrowed these values
        public static final double kIModuleTurnController = 0;
        public static final double kDModuleTurnController = 0.01;

        //Borrowed these values...
        public static final double kModuleMaxAngularVelocity = 3 * Math.PI; // Math.PI  (units: rad/s)
        public static final double kModuleMaxAngularAcceleration = 6 * Math.PI; //Math.PI * 2  (units: rad/s/s)

        //Some limits governing overall robot movement
        public static final double kMaxWheelSpeedMetersPerSecond = 4; //3
        public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI * 1; 

        public static final class moduleA {
            public static final double otherTurnOffset = 79; //May need finer adjustment on these.
            public static final double practiceTurnOffset = -57; //May need finer adjustment on these.
            public static final double compTurnOffset = 141.5;

            public static double turnOffset() {
                return (CompBot) ? compTurnOffset : (PracticeBot ? practiceTurnOffset : otherTurnOffset);
            }
        }

        public static final class moduleB {
            public static final double otherTurnOffset = -96; //May need finer adjustment on these.
            public static final double practiceTurnOffset = 53; //May need finer adjustment on these.
            public static final double compTurnOffset = 69.6;

            public static double turnOffset() {
                return (CompBot) ? compTurnOffset : (PracticeBot ? practiceTurnOffset : otherTurnOffset);
            }
        }

        public static final class moduleC {
            public static final double otherTurnOffset = 104; //May need finer adjustment on these.
            public static final double practiceTurnOffset = -14; //May need finer adjustment on these.
            public static final double compTurnOffset = -148.1;

            public static double turnOffset() {
                return (CompBot) ? compTurnOffset : (PracticeBot ? practiceTurnOffset : otherTurnOffset);
            }
        }
        
        public static final class moduleD {
            public static final double otherTurnOffset = -52; //May need finer adjustment on these.
            public static final double practiceTurnOffset = 18; //May need finer adjustment on these.
            public static final double compTurnOffset = 41.6;

            public static double turnOffset() {
                return (CompBot) ? compTurnOffset : (PracticeBot ? practiceTurnOffset : otherTurnOffset);
            }
        }

        //CAN IDs
        public static final int gyroID = 1;

        public static final int angleMotorA = 20;
        public static final int angleMotorB = 30;
        public static final int angleMotorC = 40;
        public static final int angleMotorD = 50;

        public static final int driveMotorA = 21;
        public static final int driveMotorB = 31;
        public static final int driveMotorC = 41;
        public static final int driveMotorD = 51;

        public static final int turnEncoderA = 22;
        public static final int turnEncoderB = 32;
        public static final int turnEncoderC = 42;
        public static final int turnEncoderD = 52;
    }

    public static final class EndEffectorConstants {
        public static final int neoID = 12;
        public static final int snowblowerID = 13;
    }

    public static final class LimelightConstants {
        public static final double driverPipe = 2.0;
        public static final double targetPipe = 1.0;
        public static final double offPipe = 0.0;
        
        public static final double kP = 0.015;
        public static final double kI = 0;
        public static final double kD = 0;

        //Reduced these for safety until we are comfortable withe autoaim command
        public static final double minOutput = -0.75;
        public static final double maxOutput = 0.75;
        // 1 degree angle of error which is considered tolerable for the PID
        public static final double threshold = 0.1;
    }

    public static final class ArmConstants {
        public static final int leftArmID = 10;
        public static final int rightArmID = 11;

        public static final int camEncoderDIO = 7;

        public static final double armGearRatio = 166.5; //this is what's listed in the arm engineering notebook, no idea if that's right

        //Kinematics values based on arm geometry.  All of this is in inches and degrees for easy comparison to CAD
        public static final double R1 = 6.125; //Crank length
        public static final double R2 = 42.5;  //Connecting rod length
        public static final double R3 = 7.25; //Connecting rod pivot to arm pivot
        public static final double PHI = 9.1;  //Angle that connecting rod leads arm
        public static final double X3 = 18.18; // horizontal distance between crank pivot and arm pivot
        public static final double Y3 =  39.304; //vertical distance between crank pivot and arm pivot
        public static final double maxCrankAngle = 241.11; //degrees for horizontal forward.
        public static final double minCrankAngle = 421.11;

        //random numbers for each of these levels, need to find accurate measurements later
        public static final double highLevel = 10;
        public static final double midLevel = -10;
        public static final double lowLevel = -88;
        public static final double stowLevel = -110;//-106
        public static final double pickup = -77;

        public static final double kS = 0.6;//0.6 most recently 0.3 original
        public static final double kV = 0.06;

    }
}
