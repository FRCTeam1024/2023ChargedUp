package frc.robot;

import java.util.Map;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

import edu.wpi.first.wpilibj2.command.Command;

import edu.wpi.first.wpilibj2.command.ProxyCommand;

public class RobotDashboardContainer extends RobotAutoContainer {
    
    public RobotDashboardContainer() {
        super();

        // Configure the dashboard
        configureDashboard();
    }

    /**
   * Use this method to configure the dashboard
   * 
   */
  private void configureDashboard() {

    //Create ShuffleBoard Tabs
    ShuffleboardTab diagnosticsTab = Shuffleboard.getTab("1024Diagnostics");
    ShuffleboardTab driverTab = Shuffleboard.getTab("1024Driver");
    ShuffleboardTab swerveTab = Shuffleboard.getTab("1024Swerve");
    /**
     * Diagnostics for programmers
     */
    //Add command status to dashboard
    diagnosticsTab.add("DrivetrainCommand",drivetrain)
       .withSize(2,1)
       .withPosition(8,0);

    /**
     * Driver's operator interface
     */

    //Display the name and version number of the code.
    /**driverTab.add("Running Code Version:", BuildConfig.APP_NAME + " " + BuildConfig.APP_VERSION)
        .withSize(3,1)
        .withPosition(0,0);*/

    //Add commands to auto chooser, set default to null to avoid surprise operation
    m_AutoChooser.setDefaultOption("None", null);
    //m_AutoChooser.addOption("Test Path", new ProxyCommand(() -> TestAuto()));
    //m_AutoChooser.addOption("Test2", new ProxyCommand(() -> TestAuto2()));
    //m_AutoChooser.addOption("Testing Swerve Auto", new ProxyCommand(() -> returnAutoCommand()));
    //m_AutoChooser.addOption("Testing Auto Balance", new ProxyCommand(() -> TestAutoBalance()));
    //m_AutoChooser.addOption("C-Charge", new ProxyCommand(() -> C_Charge()));
    //m_AutoChooser.addOption("C-Cross-Charge", new ProxyCommand(() -> C_Cross_Charge()));
    //m_AutoChooser.addOption("C-1-O", new ProxyCommand(() -> C_1_O()));
    //m_AutoChooser.addOption("C-2-O", new ProxyCommand(() -> C_2_O()));
    //m_AutoChooser.addOption("C-OuterRoute-Charge", new ProxyCommand(() -> C_OuterRoute_Charge()));
    //m_AutoChooser.addOption("C-4-I", new ProxyCommand(() -> C_4_I()));
    //m_AutoChooser.addOption("C-3-I", new ProxyCommand(() -> C_3_I()));
    //m_AutoChooser.addOption("I-3-I", new ProxyCommand(() -> I_3_I()));
    //m_AutoChooser.addOption("I-4-I", new ProxyCommand(() -> I_4_I()));
    //m_AutoChooser.addOption("O-1-O", new ProxyCommand(() -> O_1_O()));
    //m_AutoChooser.addOption("O-2-O", new ProxyCommand(() -> O_2_O()));
    //m_AutoChooser.addOption("C-InnerRoute-Charge", new ProxyCommand(() -> C_InnerRoute_Charge()));
    //m_AutoChooser.addOption("I-Charge", new ProxyCommand(() -> I_Charge()));
    //m_AutoChooser.addOption("O-Charge", new ProxyCommand(() -> O_Charge()));
    m_AutoChooser.addOption("C-Cone-Cross-Charge", new ProxyCommand(() -> C_Cone_Cross_Charge()));
    m_AutoChooser.addOption("C-Cone-Charge", new ProxyCommand(() -> C_Cone_Charge()));
    m_AutoChooser.addOption("C-Cube-Cross-Charge", new ProxyCommand(() -> C_Cube_Cross_Charge()));
    m_AutoChooser.addOption("OuterConeGrab", new ProxyCommand(() -> OuterConeGrab()));
    m_AutoChooser.addOption("2OuterCones", new ProxyCommand(() -> OuterCones()));
    m_AutoChooser.addOption("2InnerCones", new ProxyCommand(() -> InnerCones()));
    m_AutoChooser.addOption("OuterConesCharge - Testing only", new ProxyCommand(() -> OuterConesCharge()));
    m_AutoChooser.addOption("2.5OuterPieces", new ProxyCommand(() -> OuterConesPlusOne()));
    m_AutoChooser.addOption("Low Link", new ProxyCommand(() -> LowLink()));
    

    //Puts the auto chooser on the dashboard
    driverTab.add("Auto Mode",m_AutoChooser)
       .withSize(3,1)
       .withPosition(0,0);

    //Adds the limelight camera feed
    driverTab.addCamera("limelight", "OV5647", "http://photonvision.local:1182/stream.mjpg")
        .withSize(3,3)
        .withPosition(0,1); 

    driverTab.add("Arm Camera", arm.getFeed())
        .withSize(4,4)
        .withPosition(3,0)
        .withProperties(Map.of("ROTATION", "HALF"));

    driverTab.addNumber("Arm Angle", () -> arm.getArmAngle())
        .withSize(1,1)
        .withPosition(7,0);

    driverTab.addNumber("Raw Wrist Angle", () -> endEffector.getRawWristAngle())
        .withSize(1,1)
        .withPosition(7,1);

    driverTab.addNumber("Wrist Angle", () -> endEffector.getWristAngle())
        .withSize(1,1)
        .withPosition(7,2);

    /**driverTab.addNumber("AprilTag ID", () -> drivetrain.getCamera().getBestTarget().getFiducialId())
        .withSize(1,1)
        .withPosition(0,2);

    driverTab.addBoolean("AprilTag targets", () -> drivetrain.getCamera().hasTargets())
        .withSize(1,1)
        .withPosition(0,3);*/

    diagnosticsTab.addNumber("Robot Yaw", () -> drivetrain.getYawDegrees())
        .withSize(1,1)
        .withPosition(0,0);

    diagnosticsTab.addNumber("Pigeon Heading", () -> drivetrain.getPigeonHeading())
        .withSize(1,1)
        .withPosition(0,1);

    diagnosticsTab.addNumber("Pigeon Yaw", () -> drivetrain.getPigeonYaw())
        .withSize(1,1)
        .withPosition(0,2);
        
    //Swerve module angles A-D
    diagnosticsTab.addNumber("SwerveModule A Angle", () -> drivetrain.getAngleRad(1))
        .withSize(1,1)
        .withPosition(5,0);

    diagnosticsTab.addNumber("SwerveModule B Angle", () -> drivetrain.getAngleRad(2))
        .withSize(1,1)
        .withPosition(6,0);

    diagnosticsTab.addNumber("SwerveModule C Angle", () -> drivetrain.getAngleRad(3))
        .withSize(1,1)
        .withPosition(5,1);

    diagnosticsTab.addNumber("SwerveModule D Angle", () -> drivetrain.getAngleRad(4))
        .withSize(1,1)
        .withPosition(6,1);

    diagnosticsTab.addNumber("Neo Temperature", () -> endEffector.getTemperature())
        .withSize(1,1)
        .withPosition(3,2);

    //Swerve module velocity errors A-D
    swerveTab.addNumber("SwerveModule A Vel Error", () -> drivetrain.getModVelError(1))
    .withSize(1,1)
    .withPosition(7,0);

    swerveTab.addNumber("SwerveModule B Vel Error", () -> drivetrain.getModVelError(2))
    .withSize(1,1)
    .withPosition(8,0);

    swerveTab.addNumber("SwerveModule C Vel Error", () -> drivetrain.getModVelError(3))
    .withSize(1,1)
    .withPosition(7,1);

    swerveTab.addNumber("SwerveModule D Vel Error", () -> drivetrain.getModVelError(4))
    .withSize(1,1)
    .withPosition(8,1);

    //Swerve module angle errors A-D
    swerveTab.addNumber("SwerveModule A Angle Error", () -> drivetrain.getModAngError(1))
    .withSize(1,1)
    .withPosition(7,2);

    swerveTab.addNumber("SwerveModule B Angle Error", () -> drivetrain.getModAngError(2))
    .withSize(1,1)
    .withPosition(8,2);

    swerveTab.addNumber("SwerveModule C Angle Error", () -> drivetrain.getModAngError(3))
    .withSize(1,1)
    .withPosition(7,3);

    swerveTab.addNumber("SwerveModule D Angle Error", () -> drivetrain.getModAngError(4))
    .withSize(1,1)
    .withPosition(8,3);

    //Swerve module drive voltage
    swerveTab.addNumber("SwerveModule A Drive Volts", () -> drivetrain.getModDriveVoltage(1))
    .withSize(1,1)
    .withPosition(0,0);

    swerveTab.addNumber("SwerveModule B Drive Volts", () -> drivetrain.getModDriveVoltage(2))
    .withSize(1,1)
    .withPosition(1,0);

    swerveTab.addNumber("SwerveModule C Drive Volts", () -> drivetrain.getModDriveVoltage(3))
    .withSize(1,1)
    .withPosition(0,1);

    swerveTab.addNumber("SwerveModule D Drive Volts", () -> drivetrain.getModDriveVoltage(4))
    .withSize(1,1)
    .withPosition(1,1);

    //Swerve module drive kV
    swerveTab.addNumber("SwerveModule A Drive kV", () -> drivetrain.getModDrivekV(1))
    .withSize(1,1)
    .withPosition(0,2);

    swerveTab.addNumber("SwerveModule B Drive kV", () -> drivetrain.getModDrivekV(2))
    .withSize(1,1)
    .withPosition(1,2);

    swerveTab.addNumber("SwerveModule C Drive kV", () -> drivetrain.getModDrivekV(3))
    .withSize(1,1)
    .withPosition(0,3);

    swerveTab.addNumber("SwerveModule D Drive kV", () -> drivetrain.getModDrivekV(4))
    .withSize(1,1)
    .withPosition(1,3);

    //Swerve module turn voltage
    swerveTab.addNumber("SwerveModule A Turn Volts", () -> drivetrain.getModTurnVoltage(1))
    .withSize(1,1)
    .withPosition(2,0);

    swerveTab.addNumber("SwerveModule B Turn Volts", () -> drivetrain.getModTurnVoltage(2))
    .withSize(1,1)
    .withPosition(3,0);

    swerveTab.addNumber("SwerveModule C Turn Volts", () -> drivetrain.getModTurnVoltage(3))
    .withSize(1,1)
    .withPosition(2,1);

    swerveTab.addNumber("SwerveModule D Turn Volts", () -> drivetrain.getModTurnVoltage(4))
    .withSize(1,1)
    .withPosition(3,1);

    //Swerve module turn kV
    swerveTab.addNumber("SwerveModule A Turn kV", () -> drivetrain.getModTurnkV(1))
    .withSize(1,1)
    .withPosition(2,2);

    swerveTab.addNumber("SwerveModule B Turn kV", () -> drivetrain.getModTurnkV(2))
    .withSize(1,1)
    .withPosition(3,2);

    swerveTab.addNumber("SwerveModule C Turn kV", () -> drivetrain.getModTurnkV(3))
    .withSize(1,1)
    .withPosition(2,3);

    swerveTab.addNumber("SwerveModule D Turn kV", () -> drivetrain.getModTurnkV(4))
    .withSize(1,1)
    .withPosition(3,3);

      //Tells if it is practice bot for different swevrve module offsets
    diagnosticsTab.addBoolean("Is Practice Bot", () -> Constants.PracticeBot)
        .withSize(1,1)
        .withPosition(4,1);

    diagnosticsTab.addBoolean("Is Comp Bot", () -> Constants.CompBot)
        .withSize(1,1)
        .withPosition(4,2);

    diagnosticsTab.addNumber("Wrist Angle", () -> endEffector.getWristAngle())
        .withSize(1,1)
        .withPosition(2,3);

    diagnosticsTab.addBoolean("Arm At Limit", () -> arm.atCrankLimit())
        .withSize(1,1)
        .withPosition(4,0);

    /**diagnosticsTab.addNumber("AbsoluteAngle", () -> endEffector.getAbsoluteAngle())
        .withSize(1,1)
        .withPosition(3,3);*/

    /**diagnosticsTab.addNumber("Robot Yaw", () -> drivetrain.getYawDegrees())
        .withSize(1,1)
        .withPosition(0,0);

    diagnosticsTab.addNumber("X-Position", () -> drivetrain.getPose().getX())
        .withSize(1,1)
        .withPosition(1,0);

    //putting the field in in case we want to have that visualization on shuffleboard
    diagnosticsTab.add(field)
        .withSize(4,3)
        .withPosition(1,1);

    diagnosticsTab.addString("Vision Estimated Pose", () -> drivetrain.getPose().toString())
        .withSize(4,1)
        .withPosition(0,4);

    diagnosticsTab.add("AutoAlignAprilTag", new AutoAlignAprilTag(drivetrain, drivetrain.getCamera()))
        .withSize(2,1)
        .withPosition(0,2);

    diagnosticsTab.addNumber("Target Yaw", () -> drivetrain.getTargetYaw())
        .withSize(1,1)
        .withPosition(0,3);

    diagnosticsTab.add("AutoMoveToAprilTag", new AutoMoveToAprilTag(drivetrain, drivetrain.getCamera()).move())
        .withSize(2,1)
        .withPosition(7, 0);

    diagnosticsTab.add("AutoBalance", new AutoBalance(drivetrain))
        .withSize(2,1)
        .withPosition(7,2);

    diagnosticsTab.addString("Pose", () -> drivetrain.getPose().toString())
        .withSize(2,1)
        .withPosition(7,1); */

    diagnosticsTab.addNumber("CrankAngle", () -> arm.getCrankAngle())
        .withSize(1,1)
        .withPosition(1,1);

    diagnosticsTab.addNumber("GoalVelocity", () -> arm.getGoalVelocity())
        .withSize(1,1)
        .withPosition(3,1);

    diagnosticsTab.addNumber("ArmVoltage", () -> arm.getVoltage())
        .withSize(1,1)
        .withPosition(5,1);


    diagnosticsTab.addNumber("RobotPitch", () -> drivetrain.getPitch())
        .withSize(1,1)
        .withPosition(7,1);

    diagnosticsTab.addNumber("RobotRoll", () -> drivetrain.getRoll())
        .withSize(1,1)
        .withPosition(8,1);

    diagnosticsTab.addNumber("Neo Voltage", () -> endEffector.getNeoVoltage())
        .withSize(1,1)
        .withPosition(4,3);

    /**diagnosticsTab.addNumber("Charging Station Angle", () -> drivetrain.getChargeStationAngle())
        .withSize(1,1)
        .withPosition(7,3);*/

    driverTab.addNumber("Battery Voltage", () -> RobotController.getBatteryVoltage())
        .withSize(1,1)
        .withPosition(7,3);

    diagnosticsTab.addString("Pose:", () -> drivetrain.getPose().toString())
        .withSize(3,1)
        .withPosition(5,2);

    diagnosticsTab.addString("Vision Estimated Pose:", () -> drivetrain.getVisionEstimatedPose().toString())
        .withSize(3,1)
        .withPosition(5,3);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return m_AutoChooser.getSelected();
  }
}
