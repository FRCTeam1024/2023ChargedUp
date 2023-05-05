// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;

import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ProxyCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.DriveWithJoysticks;
import frc.robot.commands.TurnWristWithJoysticks;
import frc.robot.oi.Logitech;
import frc.robot.Constants.*;
import frc.robot.subsystems.EndEffector;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.SwerveDrive;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  //Subsystems
  protected final SwerveDrive drivetrain = new SwerveDrive();
  protected final Arm arm = new Arm();
  protected final EndEffector endEffector = new EndEffector();

  //Operator Inputs
  private final Logitech driverController = new Logitech(0);
  private final Logitech operatorController = new Logitech(1);

  //Default Commands
  private final DriveWithJoysticks driveWithController = new DriveWithJoysticks(drivetrain, driverController, .58, arm);
  private final TurnWristWithJoysticks turnWristWithJoysticks = new TurnWristWithJoysticks(endEffector, operatorController);
 
  //Chooser for auto
  SendableChooser<Command> m_AutoChooser = new SendableChooser<>();

  Field2d field = new Field2d();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    resetDriver();

    //Assign default commands
    drivetrain.setDefaultCommand(driveWithController);
    endEffector.setDefaultCommand(turnWristWithJoysticks);

    // Configure the button bindings
    configureButtonBindings();
  }

    /**
   * List any PID subsystems here so that they get disabled when the robot
   * is disabled and integral error doesn't accumulate.  Usually doesn't matter
   * since we often don't use integral gain but just in case.
   */
  public void disablePIDSubsystems() {


  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    //need to add button to switch from field to robot relative

    // DRIVER CONTROLS
    driverController.aButton.onTrue(new InstantCommand(() -> drivetrain.setHeading180()));
    driverController.yButton.onTrue(new InstantCommand(() -> drivetrain.zeroHeading()));
    driverController.leftTrigger.whileTrue(new DriveWithJoysticks(drivetrain,driverController,0.35, arm));
    driverController.leftBumper.whileTrue(new DriveWithJoysticks(drivetrain,driverController,1, arm));

    //driverController.dPadUp.whileTrue(new AutoTurn(drivetrain, 0));
    //driverController.dPadDown.whileTrue(new AutoTurn(drivetrain, 180));
 

    /**driverController.rightBumper.onTrue(new ProxyCommand(
      () -> drivetrain.followVisionTrajectory()
    ));*/
    

    //OPERATOR CONTROLS


    //controls for arm - could change if hand needs more buttons
    operatorController.dPadUp.whileTrue(new ProxyCommand(() -> arm.moveTo(ArmConstants.highLevel)));  //Use moveTo(ArmConstant.MaxArmAngle)
    operatorController.dPadDown.whileTrue(new ProxyCommand(() -> arm.moveTo(ArmConstants.stowLevel)));  //Use moveTo(ArmConstants.MinArmAngle)

    operatorController.aButton.onTrue(new ProxyCommand(() -> arm.moveTo(ArmConstants.stowLevel)));
    operatorController.xButton.onTrue(new ProxyCommand(() -> arm.moveTo(ArmConstants.pickup)));
    operatorController.bButton.onTrue(new ProxyCommand(() -> arm.moveTo(ArmConstants.midLevel)));
    operatorController.yButton.onTrue(new ProxyCommand(() -> arm.moveTo(6)));

    //need to test and see if these should be instantcommands or proxycommands, as well as if we need an automatic stop after movement
    /**operatorController.dPadLeft.whileTrue(new InstantCommand(() -> endEffector.turnWrist(0.6)));
    operatorController.dPadLeft.onFalse(new InstantCommand(() -> endEffector.stop()));
    operatorController.dPadRight.whileTrue(new InstantCommand(() -> endEffector.turnWrist(-0.6)));
    operatorController.dPadRight.onFalse(new InstantCommand(() -> endEffector.stop()));*/

    //if no more reverse cones, change the below angle to 120 or something higher
    operatorController.leftTrigger.whileTrue(new ProxyCommand(() -> endEffector.turnWristToAngle(95)));
    operatorController.leftBumper.whileTrue(new ProxyCommand(() -> endEffector.turnWristToAngle(-75)));
    operatorController.rightTrigger.whileTrue(new InstantCommand(() -> endEffector.intakeCube()));
    operatorController.rightTrigger.onFalse(new InstantCommand(() -> endEffector.stop()));
    operatorController.rightBumper.whileTrue(new InstantCommand(() -> endEffector.releaseCube()));
    operatorController.rightBumper.onFalse(new InstantCommand(() -> endEffector.stop()));

    //Neo motor stall safety, prevent stalling motor for too long
    Trigger intakeStall = new Trigger(endEffector::intakeStalled).debounce(1);
    intakeStall.onTrue(new InstantCommand(endEffector::stop));

    operatorController.startButton.onTrue(new InstantCommand(() -> endEffector.resetWristAngle()));

    //Arm position calibration
    Trigger atLimit = new Trigger(arm::atCrankLimit).debounce(0.02);
    operatorController.backButton.and(atLimit.negate()).whileTrue(arm.calMove());
    atLimit.whileTrue(new SequentialCommandGroup(
                    new InstantCommand(() -> arm.simpleMove(0),arm),
                    new WaitCommand(0.1),
                    new InstantCommand(arm::resetArmAngle,arm)
    ));
                                                        
            


  }
  
  public void stopSubsystems(){
    endEffector.stop();
    drivetrain.stop();
  }

  public void resetDriver(){
    drivetrain.setHeading180();
  }
}