// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.oi.Logitech;
import frc.robot.subsystems.EndEffector;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class HoldEndEffectorPosition extends PIDCommand {
  private final EndEffector endEffector;
  private final Logitech controller;
  /** Creates a new HoldEndEffectorPosition. */
  public HoldEndEffectorPosition(EndEffector m_endEffector, Logitech controllerParam) {
    super(
        // The controller that the command will use
        new PIDController(0, 0, 0),
        // This should return the measurement
        () -> 0,
        // This should return the setpoint (can also be a constant)
        () -> 0,
        // This uses the output
        output -> {
          // Use the output here
        });
    // Use addRequirements() here to declare subsystem dependencies.
    endEffector = m_endEffector;
    controller = controllerParam;
    addRequirements(endEffector);
    // Configure additional PID options by calling `getController` here.
    endEffector.turnWristWithJoysticks(controller.getRightStickY());
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
