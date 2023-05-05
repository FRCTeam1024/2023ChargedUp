package frc.robot;

import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.ProxyCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.AutoBalance;
import frc.robot.commands.AutoMoveToAprilTag;
import frc.robot.commands.PathPlannerCommand;
import frc.robot.Constants.*;

public class RobotAutoContainer extends RobotContainer {
    
    public RobotAutoContainer() {
        super();
    }

    protected Command returnAutoCommand(){
        return drivetrain.followTrajectory(PathPlanner.loadPath("Test Path", new PathConstraints(2.5, 2.5)));
    }

    // Test path that moves in a straight line and back, while turning
    protected Command TestAuto(){
        PathPlannerTrajectory path = PathPlanner.loadPath("Test Path", new PathConstraints(2.5, 2.5));
        return new SequentialCommandGroup(
          new InstantCommand(() -> drivetrain.resetPosition(path.getInitialHolonomicPose())),
          new ParallelCommandGroup(
            new PPSwerveControllerCommand(
              path,
              drivetrain::getPose, // Pose supplier
              drivetrain.getSwerveDriveKinematics(), // SwerveDriveKinematics
              new PIDController(7.5, 0, 0), // X controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
              new PIDController(7.5, 0, 0), // Y controller (usually the same values as X controller)
              new PIDController(0.5, 0, 0.005), // Rotation controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
              drivetrain::setModuleStates, // Module states consumer
              drivetrain // Requires this drive subsystem
            )
          )
        );
      }
    
      //Attempting to pass auto into a separate command has not worked, even with the exact same arguments
      // Test path that moves into a circle
      protected Command TestAuto2(){
        PathPlannerTrajectory path = PathPlanner.loadPath("Circle Path", new PathConstraints(1, 1));
        return new SequentialCommandGroup(
          new PathPlannerCommand(path,drivetrain,true).configure()
        );
      }
      //Initial test of just driving on to the charging station and running auto balance
      protected Command TestAutoBalance(){
        PathPlannerTrajectory path = PathPlanner.loadPath("AutoBalanceTest", new PathConstraints(2.5, 2.5));
        return new SequentialCommandGroup(
          new InstantCommand(() -> drivetrain.resetPosition(path.getInitialHolonomicPose())),
          new ParallelCommandGroup(
            new PPSwerveControllerCommand(
              path,
              drivetrain::getPose, // Pose supplier
              drivetrain.getSwerveDriveKinematics(), // SwerveDriveKinematics
              new PIDController(7.5, 0, 0), // X controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
              new PIDController(7.5, 0, 0), // Y controller (usually the same values as X controller)
              new PIDController(0.5, 0, 0.005), // Rotation controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
              drivetrain::setModuleStates, // Module states consumer
              drivetrain // Requires this drive subsystem
            )
          ),
          new AutoBalance(drivetrain)
        );
      }
    
      //Moves from the center grid, on to the charge station, then attempts to auto balance
      protected Command C_Charge(){
        PathPlannerTrajectory path = PathPlanner.loadPath("C_Charge", new PathConstraints(1, 1));
        /**return new SequentialCommandGroup(
          new ProxyCommand(() -> arm.moveTo(-75)).withTimeout(1),
          new InstantCommand(() -> endEffector.releaseCube()),
          new WaitCommand(1),
          new InstantCommand(() -> endEffector.stop()),
          new ParallelDeadlineGroup(
            drivetrain.followTrajectory(path),
            //arm.moveTo(ArmConstants.stowLevel) just removed for sketchy testing
            new ProxyCommand(() -> arm.moveTo(-120)).withTimeout(1)
          ),
          new PrintCommand("Does this finish these commands?")
          new AutoBalance(drivetrain)
        
        );*/
        return new SequentialCommandGroup(
          new InstantCommand(() -> endEffector.resetWristAngle()),
          drivetrain.followTrajectory(path).withTimeout(6),
          new AutoBalance(drivetrain)
        );
      }
    
      //Moves from center grid onto and over the charge station, and then moves directly back onto the charge station to auto balance
      protected Command C_Cross_Charge(){
        PathPlannerTrajectory path = PathPlanner.loadPath("C-Cross-Charge", new PathConstraints(1.5,1.5));
        return new SequentialCommandGroup(
          /**new PrintCommand("\n\n" + path.getEndState().toString() + "\n\n"),
          arm.moveTo(ArmConstants.lowLevel),
          new InstantCommand(() -> endEffector.releaseCube()),
          new WaitCommand(0.5),
          new InstantCommand(() -> endEffector.stop()),
          new ParallelDeadlineGroup(
            drivetrain.followTrajectory(path),
            arm.moveTo(ArmConstants.stowLevel)
          ),*/
          new InstantCommand(() -> endEffector.resetWristAngle()),
          drivetrain.followTrajectory(path),
          new AutoBalance(drivetrain)
        );
      }
    
        //Places a high cone Moves from center grid onto and over the charge station, and then moves directly back onto the charge station to auto balance
      protected Command C_Cone_Cross_Charge(){
        List<PathPlannerTrajectory> path = PathPlanner.loadPathGroup("C-Cone-Cross-Charge", 
                                              new PathConstraints(1.5,1.5),
                                              new PathConstraints(2,2),
                                              new PathConstraints(2.5,2.5));
        return new SequentialCommandGroup(
          new InstantCommand(() -> endEffector.resetWristAngle()),
          new ParallelCommandGroup(
            arm.moveToAuto(14).withTimeout(2.5),
            new SequentialCommandGroup(
              new WaitCommand(1),
              drivetrain.followTrajectory(path.get(0))
            )
          ),
          new ParallelCommandGroup(
            arm.moveToAuto(-5).withTimeout(1),
            new SequentialCommandGroup(
              new WaitCommand(0.7),
              new InstantCommand(() -> endEffector.runIntakeAuto(-0.3))
            )
          ),
          new ParallelCommandGroup(
            new SequentialCommandGroup(
              drivetrain.followTrajectory(path.get(1)),
              new PrintCommand("Over the charge station"),
              drivetrain.followTrajectory(path.get(2))
            ),
            new SequentialCommandGroup(
              arm.moveToAuto(ArmConstants.highLevel).withTimeout(0.5),
              new InstantCommand(() -> endEffector.stop()),
              arm.moveToAuto(ArmConstants.stowLevel).withTimeout(3)
            )
          ),
          new PrintCommand("AutoBalancing"),
          new AutoBalance(drivetrain),
          new PrintCommand("Done with Autobalancing")
        );
      }
    
      protected Command C_Cone_Charge(){
        List<PathPlannerTrajectory> path = PathPlanner.loadPathGroup("C-Cone-Charge", 
                                              new PathConstraints(1.5,1.5),
                                              new PathConstraints(2,2),
                                              new PathConstraints(2.5,2.5));
        return new SequentialCommandGroup(
          new InstantCommand(() -> endEffector.resetWristAngle()),
          new ParallelCommandGroup(
            arm.moveToAuto(14).withTimeout(2.5),
            new SequentialCommandGroup(
              new WaitCommand(1),
              drivetrain.followTrajectory(path.get(0))
            )
          ),
          arm.moveToAuto(-5).withTimeout(1),
          new InstantCommand(() -> endEffector.runIntakeAuto(-0.3)),
          new ParallelCommandGroup(
            new SequentialCommandGroup(
              drivetrain.followTrajectory(path.get(1))
            ),
            new SequentialCommandGroup(
              arm.moveToAuto(ArmConstants.highLevel).withTimeout(0.5),
              new InstantCommand(() -> endEffector.stop()),
              arm.moveToAuto(ArmConstants.stowLevel).withTimeout(2)
            )
          ),
          new PrintCommand("AutoBalancing"),
          new AutoBalance(drivetrain),
          new PrintCommand("Done with Autobalancing")
        );
      }
    
        //Places a high cone Moves from center grid onto and over the charge station, and then moves directly back onto the charge station to auto balance
        //needto rearrange lift up for cube
      protected Command C_Cube_Cross_Charge(){
        List<PathPlannerTrajectory> path = PathPlanner.loadPathGroup("C-Cone-Cross-Charge", 
                                              new PathConstraints(1.5,1.5),
                                              new PathConstraints(2,2),
                                              new PathConstraints(2.5,2.5));
        return new SequentialCommandGroup(
          new InstantCommand(() -> endEffector.resetWristAngle()),
          new ParallelCommandGroup(
            arm.moveToAuto(14).withTimeout(2.5),
            new SequentialCommandGroup(
              new WaitCommand(1),
              drivetrain.followTrajectory(path.get(0))
            )
          ),
          arm.moveToAuto(-5).withTimeout(1),
          new InstantCommand(() -> endEffector.runIntakeAuto(0.3)),
          new ParallelCommandGroup(
            new SequentialCommandGroup(
              drivetrain.followTrajectory(path.get(1)),
              drivetrain.followTrajectory(path.get(2))
            ),
            new SequentialCommandGroup(
              arm.moveToAuto(ArmConstants.highLevel).withTimeout(0.5),
              new InstantCommand(() -> endEffector.stop()),
              arm.moveToAuto(ArmConstants.stowLevel).withTimeout(3)
            )
          ),
          new AutoBalance(drivetrain)
        );
      }
    
      protected Command OuterConeGrab(){
        List<PathPlannerTrajectory> path = PathPlanner.loadPathGroup("2OuterCone",
                                          new PathConstraints(1.5,1.5),
                                          new PathConstraints(2,2), //next up this speed, verify functionality
                                          new PathConstraints(0,0), //3rd path isn't used
                                          new PathConstraints(0,0)); //4th path isn't used
        return new SequentialCommandGroup(
          new InstantCommand(() -> endEffector.resetWristAngle()),
          new ParallelCommandGroup(
            arm.moveToAuto(ArmConstants.highLevel).withTimeout(2.5),
            new SequentialCommandGroup(
              new WaitCommand(1),
              drivetrain.followTrajectory(path.get(0))
            )
          ),
          arm.moveToAuto(-5).withTimeout(1),
          new InstantCommand(() -> endEffector.intakeCube()),
          arm.moveToAuto(ArmConstants.highLevel).withTimeout(1),
          new ParallelCommandGroup(
            drivetrain.followTrajectory(path.get(1)),
            new SequentialCommandGroup(
              new InstantCommand(() -> endEffector.intakeCube()),
              new ProxyCommand(() -> endEffector.turnWristToAngle(-200)).withTimeout(1)
            ),
            new SequentialCommandGroup(
              new WaitCommand(0.25), //might need to decrease this time to get the arm down fast enough
              arm.moveToAuto(ArmConstants.pickup).withTimeout(3) //check whether the arm or path is completed faster
            )
          ),
          arm.moveToAuto(-85).withTimeout(0.3),
          new WaitCommand(1),
          new InstantCommand(() -> endEffector.stop())
        );
      }
    
      protected Command OuterCones(){
        List<PathPlannerTrajectory> path = PathPlanner.loadPathGroup("2OuterCone",
                                          new PathConstraints(1.5,1.5),
                                          new PathConstraints(2,2), //next up this speed, verify functionality
                                          new PathConstraints(2,2),
                                          new PathConstraints(0,0)); //4th path isn't used
        return new SequentialCommandGroup(
          new InstantCommand(() -> endEffector.resetWristAngle()),
          new ParallelCommandGroup(
            arm.moveToAuto(ArmConstants.highLevel).withTimeout(2.5),
            new SequentialCommandGroup(
              new WaitCommand(1),
              drivetrain.followTrajectory(path.get(0))
            )
          ),
          arm.moveToAuto(-5).withTimeout(1),
          new InstantCommand(() -> endEffector.intakeCube()),
          arm.moveToAuto(ArmConstants.highLevel).withTimeout(1),
          new ParallelCommandGroup(
            drivetrain.followTrajectory(path.get(1)),
            new SequentialCommandGroup(
              new InstantCommand(() -> endEffector.intakeCube()),
              new ProxyCommand(() -> endEffector.turnWristToAngle(-200)).withTimeout(1)
            ),
            new SequentialCommandGroup(
              new WaitCommand(0.25), //might need to decrease this time to get the arm down fast enough
              arm.moveToAuto(ArmConstants.pickup).withTimeout(3) //check whether the arm or path is completed faster
            )
          ),
          arm.moveToAuto(-85).withTimeout(0.3),
          new ParallelCommandGroup(
            drivetrain.followTrajectory(path.get(2)),
            new SequentialCommandGroup(
              new WaitCommand(1), //if we up the speed, this may need to decrease a little bit
              arm.moveToAuto(ArmConstants.highLevel).withTimeout(3)//check whether the arm or path is completed faster
            ),
            new SequentialCommandGroup(
              new WaitCommand(2.5),
              new InstantCommand(() -> endEffector.stop())
            )
          ),
          new InstantCommand(() -> endEffector.runIntake(0.8)),
          new WaitCommand(0.25), //can decrease this time
          new InstantCommand(() -> endEffector.stop())
        );
      }
    
      protected Command OuterConesCharge(){
        List<PathPlannerTrajectory> path = PathPlanner.loadPathGroup("2OuterCone",
                                          new PathConstraints(1.5,1.5),
                                          new PathConstraints(2,2), //next up this speed, verify functionality
                                          new PathConstraints(2,2),
                                          new PathConstraints(3,3)); //can up this speed
        return new SequentialCommandGroup(
          new InstantCommand(() -> endEffector.resetWristAngle()),
          new ParallelCommandGroup(
            arm.moveToAuto(ArmConstants.highLevel).withTimeout(2.5),
            new SequentialCommandGroup(
              new WaitCommand(1),
              drivetrain.followTrajectory(path.get(0))
            )
          ),
          arm.moveToAuto(-5).withTimeout(1),
          new InstantCommand(() -> endEffector.intakeCube()),
          arm.moveToAuto(ArmConstants.highLevel).withTimeout(1),
          new ParallelCommandGroup(
            drivetrain.followTrajectory(path.get(1)),
            new SequentialCommandGroup(
              new InstantCommand(() -> endEffector.intakeCube()),
              new ProxyCommand(() -> endEffector.turnWristToAngle(-200)).withTimeout(1)
            ),
            new SequentialCommandGroup(
              new WaitCommand(0.25), //might need to decrease this time to get the arm down fast enough
              arm.moveToAuto(ArmConstants.pickup).withTimeout(3) //check whether the arm or path is completed faster
            )
          ),
          arm.moveToAuto(-85).withTimeout(0.3),
          new ParallelCommandGroup(
            drivetrain.followTrajectory(path.get(2)),
            new SequentialCommandGroup(
              new WaitCommand(1), //if we up the speed, this may need to decrease a little bit
              arm.moveToAuto(ArmConstants.highLevel).withTimeout(3)//check whether the arm or path is completed faster
            ),
            new SequentialCommandGroup(
              new WaitCommand(2.5),
              new InstantCommand(() -> endEffector.stop())
            )
          ),
          new InstantCommand(() -> endEffector.runIntake(0.8)),
          new WaitCommand(0.25), //can decrease this time
          new InstantCommand(() -> endEffector.stop()),//cutoff from last path
          new ParallelCommandGroup(
            drivetrain.followTrajectory(path.get(3)),
            new SequentialCommandGroup(
              new WaitCommand(0.5), //can decrease this time
              arm.moveToAuto(ArmConstants.stowLevel).withTimeout(2.5)
            )
          ),
          new AutoBalance(drivetrain)
        );
      }
    
      protected Command OuterConesPlusOne(){
        List<PathPlannerTrajectory> path = PathPlanner.loadPathGroup("2.5OuterCone",
                                          new PathConstraints(1.5,1.5),
                                          new PathConstraints(2.2,2.2), //next up this speed, verify functionality
                                          new PathConstraints(2.5,2.5),
                                          new PathConstraints(3,3)); //can up this speed
        return new SequentialCommandGroup(
          new InstantCommand(() -> endEffector.resetWristAngle()),
          new ParallelCommandGroup(
            arm.moveToAuto(ArmConstants.highLevel).withTimeout(2.5),
            new SequentialCommandGroup(
              new WaitCommand(1),
              drivetrain.followTrajectory(path.get(0))
            )
          ),
          arm.moveToAuto(-5).withTimeout(0.5),
          new InstantCommand(() -> endEffector.intakeCube()),
          arm.moveToAuto(ArmConstants.highLevel).withTimeout(0.5),
          new ParallelCommandGroup(
            drivetrain.followTrajectory(path.get(1)),
            new SequentialCommandGroup(
              new InstantCommand(() -> endEffector.intakeCube()),
              new ProxyCommand(() -> endEffector.turnWristToAngle(-200)).withTimeout(1)
            ),
            new SequentialCommandGroup(
              new WaitCommand(0.25), //might need to decrease this time to get the arm down fast enough
              arm.moveToAuto(ArmConstants.pickup).withTimeout(3) //check whether the arm or path is completed faster
            )
          ),
          arm.moveToAuto(-85).withTimeout(0.3),
          new ParallelCommandGroup(
            drivetrain.followTrajectory(path.get(2)),
            new SequentialCommandGroup(
              new WaitCommand(1), //if we up the speed, this may need to decrease a little bit
              arm.moveToAuto(ArmConstants.highLevel).withTimeout(2.5)//check whether the arm or path is completed faster
            ),
            new SequentialCommandGroup(
              new WaitCommand(2.5),
              new InstantCommand(() -> endEffector.stop())
            )
          ),
          new InstantCommand(() -> endEffector.runIntake(0.8)),
          new WaitCommand(0.25), //can decrease this time
          new InstantCommand(() -> endEffector.stop()),//cutoff from last path
          new ParallelCommandGroup(
            drivetrain.followTrajectory(path.get(3)),
            new SequentialCommandGroup(
              new WaitCommand(0.5), //can decrease this time
              new InstantCommand(() -> endEffector.intakeCube()),
              arm.moveToAuto(ArmConstants.pickup).withTimeout(2.5)
            )
          )
        );
      }

      protected Command LowLink(){
        List<PathPlannerTrajectory> path = PathPlanner.loadPathGroup("Low Outer Link",
                                          new PathConstraints(3.2,2.8),
                                          new PathConstraints(3.2,2.8),
                                          new PathConstraints(3.2,2.8),
                                          new PathConstraints(3.2,2.8));
        return new SequentialCommandGroup(
          new ParallelCommandGroup(
            new InstantCommand(() -> endEffector.intakeCube()),
            new ProxyCommand(() -> arm.moveToAuto(-85)).withTimeout(0.25)
          ),
          //new WaitCommand(0.25), see if we drop the cone with no delay
          new ParallelCommandGroup(
            new ProxyCommand(() -> endEffector.turnWristToAngle(-200)).withTimeout(1),
            drivetrain.followTrajectory(path.get(0)),
            new SequentialCommandGroup(
              new ProxyCommand(() -> arm.moveToAuto(ArmConstants.pickup)).withTimeout(0.75),
              new WaitCommand(0.5),
              new ProxyCommand(() -> arm.moveToAuto(-93)).withTimeout(0.5)
            )
          ),
          new ParallelCommandGroup(
            drivetrain.followTrajectory(path.get(1)),
            new SequentialCommandGroup(
              new ProxyCommand(() -> arm.moveToAuto(-70)).withTimeout(0.75), //arm angle may need to rise, need to check with inflated cubes
              new InstantCommand(() -> endEffector.stop()),
              new WaitCommand(1.6),
              new InstantCommand(() -> endEffector.releaseCube())
            )
          ),
          new WaitCommand(0.15),
          new ParallelCommandGroup(
            drivetrain.followTrajectory(path.get(2)),
            new SequentialCommandGroup(
              new InstantCommand(() -> endEffector.intakeCube()),
              new WaitCommand(1.4),
              new ProxyCommand(() -> arm.moveToAuto(-93)).withTimeout(0.75)
            )
          ),
          new ParallelCommandGroup(
            drivetrain.followTrajectory(path.get(3)),
            new SequentialCommandGroup(
              new ProxyCommand(() -> arm.moveToAuto(ArmConstants.pickup)).withTimeout(1),
              new InstantCommand(() -> endEffector.stop()),
              new WaitCommand(2.2),
              new InstantCommand(() -> endEffector.releaseCube())
            )
          ),
          new ProxyCommand(() -> arm.moveToAuto(-60)).withTimeout(1)
        );
      }
    
      protected Command InnerCones(){
        List<PathPlannerTrajectory> path = PathPlanner.loadPathGroup("2InnerCone",
                                          new PathConstraints(1.5,1.5),
                                          new PathConstraints(1.5,1.5),
                                          new PathConstraints(2,2),
                                          new PathConstraints(2,2));
        return new SequentialCommandGroup(
          new InstantCommand(() -> endEffector.resetWristAngle()),
          new ParallelCommandGroup(
            arm.moveToAuto(ArmConstants.highLevel).withTimeout(2.5),
            new SequentialCommandGroup(
              new WaitCommand(1),
              drivetrain.followTrajectory(path.get(0))
            )
          ),
          arm.moveToAuto(-5).withTimeout(1),
          new InstantCommand(() -> endEffector.intakeCube()),
          arm.moveToAuto(ArmConstants.highLevel).withTimeout(1),
          new ParallelCommandGroup(
            drivetrain.followTrajectory(path.get(1)),
            new SequentialCommandGroup(
              new WaitCommand(0.5),
              arm.moveToAuto(ArmConstants.stowLevel).withTimeout(2.5)
            ),
            endEffector.turnWristToOffset(-10).withTimeout(1)
          ),
          new ParallelCommandGroup(
            new InstantCommand(() -> endEffector.intakeCube()),
            drivetrain.followTrajectory(path.get(2)),
            new SequentialCommandGroup(
              new WaitCommand(2.3),
              arm.moveToAuto(ArmConstants.pickup).withTimeout(1)
            )
          ),
          new WaitCommand(1),
          new InstantCommand(() -> endEffector.stop()),
          new ParallelCommandGroup(
            new SequentialCommandGroup(      
              arm.moveToAuto(ArmConstants.stowLevel).withTimeout(1),
              new WaitCommand(0.5),
              arm.moveToAuto(ArmConstants.highLevel).withTimeout(2.5)
            ),
            new SequentialCommandGroup(
              new WaitCommand(0.5),
              drivetrain.followTrajectory(path.get(3))
            )
          ),
          new InstantCommand(() -> endEffector.runIntakeAuto(-0.5))
        );
      }
    
      // Moves from center grid, to the first cube, picks it up, and then moves to the outer grid to score it.
      protected Command C_1_O(){
        List<PathPlannerTrajectory> path = PathPlanner.loadPathGroup("C-1-O",
                                          new PathConstraints(1,1),
                                          new PathConstraints(1,1));
        return new SequentialCommandGroup(
          arm.moveTo(ArmConstants.lowLevel),
          new InstantCommand(() -> endEffector.releaseCube()),
          new WaitCommand(0.5),
          new InstantCommand(() -> endEffector.stop()),
          new ParallelDeadlineGroup(
            drivetrain.followTrajectory(path.get(0))
          ),
          new InstantCommand(() -> endEffector.intakeCube()),
          new WaitCommand(0.5),
          new InstantCommand(() -> endEffector.stop()),
          new ParallelDeadlineGroup(
            drivetrain.followTrajectory(path.get(1))
          ),
          new InstantCommand(() -> endEffector.releaseCube()),
          new WaitCommand(0.5),
          new InstantCommand(() -> endEffector.stop())
        );
      }
    
      // Moves from center grid, to the second cube, picks it up, and then moves to the outer grid to score it.
      protected Command C_2_O(){
        PathPlannerTrajectory path = PathPlanner.loadPath("C-2-O", new PathConstraints(1,1));
        return new SequentialCommandGroup(
          drivetrain.followTrajectory(path)
        );
      }
      // Moves from center grid, to the third cube, picks it up, and then moves to the inner grid to score it.
      protected Command C_3_I(){
        PathPlannerTrajectory path = PathPlanner.loadPath("C-3-I", new PathConstraints(1,1));
        return new SequentialCommandGroup(
          drivetrain.followTrajectory(path)
        );
      }
      // Moves from center grid, to the fourth cube, picks it up, and then moves to the inner grid to score it.
      protected Command C_4_I(){
        PathPlannerTrajectory path = PathPlanner.loadPath("C-4-I", new PathConstraints(2,2));
        return new SequentialCommandGroup(
          drivetrain.followTrajectory(path)
        );
      }
      // Moves from the center grid, towards the inner grid, and then proceeds to go all the way around, drives up onto the charge station and auto balances.
      protected Command C_InnerRoute_Charge(){
        PathPlannerTrajectory path = PathPlanner.loadPath("C-InnerRoute-Charge", new PathConstraints(1,1));
        return new SequentialCommandGroup(
          drivetrain.followTrajectory(path),
          new AutoBalance(drivetrain)
        );
      }
      // Moves from the center grid, towards the outer grid, and then proceeds to go all the way around, drives up onto the charge station and auto balances.
      protected Command C_OuterRoute_Charge(){
        PathPlannerTrajectory path = PathPlanner.loadPath("C-OuterRoute-Charge", new PathConstraints(1.5,1.5));
        return new SequentialCommandGroup(
          drivetrain.followTrajectory(path),
          new AutoBalance(drivetrain)
        );
      }
      // Moves from inner grid, to the third cube, picks it up, and then moves back to the inner grid to score it.
      protected Command I_3_I(){
       List<PathPlannerTrajectory> path = PathPlanner.loadPathGroup("I-3-I", new PathConstraints(2,2),
                                                                                  new PathConstraints(2,2));
        return new SequentialCommandGroup(
          new ParallelDeadlineGroup(
            new SequentialCommandGroup(
              drivetrain.followTrajectory(path.get(0)),
              new WaitCommand(1)
            ),
            new ProxyCommand(() -> arm.moveTo(ArmConstants.lowLevel)),
            new InstantCommand(() -> endEffector.intakeCube())
          ),
          new InstantCommand(() -> endEffector.stop()),
          new ParallelDeadlineGroup(
            new WaitCommand(4),
            drivetrain.followTrajectory(path.get(1)),
            new SequentialCommandGroup(
              new WaitCommand(2),
              new ProxyCommand(() -> arm.moveTo(ArmConstants.midLevel))
            )
          ),
          new AutoMoveToAprilTag(drivetrain, drivetrain.getCamera()).withTimeout(1),
          new InstantCommand(() -> endEffector.releaseCube()),
          new WaitCommand(0.5),
          new InstantCommand(() -> endEffector.stop())
        );
      }
      // Moves from inner grid, to the third cube, picks it up, and then moves back to the inner grid to score it.
      protected Command I_4_I(){
        List<PathPlannerTrajectory> path = PathPlanner.loadPathGroup("I-4-I", new PathConstraints(2.5,2.5),
                                                                                    new PathConstraints(2.5,2.5));
        return new SequentialCommandGroup(
          new ParallelDeadlineGroup(
            new InstantCommand(() -> endEffector.resetWristAngle()),
            new SequentialCommandGroup(
              drivetrain.followTrajectory(path.get(0)),
              new WaitCommand(2)
            ),
            new ProxyCommand(() -> arm.moveTo(ArmConstants.pickup)),
            new InstantCommand(() -> endEffector.intakeCube()),
            new ProxyCommand(endEffector.turnWristToAngle(150))
          ),
          new InstantCommand(() -> endEffector.stop()),
          new ParallelDeadlineGroup(
            new WaitCommand(4),
            drivetrain.followTrajectory(path.get(1)),
            new SequentialCommandGroup(
              new WaitCommand(0.5),
              new ProxyCommand(() -> arm.moveTo(ArmConstants.midLevel))
            )
          ),
          //new AutoMoveToAprilTag(drivetrain, drivetrain.getCamera()).withTimeout(1),
          new InstantCommand(() -> endEffector.releaseCube()),
          new WaitCommand(0.5),
          new InstantCommand(() -> endEffector.stop())
        );
      }
      // Moves from the inner grid, goes around the side, and then moves up onto the charge station to auto balance
      protected Command I_Charge(){
        PathPlannerTrajectory path = PathPlanner.loadPath("I-Charge", new PathConstraints(1,1));
        return new SequentialCommandGroup(
          drivetrain.followTrajectory(path)
        );
      }
      // Moves from outer grid, to the first cube, picks it up, and then moves back to the outer grid to score it.
      protected Command O_1_O(){
        //PathPlannerTrajectory path = PathPlanner.loadPath("O-1-O", new PathConstraints(1,1));
        List<PathPlannerTrajectory> path = PathPlanner.loadPathGroup("O-1-O", new PathConstraints(2,2), new PathConstraints(2,2));
        return new SequentialCommandGroup(
          new ParallelDeadlineGroup(
            new SequentialCommandGroup(
              new InstantCommand(() -> endEffector.resetWristAngle()),
              drivetrain.followTrajectory(path.get(0)),
              new WaitCommand(2)
            ),
            new SequentialCommandGroup(
              new WaitCommand(1),
              new InstantCommand(() -> endEffector.intakeCube()),
              new ProxyCommand(() -> endEffector.turnWristToAngle(130)).withTimeout(2)
            ),
            new SequentialCommandGroup(
              new ProxyCommand(() -> arm.moveTo(ArmConstants.pickup))
            )
          ),
          new ParallelDeadlineGroup(
            new WaitCommand(5),
            drivetrain.followTrajectory(path.get(1)),
            new SequentialCommandGroup(
              new WaitCommand(1),
              new InstantCommand(() -> endEffector.stop()),
              new ProxyCommand(() -> arm.moveTo(ArmConstants.midLevel))
            )
          ),
          //new ProxyCommand(() -> new AutoMoveToAprilTag(drivetrain, drivetrain.getCamera()).withTimeout(1)),
          new InstantCommand(() -> endEffector.releaseCube()),
          new WaitCommand(0.5),
          new InstantCommand(() -> endEffector.stop())
        );
      }
      // Moves from outer grid, to the second cube, picks it up, and then moves back to the outer grid to score it.
      protected Command O_2_O(){
        PathPlannerTrajectory path = PathPlanner.loadPath("O-2-O", new PathConstraints(1,1));
        return new SequentialCommandGroup(
          drivetrain.followTrajectory(path)
        );
      }
    // Moves from the outer grid, goes around the side, then goes onto the charge station to auto balance
    protected Command O_Charge(){
        PathPlannerTrajectory path = PathPlanner.loadPath("O_Charge", new PathConstraints(1,1));
        return new SequentialCommandGroup(
          drivetrain.followTrajectory(path)
        );
    }
}
