package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.RobotContainer;

import frc.robot.Constants.PositionClass.Positions;
public class autoMoveToL4 extends SequentialCommandGroup{


    public autoMoveToL4(){

        addRequirements(RobotContainer.arm, RobotContainer.elevator);

    addCommands(
        new WaitUntilCommand(() -> RobotContainer.intake.isNeitherCoralIntaked()), //wait until it passes the sensor
        new InstantCommand(RobotContainer.intake::stopCoral),
        new InstantCommand(() -> RobotContainer.arm.setArmPosition(Positions.L4)),
        new InstantCommand(() -> RobotContainer.elevator.setPrimaryPosition(Positions.L4)),
        new InstantCommand(() -> RobotContainer.elevator.setSecondaryPosition(Positions.L4)),
        new InstantCommand(RobotContainer.led::setBothLava)
        
         );
    }
}