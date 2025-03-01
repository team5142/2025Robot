package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.RobotContainer;

import frc.robot.Constants.PositionClass.Positions;
public class moveToPosition extends SequentialCommandGroup{


    public moveToPosition(Positions position){

        addRequirements(RobotContainer.arm, RobotContainer.elevator);

    addCommands(

        new InstantCommand(() -> RobotContainer.arm.setArmPosition(position)),
        new InstantCommand(() -> RobotContainer.elevator.setPrimaryPosition(position)),
        new InstantCommand(() -> RobotContainer.elevator.setSecondaryPosition(position)),
        new InstantCommand(() -> RobotContainer.smartdashboard.updatePosition(position))

         );
    }
}