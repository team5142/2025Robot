package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.RobotContainer;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.Constants.PositionClass;
import frc.robot.Constants.PositionClass.Positions;
public class moveToPosition extends SequentialCommandGroup{


    public moveToPosition(Positions position){

        addRequirements(RobotContainer.arm, RobotContainer.elevator);

    addCommands(

        new InstantCommand(() -> RobotContainer.arm.setArmPosition(position)),
        new InstantCommand(() -> RobotContainer.elevator.setPrimaryPosition(position)),
        new InstantCommand(() -> RobotContainer.elevator.setSecondaryPosition(position))

         );
    }
}