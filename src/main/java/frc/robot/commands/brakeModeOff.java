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
public class brakeModeOff extends SequentialCommandGroup{


    public brakeModeOff(){

        addRequirements(RobotContainer.arm, RobotContainer.elevator, RobotContainer.intake);

    addCommands(

        new InstantCommand(() -> RobotContainer.arm.turnOffBrake()),
        new InstantCommand(() -> RobotContainer.elevator.turnOffBrake()),
        new InstantCommand(() -> RobotContainer.intake.turnOffBrake())

         );
    }
}