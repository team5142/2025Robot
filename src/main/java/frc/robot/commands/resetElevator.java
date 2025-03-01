package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.RobotContainer;

public class resetElevator extends SequentialCommandGroup{


    public resetElevator(){

        addRequirements(RobotContainer.elevator);

    addCommands(

        new InstantCommand(RobotContainer.elevator::primaryDown),
        new WaitUntilCommand(RobotContainer.elevator::isPrimaryLimitSwitchPressed),
        new InstantCommand(RobotContainer.elevator::primaryStop),
        new InstantCommand(RobotContainer.elevator::zeroPrimaryEncoder)

        //then do these for secondary in parallel
        //add timeout?
        //put in elevator subsystem?
        

         );
    }
}