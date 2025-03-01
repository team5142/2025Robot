package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.RobotContainer;
import frc.robot.Constants.PositionClass.Positions;
public class algaeThrow extends SequentialCommandGroup{

    double stageDelay = 1.5;
    double throwDelay = 0.1; //amount of time to wait before throwing algae


    public algaeThrow(){
        
        addRequirements(RobotContainer.intake);

    addCommands(

            //NOTE: The second stage hits the top first. In order to get the most launch,
            //we need to stall the first stage
        new moveToPosition(Positions.BargePrep),
        new WaitCommand(stageDelay),
        new moveToPosition(Positions.L4), //now still go to top 
        // new WaitCommand(throwDelay), //wait until we are at the top to eject

        new InstantCommand(RobotContainer.intake::ejectAlgae),
        new WaitCommand(0.75),
        new InstantCommand(RobotContainer.intake::stopAlgae) //throw the algae with upwards momentum
            

         );
    }
}