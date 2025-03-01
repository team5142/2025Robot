package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.RobotContainer;

public class algaeIntake extends SequentialCommandGroup{

    double delay = 0.1; //amount of time to continue intaking after algae is detected

    public algaeIntake(){
        
        addRequirements(RobotContainer.intake);

    addCommands(

        new InstantCommand(RobotContainer.intake::intakeAlgae),
        new WaitUntilCommand(RobotContainer.intake::isAlgaeIntaked),
        new WaitCommand(delay),
        new InstantCommand(RobotContainer.intake::holdAlgae)
        // new xboxVibrate()

         );
    }
}