package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import frc.robot.Constants.PositionClass.Positions;
import frc.robot.RobotContainer;
import frc.robot.subsystems.IntakeSubsystem;
public class coralIntake extends SequentialCommandGroup{

    double delay = 0.325; //amount of time to continue intaking after coral is detected

    public coralIntake(){

        addRequirements(RobotContainer.intake);

    addCommands(


        new moveToPosition(Positions.Feed),
        new InstantCommand(RobotContainer.intake::intakeCoral),
        new WaitUntilCommand(RobotContainer.intake::isCoralIntaked),
        new WaitCommand(delay),
        new InstantCommand(RobotContainer.intake::stopCoral),
        new xboxVibrate()


        
    );
    }
}