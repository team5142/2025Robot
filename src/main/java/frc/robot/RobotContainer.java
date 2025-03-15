// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.swerve.SwerveRequest;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.PositionClass.Positions;
import frc.robot.commands.turnToAngle;
import frc.robot.commands.algaeIntake;
import frc.robot.commands.algaeThrow;

import frc.robot.commands.coralIntake;
import frc.robot.commands.moveToPosition;
import frc.robot.commands.xboxVibrate;
import frc.robot.generated.TunerConstants;
import frc.robot.commands.autoCoralIntake;
import frc.robot.subsystems.*;

public class RobotContainer {
    /* ============== Make a function to do this below */

    public static double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    public static double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors


    
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    private final SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric()
    .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    
    private final Telemetry logger = new Telemetry(MaxSpeed);
    

    

    //DEFINE CONTROLLERS
    //main drive controller
    public final static CommandXboxController joystick = new CommandXboxController(0);
    //right side of button box: port 1    
    private final CommandGenericHID rightSide = new CommandGenericHID(1);
    //left side of button box: port 2
    private final CommandGenericHID leftSide = new CommandGenericHID(2);
    
    
    public final static ElevatorSubsystem elevator = new ElevatorSubsystem();
    public final static ArmSubsystem arm = new ArmSubsystem();
    public final static IntakeSubsystem intake = new IntakeSubsystem();
    public final static SmartDashboardSubsystem smartdashboard = new SmartDashboardSubsystem();
    public final static LEDSubsystem led = new LEDSubsystem();
    public final static ClimberSubsystem climber = new ClimberSubsystem();
    private final SendableChooser<Command> autoChooser;

    public final static CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

        


    public RobotContainer() {

        registerNamedCommands();

        autoChooser = AutoBuilder.buildAutoChooser();

        SmartDashboard.putData("Auto Mode", autoChooser);

        configureBindings();

        drivetrain.setIMU180(); //we start the match facing backwards so we need to set the imu reversed

    }


    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according toc  WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX((-joystick.getLeftY() * MaxSpeed)) // Drive forward with negative Y (forward)  /2 is to slow it down
                    .withVelocityY((-joystick.getLeftX() * MaxSpeed)) // Drive left with negative X (left)
                    .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );
        
        //for testing elevator
     



        // leftSide.button(8).onTrue(
        //                  new coralIntake().withTimeout(6).unless(elevator::isElevatorActive)
        //                  .andThen(Commands.runOnce(intake::stopCoral))
        //                  .andThen(new moveToPosition(Positions.Home)));              //Intake goes until it detects a piece (and then 
                                                                                   //goes a bit more to make sure its all the way in)
                                                                                   //but if it takes more than 8 seconds it stops going
                                                                                   //it first moves the arm back all the way to intake
                                                                                   //and home after
                                                                                   //all unless elevator is up to prevent breaking
        joystick.rightBumper().onTrue
        (Commands.runOnce(intake::ejectCoral))//shoots out the coral, or just manual intake 
                          .onFalse(Commands.runOnce(intake::stopCoral));
                        //   .andThen(new moveToPosition(Positions.Home)));

        //leftSide.button 6 and 4 operater prefernce 7 will designate Left and Right intake

        rightSide.button(7).onTrue(new moveToPosition(Positions.L1));
        rightSide.button(3).onTrue(new moveToPosition(Positions.L2));
        rightSide.button(2).onTrue(new moveToPosition(Positions.L3));
        rightSide.button(1).onTrue(new moveToPosition(Positions.L4));
        rightSide.button(8).onTrue(new moveToPosition(Positions.Home));

        rightSide.button(5).onTrue(new moveToPosition(Positions.Algae1));
        rightSide.button(4).onTrue(new moveToPosition(Positions.Algae2));

        rightSide.button(9).onTrue(Commands.runOnce(climber::setClimberUp));
        rightSide.button(10).onTrue(Commands.runOnce(climber::climb));

        leftSide.button(3).onTrue(new SequentialCommandGroup( //Algae intake

        new moveToPosition(Positions.groundAlgae).unless(elevator::isElevatorActive),
        //if the elevator is active, just intake from the reef. If it isn't, we want to ground intake, so put the arm down.
        new algaeIntake().handleInterrupt(() -> {intake.stopAlgae(); intake.turnOffAlgaeLight();}).withTimeout(8)
        //intake an algae, if it doesn't work within 4 seconds stop (handle interrupt detects the timeout).

        )); 

        leftSide.button(1).onTrue(new moveToPosition(Positions.Processor));

        //leftSide.button(7).onTrue(Commands.runOnce(elevator::getLPosition));
        //leftSide.button(5).onTrue(Commands.runOnce(elevator::getFPosition));


        //intake left
        leftSide.button(6).onTrue(Commands.runOnce(() -> {led.setLeftRed(); led.setRightOff();})
                         .andThen(new coralIntake().handleInterrupt(() -> joystick.setRumble(RumbleType.kBothRumble, 0)).withTimeout(10).unless(elevator::isElevatorActive)
                         .andThen(Commands.runOnce(intake::stopCoral))
                         .andThen(new moveToPosition(Positions.Intaked))));



        //intake right
        leftSide.button(4).onTrue(Commands.runOnce(() -> {led.setRightRed(); led.setLeftOff();})
                         .andThen(new coralIntake().handleInterrupt(() -> joystick.setRumble(RumbleType.kBothRumble, 0)).withTimeout(10).unless(elevator::isElevatorActive)
                         .andThen(Commands.runOnce(intake::stopCoral))
                         .andThen(new moveToPosition(Positions.Intaked))));


       

        // leftSide.button(5).onTrue(new algaeThrow()); //throws algae with upward momentum while going to top position
        leftSide.button(2).onTrue(Commands.runOnce(intake::ejectAlgae))
        .onFalse(Commands.runOnce(intake::stopAlgae).andThen(Commands.runOnce(intake::turnOffAlgaeLight)));


        
        /* ENABLE TO TEST TURN TO DEGREES 
        joystick.x().onTrue(Commands.runOnce(() -> {
            circleDegreesRight();
            new TurnToAngle(drivetrain, Rotation2d.fromDegrees(storedAngleTurn));
        }));

        
        joystick.y().onTrue(Commands.runOnce(() -> {
            circleDegreesLeft();
            new TurnToAngle(drivetrain, Rotation2d.fromDegrees(storedAngleTurn));
        }));

        // ALSO NEED TO PUT THIS CODE WHENEVER CORAL IS FIRST INTAKED (will match the angle the robot is at, at the driver station for each side.)
        // resetStoredAngleTurn();
        
        joystick.x().onTrue(Commands.runOnce(() -> {
            resetStoredAngleTurn();
        }));
        
*/      


        joystick.x().whileTrue(new turnToAngle(120));
        joystick.y().whileTrue(new turnToAngle(180));
        joystick.b().whileTrue(new turnToAngle(240));
            
        
        
        //joystick.y().onTrue(Commands.runOnce(() -> {
        //    new turnToAngle(drivetrain, Rotation2d.fromDegrees(storedAngleTurn));
        //}));
        //joystick.y().whileTrue(drivetrain.applyRequest(() -> {
        //    new turnToAngle(drivetrain, Rotation2d.fromDegrees(storedAngleTurn));
        //}));
        //joystick.y().whileTrue(new turnToAngle(drivetrain, storedAngleTurn));

        joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
        // joystick.b().whileTrue(drivetrain.applyRequest(() ->
        //     point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))
        // )); // test this out -- it should point the wheels in the direction of the left joystick

        // RobotModeTriggers.disabled().onTrue(Commands.runOnce(() -> {
        //         arm.turnOffBrake();
        //         elevator.turnOffBrake();
        //         intake.turnOffBrake();
        //     }).ignoringDisable(true));
        

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        // plug flash drive in for these?
        //joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        //joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        //joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        //joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        //rightSide.button(9).onTrue(Commands.runOnce(SignalLogger::start));
        //rightSide.button(10).onTrue(Commands.runOnce(SignalLogger::stop));


        // reset the field-centric heading on left bumper press
        joystick.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        joystick.rightBumper().onTrue(Commands.runOnce(() -> {intake.ejectCoral(); intake.ejectAlgae();})) //run both intakes from right trigger
        .onFalse(Commands.runOnce(() -> {intake.stopCoral(); intake.stopAlgae();})); //stop both intakes from right trigger release
        
        // End of button binds

        // Turn off motor brakes when disabled, and on when enabled

        //

        // RobotModeTriggers.teleop().onTrue(Commands.runOnce(() -> {
        //         new resetElevator();
        //     }));

        // RobotModeTriggers.disabled().onTrue(Commands.runOnce(() -> {
        //         arm.turnOffBrake();
        //         elevator.turnOffBrake();
        //         intake.turnOffBrake();
        //     }).ignoringDisable(true));



        drivetrain.registerTelemetry(logger::telemeterize);
    }
/* 
    private double circleDegreesRight() {
        return storedAngleTurn += angleStep;
    }

    private double circleDegreesLeft() {
        return storedAngleTurn -= angleStep;
    }

    private void resetStoredAngleTurn(){
        Rotation2d currentHeading = drivetrain.getHeadingFromIMU();
        double remainder = currentHeading.getDegrees() % angleStep;
        if (remainder <= (angleStep/2)){
            storedAngleTurn = currentHeading.getDegrees() - remainder;
        } else {
            storedAngleTurn = currentHeading.getDegrees() + (angleStep - remainder);
        }
        storedAngleTurn = storedAngleTurn % 360;
        SmartDashboard.putNumber("currentHeading:", currentHeading.getDegrees());
        SmartDashboard.putNumber("remainder:", remainder
        );
        SmartDashboard.putNumber("storedAngleTurn:", storedAngleTurn);
        //return storedAngleTurn % 360;
    }
*/

    public void registerNamedCommands() { //registering commands for pathplanner autos

        NamedCommands.registerCommand("algaeIntake", new algaeIntake().withTimeout(8));
        NamedCommands.registerCommand("algaeThrow", new algaeThrow());
        NamedCommands.registerCommand("moveToHome", new moveToPosition(Positions.Home));
        NamedCommands.registerCommand("moveToL1", new moveToPosition(Positions.L1));
        NamedCommands.registerCommand("moveToL2", new moveToPosition(Positions.L2));
        NamedCommands.registerCommand("moveToL3", new moveToPosition(Positions.L3));
        NamedCommands.registerCommand("moveToL4", new moveToPosition(Positions.L4).andThen(new WaitCommand(1)));
        NamedCommands.registerCommand("autoCoralIntake", new autoCoralIntake());
        NamedCommands.registerCommand("coralIntake", //paste of previous coral intake command
        new coralIntake().withTimeout(6).unless(elevator::isElevatorActive)
                         .andThen(new moveToPosition(Positions.Intaked))
                         .andThen(Commands.runOnce(intake::stopCoral)));

        NamedCommands.registerCommand("coralShoot", 

        Commands.sequence(//for shooting out coral autonomously:

            Commands.runOnce(intake::ejectCoral), //run the intake to shoot it out
            new WaitCommand(1), //wait a bit (change this to make it take less long)
            Commands.runOnce(intake::stopCoral), //stop the intake
            new moveToPosition(Positions.Home) //elevator down

        ));
    }

    public Command getAutonomousCommand() {
        // return Commands.print("No autonomous command configured");
        return autoChooser.getSelected();

    }
}
