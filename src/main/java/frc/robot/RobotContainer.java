// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.PositionClass.Positions;
import frc.robot.commands.algaeIntake;
import frc.robot.commands.algaeThrow;

import frc.robot.commands.coralIntake;
import frc.robot.commands.moveToPosition;
import frc.robot.generated.TunerConstants;
import frc.robot.commands.coralIntake;
import frc.robot.subsystems.*;

public class RobotContainer {
    /* ============== Make a function to do this below */

    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.05).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors


    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    private final SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric()
    .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    private final Telemetry logger = new Telemetry(MaxSpeed);
    

    

    //DEFINE CONTROLLERS
    //main drive controller
    public final static CommandXboxController joystick = new CommandXboxController(0);
    //right side of button box: port 1    
    private final CommandGenericHID rightSide = new CommandGenericHID(1);
    //left side of button box: port 2
    private final CommandGenericHID leftSide = new CommandGenericHID(2);
    
    
    public static final ElevatorSubsystem elevator = new ElevatorSubsystem();

    public final static ArmSubsystem arm = new ArmSubsystem();
    public final static IntakeSubsystem intake = new IntakeSubsystem();

    private final SendableChooser<Command> autoChooser;


    public RobotContainer() {

        autoChooser = AutoBuilder.buildAutoChooser();

        SmartDashboard.putData("Auto Mode", autoChooser);

        configureBindings();
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according toc  WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX((-joystick.getLeftY() * MaxSpeed)/2) // Drive forward with negative Y (forward)  /2 is to slow it down
                    .withVelocityY((-joystick.getLeftX() * MaxSpeed)/2) // Drive left with negative X (left)
                    .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );
        
        //for testing elevator
     



        rightSide.button(4).onTrue(
                         new coralIntake().withTimeout(6).unless(() -> elevator.isElevatorActive())
                         .andThen(new moveToPosition(Positions.Home))
                         .andThen(Commands.runOnce(intake::stopCoral)));              //Intake goes until it detects a piece (and then 
                                                                                   //goes a bit more to make sure its all the way in)
                                                                                   //but if it takes more than 8 seconds it stops going
                                                                                   //it first moves the arm back all the way to intake
                                                                                   //and home after
                                                                                   //all unless elevator is up to prevent breaking
        
        rightSide.button(5).onTrue(Commands.runOnce(intake::intakeCoral)) //shoots out the coral, or just manual intake 
                          .onFalse(Commands.runOnce(intake::stopCoral));    


        rightSide.button(2).onTrue(new moveToPosition(Positions.L1));
        rightSide.button(8).onTrue(new moveToPosition(Positions.L2));
        rightSide.button(1).onTrue(new moveToPosition(Positions.L4));
        rightSide.button(3).onTrue(new moveToPosition(Positions.Home));

        rightSide.button(7).onTrue(new algaeIntake().withTimeout(8)); //intakes algae until it detects a piece (cancels after 8 secs)
        
        rightSide.button(6).onTrue(new algaeThrow()); //throws algae with upward momentum while going to top position

        


      

        

        //RIGHT SIDE BINDINGS

        //Intake Coral - 7
        //Intake Algae - 2
        //Output Algae - 3

        //Elevator to L1 - 6
        //Elevator to L2 - 4
        //Elevator to L3 - 1
        //Elevator to L4 - 5
        
        //Climb - 9

        //Left Side Bindings
        //Climbing mode - Switch 9
        //Arm to Home Position - 11
        //Scoring in Processor Position - 2
        //Arm to Barge Position - 3


        joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
        joystick.b().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))
        )); // test this out -- it should point the wheels in the direction of the left joystick


        

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        // plug flash drive in for these?
        joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));
        

        // reset the field-centric heading on left bumper press
        joystick.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));


        // End of button binds

        // Turn off motor brakes when disabled, and on when enabled


        RobotModeTriggers.teleop().onTrue(Commands.runOnce(() -> {
                arm.turnOnBrake();
                elevator.turnOnBrake();
                intake.turnOnBrake();
            }));

        RobotModeTriggers.disabled().onTrue(Commands.runOnce(() -> {
                arm.turnOffBrake();
                elevator.turnOffBrake();
                intake.turnOffBrake();
            }).ignoringDisable(true));



        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        // return Commands.print("No autonomous command configured");
        return autoChooser.getSelected();

    }
}
