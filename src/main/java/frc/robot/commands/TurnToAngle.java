package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import com.ctre.phoenix6.swerve.SwerveRequest;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.RobotContainer;

public class turnToAngle extends Command {
    private CommandSwerveDrivetrain drivetrain;
    private PIDController pidController;
    private Rotation2d targetAngle;
    private final CommandXboxController controller;
    private SwerveRequest.FieldCentric driveRequest = new SwerveRequest.FieldCentric();

    public turnToAngle(CommandSwerveDrivetrain drivetrain, Rotation2d tAngle, CommandXboxController controller) {
        this.drivetrain = drivetrain;
        this.targetAngle = tAngle;
        this.controller = controller;
        SmartDashboard.putNumber("tangle", tAngle.getDegrees());
        // Initialize PID controller with gains (kP, kI, kD)
        this.pidController = new PIDController(0.1, 0.0, 0.0); // Tune these values
        pidController.setTolerance(1.0); // Tolerance in degrees

        // Require the drivetrain subsystem
        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        // Reset the PID controller to ensure it starts fresh
        pidController.reset();
    }

    private double getShortestAngleDifference(double current, double target) {
        double difference = target - current;
        while (difference > 180) {
            difference -= 360;
        }
        while (difference < -180){
            difference +=360;
        }
        return difference;
    }

    @Override
    public void execute() {
        double xSpeed = -controller.getLeftY();
        double ySpeed = -controller.getLeftX();

        // Get the current heading from the drivetrain
        Rotation2d currentHeading = drivetrain.getHeading();

        double currentDegrees = currentHeading.getDegrees();
        double targetDegrees = targetAngle.getDegrees();
        double error = getShortestAngleDifference(currentDegrees, targetDegrees);

        // Calculate the rotation output using the PID controller
        double rotationOutput = pidController.calculate(0, error);

        // Apply the rotation output to the drivetrain using a SwerveRequest
        drivetrain.setControl(driveRequest
            .withVelocityX((xSpeed * RobotContainer.MaxSpeed)*.5)
            .withVelocityY((ySpeed * RobotContainer.MaxSpeed)*.5)
            .withRotationalRate(rotationOutput));

        // Optional: Log the values to SmartDashboard for debugging
        SmartDashboard.putNumber("Current Heading", currentHeading.getDegrees());
        SmartDashboard.putNumber("Target Angle", targetAngle.getDegrees());
        SmartDashboard.putNumber("Rotation Output", rotationOutput);
        SmartDashboard.putNumber("ANGLE ERROR", error);
    }

    @Override
    public boolean isFinished() {
        // End the command when the PID controller reaches the setpoint
        return false; 
        //return pidController.atSetpoint();
    }

    @Override
    public void end(boolean interrupted) {
        // Stop the drivetrain when the command ends
        drivetrain.setControl(new SwerveRequest.Idle());
    }
}