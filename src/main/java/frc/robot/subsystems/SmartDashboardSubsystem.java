package frc.robot.subsystems;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DebugTelemetrySubsystems;
import frc.robot.Robot;
import frc.robot.RobotContainer;

public class SmartDashboardSubsystem extends SubsystemBase {

    public SmartDashboardSubsystem() {}

    public void refreshOdometryDisplay() {
        
    }
    public void refreshPoseDisplay(){
        
    }
    public void refreshIMUDisplay() {
        
    }
    public void refreshElevatorPositioningDisplay(){

    }
    public void refreshArmRotationDisplay(){
        
    }
    public void refreshIntakeDisplay(){
        
    }
    public void refreshClimberDisplay(){
        
    }

    @Override
    public void periodic() {
    // This method will be called once per scheduler run
        refreshOdometryDisplay();
        refreshPoseDisplay();
        refreshIMUDisplay();
        refreshElevatorPositioningDisplay();
        refreshArmRotationDisplay();
        refreshIntakeDisplay();
        refreshClimberDisplay();
    }
}
