// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import java.util.Map;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.ClosedLoopOutputType;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.SteerFeedbackType;


import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.*;

import com.ctre.phoenix6.swerve.SwerveModuleConstantsFactory;
import com.ctre.phoenix6.swerve.SwerveRequest;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

	public static class OperatorConstants {
		public static final int kDriverControllerPort = 0;
	}


	public static final class PositionClass {
		// Positions in order: Primary Elevator, Secondary Elevator, Arm Position (0 -> 1)
		public enum Positions {
			Home("Home",
							12,
							0,
							1.7),
			Feed("Feed",
							0,
							0,
							0),
			Intaked("Intaked",
							0,
							0,
							1.7),
			L1("L1",
							16,
							0,
							14),
			L2("L2",
							35,
							0,
							1.7),
			L3("L3",
							49.5,
							24,
							1.7),
			L4("L4",
							68,
							69,
							5), //4 
			Algae1("Algae1",
							44, //24 //44
							0,
							13.5), //7.5 //13.5
			AlgaeE1("AlgaeE1",
							3,
							0,
							6), //4.5
			AlgaeE1F("AlgaeE1F",
							3,
							0,
							1.7), //4.5
			Algae2("Algae2",
							55, //39 //55 
							24,
							13.5), //7.5 //13.5
			AlgaeE2("AlgaeE2",
							39, 
							0,
							6), //4.5 //4.5
			AlgaeE2F("AlgaeE2F",
							39,
							0,
							1.7), //4.5
			PostAlgae1("PostAlgae1",
							23,
							0,
							1.7),
			PostAlgae2("PostAlgae2",
							44,
							24,
							1.7),
			Processor("Processor",
							0,
							0,
							11.5),
			BargePrep("BargePrep",
							62,
							63,
							1.7),
			Barge("Barge",
							68,
							69,
							1.7),
			groundAlgae("groundAlgae",
							0,
							0,
							20.5),
			groundCoral("groundCoral",
							0,
							0,
							32);

			public final String label;
			public final double primaryElevator;
			public final double secondaryElevator;
			public final double armPosition;

			private Positions(String label,
							double primaryElevator,
							double secondaryElevator,
							double armPosition) {
					this.label = label;
					this.primaryElevator = primaryElevator;
					this.secondaryElevator = secondaryElevator;
					this.armPosition = armPosition;
			}

		}
    }

	public static final class PoseClass {

		public enum Poses {

			AB("AB",
				new Pose2d(3.26, 4.025, Rotation2d.fromDegrees(0)), //pose that is lined up with the AB reef
				7, //use FlipFieldPose if on red Side in command ***
				18),											//tag in front of AB reef

			CD("CD",
				new Pose2d(3.91, 3.02, Rotation2d.fromDegrees(60)), //pose that is lined up with the CD reef
				8,
				17),													 //tag in front of CD reef


			EF("EF",
				new Pose2d(5.09, 2.990, Rotation2d.fromDegrees(120)),
				9,
				22),


			GH("GH",
				new Pose2d(5.71, 4.025, Rotation2d.fromDegrees(180)),
				10,
				21),
			

			IJ("IJ",
				new Pose2d(5.09, 5.07, Rotation2d.fromDegrees(-120)), //change this, too far left (rel to tag)
				11,
				7),


			KL("KL",
				new Pose2d(3.91, 5.03, Rotation2d.fromDegrees(-60)),
				6,
				20),


			rightSideRightCoralStation("rightSideRightCoralStation",
				new Pose2d(1.605, 0.69, Rotation2d.fromDegrees(54)), //1.605, 0.69
				2,
				12),
				
			leftSideRightCoralStation("leftSideRightCoralStation",
				new Pose2d(1.522, 0.744, Rotation2d.fromDegrees(54)), //1.522, 0.744
				2,
				12),
				

			leftSideLeftCoralStation("leftSideLeftCoralStation",
				new Pose2d(1.605, 7.35, Rotation2d.fromDegrees(127)), //1.605, 7.35
				1,
				13),
				
			rightSideLeftCoralStation("rightSideLeftCoralStation",
				new Pose2d(1.522, 7.3, Rotation2d.fromDegrees(127)), //1.522, 7.3
				1,
				13);;




			public final String label;
			public final Pose2d desiredPose;
			public final int redAprilTagID;
			public final int blueAprilTagID;


			private Poses(String label, Pose2d desiredPose, int redAprilTagID, int blueAprilTagID) {
					this.label = label;
					this.desiredPose = desiredPose;
					this.redAprilTagID = redAprilTagID;
					this.blueAprilTagID = blueAprilTagID;
				
			}

		}


	}


	public static final class CurrentLimits {

		public static final int Neo550 = 20;
		public static final int Neo500 = 40;
	}

	public static final class FeedForwards {

		public static final double Neo500FF = (1/473);
		public static final double Neo550FF = (1/917);

	}
	






	public static final class VisionConstants {

		

		public static final class LimeLightConstants {

			// If changing this value, do not forget to set it in LL
			public static final String LLAprilTagName = "limelight-at"; // Limelight that will track Apriltags; may
																		// decide to use multiple ones

			// centerpose from BLUE coordinate system
			public static final Pose2d centerFieldPose = new Pose2d(8.308467, 4.098925, new Rotation2d(0));
			// NEW origin from the old origin point of view in the old coordiinate system
			public static final Pose2d originFieldPose = new Pose2d(-8.308467, -4.098925, new Rotation2d(0));

			// *** LL Detector ***
			public static final String LLDetectorName = "limelight-d"; // Limelight that will track Apriltags; may
																		// decide to use multiple ones

			// TODO: measure and verify this transform
			public static final Transform2d cameraToRobotTransform = new Transform2d(new Translation2d(-0.30, -0.23),
					Rotation2d.fromDegrees(180));

		}

	}

}
