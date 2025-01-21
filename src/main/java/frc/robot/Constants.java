// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.util.Units;
import frc.lib.util.COTSFalconSwerveConstants;
import frc.lib.util.SwerveModuleConstants;

import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;


/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static class Swerve {

        //It's called constents, but it's more of a config in this case.
        //Here is where you will tune a lot, and provide the code with the mechanical messurements of the robot's features.
        //Expect to see this file a lot.

        //Is it tunible with the PID tuner from Betalib? (Thank the BetaWolves for that)
        public static final boolean isTunable = false;

        //Outputs information of each swerve pod to smart dashboard. Used for debugging.
        public static final boolean outputPodInfo = false;

        //Multiplies the turn speed in TeleOp
        public static final double maxTurnMultiplier = 0.5;

        //Min and Max output values of the motors. In percentages, with 1 being 100%. Feel free to change if it's too fast.
        public static final double driveMinSpeedOutput = 0.0;
        public static final double driveMaxSpeedOutput = 1.0;

        public static final double turnMinSpeedOutput = 0.0;
        public static final double turnMaxSpeedOutput = 1.0;

        //Deadbanding essentally ignores everything under a certain threashold, so that the robot doesn't start turning if you acidentally move the joystick 1 micrometer.
        //The better your controller the lower these values will need to be. Ajust as needed.
        public static final double xDeadband = 0.1;
        public static final double yDeadband = 0.2;
        public static final double zDeadband = 0.1;
    
        //Wheel mesurements, in meters. If you use different wheels then the standard, change these.
        public static final double trackWidth = Units.inchesToMeters(23.5); 
        public static final double wheelBase = Units.inchesToMeters(23.5); 
        public static final double wheelDiameter = 0.1016; 
        public static final double wheelCircumference = wheelDiameter * Math.PI;

        //TODO: Set these values
        public static final double weight = 52.16; //Killograms
        public static final double forceToPull = 40.1; //To get this, get something like a fish scale, attach it to the robot, and pull. Value in Killograms is your number.
        public static final double distanceOfModuleToCenter = 0.38; //In meters

        //TODO: Set the positions from the center to the modules, in meters. x is meters up, y is meters left. Inverted from the standard, annoying, I know.
        public static final Translation2d frontLeftModulePos = new Translation2d(0.38, 0.38);
        public static final Translation2d frontrightModulePos = new Translation2d(0.38, -0.38);
        public static final Translation2d backLeftModulePos = new Translation2d(-0.38, 0.38);
        public static final Translation2d backRightModulePos = new Translation2d(-0.38, -0.38);

        //TODO: Change this to the name of your canbus.
        public static final String canbusName = "CANBUS NAME";

        //TODO: Set this to the ID of your GYRO
        public static final int pigeon2GyroID = 9;
    
        /* Swerve Profiling Values */
        //TODO: Set this to your max speed. I think. Used for pathplanner.
        // Meters per Second 
        public static final double maxSpeed = 2.0;

        // Radians per Second
        public static final double maxAngularVelocity = maxSpeed / Math.hypot(trackWidth / 2.0, wheelBase / 2.0);
    
        //Self explanatory, it's the max acceleration.
        public static final double maxAccelerationMetersPerSecond = 2.5; // TODO: Set this to your max acceleration. I think. Used for pathplanner.
        public static final double maxAccelerationRadiansPerSecond = Math.PI;
    
        //Gets some basic mesurements of the Swerve module based on what type it is. MK4s come in 4 types, L1, L2, L3, and L4. Just change it to whatever type you have, it'll do the rest.
        public static final COTSFalconSwerveConstants chosenModule =
          COTSFalconSwerveConstants.SDSMK4(COTSFalconSwerveConstants.driveGearRatios.SDSMK4_L4); //TODO: Change it to your type of module
    
        //TODO: Get these values from the SysId program.
        //These must be tuned to your robot. You can get the values by running the SysId program. To do so, press 1 to run the first program, 2 to run the second, 3 the third,
        //and 4 for the forth and final. From there, fallow the guide near the bottem of this https://docs.advantagescope.org/more-features/urcl/ website to get the files situated.
        //Also looking at the guides from here https://docs.wpilib.org/en/stable/docs/software/advanced-controls/system-identification/viewing-diagnostics.html and down in the WPILib
        //docs could prove rather useful at understanding what you're seeing.
        //The robot will move around a bit, so ensure it has plenty of room. Afterword, look at the values and enter them here.
        //Drive motor PID values
        public static final double driveKP = 0.05;
        public static final double driveKI = 0.0;
        public static final double driveKD = 0.0;
        public static final double driveKF = 0.0;
        public static final double drivePureKV = 0.0; //just regular KV
    
        //Turn motor PID values
        public static final double turnKP = 0.6;
        public static final double turnKI = 0.0;
        public static final double turnKD = 0.0;
        public static final double turnKF = 0.0;
        public static final double turnPureKV = 0.0; //just regular KV

        //Kp values used for path planner trajectories
        public static final double autonXKp = 5.0;
        public static final double autonYKp = 5.0;
        public static final double autonZKp = 1.0;

        //TODO: Drive Motor Characterization Values 
        public static final double driveKS = (0.13923); 
        public static final double driveKV = (2.5957);
        public static final double driveKA = (0.73919);

        //Each controls how fast it's respective property ramps up. At 0, it just sends full power instently. A non-zero value *can* make it run smoother, though most times,
        //You'll either see it do nothing, or fix absolutely everything instently.
        //Both apply to both turning and driving. Might be wrong to have it like that.
        public static final double openLoopRampDutyCycle = 0.25;
        public static final double openLoopRampTorque = 0.0;
        public static final double openLoopRampVoltage = 0.0;

        public static final double closedLoopRampDutyCycle = 0.0;
        public static final double closedLoopRampTorque = 0.0;
        public static final double closedLoopRampVoltage = 0.0;

        //Current limits
        //Applies to both turning and driving. Might be wrong to have it like that, idk.
        public static final int driveSupplyCurrentLimit = 35;
        public static final int driveSupplyCurrentThreshold = 60;
        public static final double driveSupplyTimeThreshold = 0.1;
        public static final boolean driveEnableSupplyCurrentLimit = false;

        //Drive gear ratio = drive gear ratio, pretty simple.
        public static final double driveGearRatio = chosenModule.driveGearRatio;

        //This is *NOT* where you invert the motors.
        public static final boolean driveMotorInvert = chosenModule.driveMotorInvert;
        public static final boolean angleMotorInvert = chosenModule.angleMotorInvert;
        public static final boolean canCoderInvert = chosenModule.canCoderInvert;

        //Not sure if this is right.
        public static final IdleMode driveNeutralMode = IdleMode.kBrake;
        public static final IdleMode turnNeutralMode = IdleMode.kCoast;

        //All pretty self explanatory, and you *shouldn't* have to mess with these. I think.
        public static final double driveConversionPositionFactor = (wheelDiameter * Math.PI) / driveGearRatio;
        public static final double driveConversionVelocityFactor = driveConversionPositionFactor / 60.0;
        public static final double turnConversionFactorAngToRad = 1 / chosenModule.angleGearRatio * 2 * Math.PI;

        //Universial bit of code for swerve
        public static SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
          new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
          new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
          new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
          new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0)
        );

        //Trajectory configs for Path planner, a program wich allows you to draw out paths for autonomous. 
        //If you can use it, *use it*, all else fails, just do timers, something like this. new RunCommand(ExampleCommand(ExampleSubsystem)).withTimeout(0.5);
        public static TrajectoryConfig trajectoryConfig = new TrajectoryConfig(maxSpeed, maxAccelerationMetersPerSecond).setKinematics(swerveKinematics);
        public static TrajectoryConfig trajectoryConfigSlow = new TrajectoryConfig(1.0, 1.0).setKinematics(swerveKinematics);

        //Module Config for Pathplanner. Here you tell it stuff about your modules.
        public static ModuleConfig pathPlannerModuleConfig = new ModuleConfig(
          (wheelDiameter/2), //Wheel Radious, in meters.
          maxSpeed, //Max Speed, in meters per second. Might want max Rotations Per Minute (RPM)
          (forceToPull/weight), //Coefficent of rolling friction. Might want the regular coefficent of friction.
          DCMotor.getNEO(1), //The Drive Motor Gearbox, including gear reductions. Unware if it is 1 or 4.
          driveSupplyCurrentLimit, //Drive Current Limit. Don't know if Supply is the correct type of limit.
          2
          );

        //Robot Config for Pathplanner. Here you tell it stuff about your robot.
        public static RobotConfig pathPlanneRobotConfig = new RobotConfig(
          weight,
          (weight * (distanceOfModuleToCenter*distanceOfModuleToCenter)), //The MOI, Moment of Inertia, calculated here by KG * M^2
          pathPlannerModuleConfig, //Module Config, as done above.
          frontLeftModulePos, //Front Left module, distance from center, in meters
          frontrightModulePos, //Front Right module, distance from center, in meters
          backLeftModulePos, //Back Left module, distance from center, in meters
          backRightModulePos  //Back right module, distance from center, in meters
        );

        /* Module Specific Constants */

        //TODO:
        //You will have to set the IDs, as well as get the angle offset. 
        //I believe you can get the angle offset from the characterization tools you had to use before for the KS, KV, and KA.
        /* Front Left Module - Module 0 */
        public static final class Mod0 {
          public static final int driveMotorID = 1;
          public static final int angleMotorID = 2;
          public static final int canCoderID = 21;
          public static final Rotation2d angleOffset = Rotation2d.fromDegrees(264.9);
          public static final SwerveModuleConstants constants =
              new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }
    
        /* Front Right Module - Module 1 */
        public static final class Mod1 {
          public static final int driveMotorID = 3;
          public static final int angleMotorID = 4;
          public static final int canCoderID = 22;
          public static final Rotation2d angleOffset = Rotation2d.fromDegrees(221.2);
          public static final SwerveModuleConstants constants =
            new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /* Back Left Module - Module 2 */
        public static final class Mod2 {
          public static final int driveMotorID = 5;
          public static final int angleMotorID = 6;
          public static final int canCoderID = 23;
          public static final Rotation2d angleOffset = Rotation2d.fromDegrees(99.2);
          public static final SwerveModuleConstants constants =
            new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /* Back Right Module - Module 3 */
        public static final class Mod3 {
          public static final int driveMotorID = 7;
          public static final int angleMotorID = 8;
          public static final int canCoderID = 24;
          public static final Rotation2d angleOffset = Rotation2d.fromDegrees(143.0);
          public static final SwerveModuleConstants constants =
            new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }
    }

    public static class Vision {
      public static final boolean isTunable = true;

      public static final double alignmentKp = 0.01;
      public static final double alignmentKi = 0.0;
      public static final double alignmentKd = 0.0;

      public static final double allowedError = 2.0;
    }
}
