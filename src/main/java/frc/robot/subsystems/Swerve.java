// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


public class Swerve extends SubsystemBase {

  private final SwerveModule frontLeft = new SwerveModule(
    0,
    Constants.Swerve.Mod0.driveMotorID,
    Constants.Swerve.Mod0.angleMotorID,
    Constants.Swerve.Mod0.canCoderID,
    Constants.Swerve.Mod0.angleOffset.getRadians()
  );

  private final SwerveModule frontRight = new SwerveModule(
    1,
    Constants.Swerve.Mod1.driveMotorID,
    Constants.Swerve.Mod1.angleMotorID,
    Constants.Swerve.Mod1.canCoderID,
    Constants.Swerve.Mod1.angleOffset.getRadians()
  );

  private final SwerveModule backLeft = new SwerveModule(
    2,
    Constants.Swerve.Mod2.driveMotorID,
    Constants.Swerve.Mod2.angleMotorID,
    Constants.Swerve.Mod2.canCoderID,
    Constants.Swerve.Mod2.angleOffset.getRadians()
  );

  private final SwerveModule backRight = new SwerveModule(
    3,
    Constants.Swerve.Mod3.driveMotorID,
    Constants.Swerve.Mod3.angleMotorID,
    Constants.Swerve.Mod3.canCoderID,
    Constants.Swerve.Mod3.angleOffset.getRadians()
  );

  // GYRO
  private final ADXRS450_Gyro gyro;
  //public double initialRoll;
  private Field2d field;
  
  // ODOMETER
  private SwerveDriveOdometry odometry;

  public Swerve() {
    gyro = new ADXRS450_Gyro();
    zeroGyroscope();
    //initialRoll = gyro.getAngle();

    odometry = new SwerveDriveOdometry(Constants.Swerve.swerveKinematics, getGyroscopeRotation(), getPositions());

    field = new Field2d();
    SmartDashboard.putData("Field", field);

    AutoBuilder.configure(
      this::getPose,
      this::resetOdometry,
      this::getChassisSpeeds,
      this::setChassisSpeeds,
      new PPHolonomicDriveController(
        new PIDConstants(Constants.Swerve.driveKP, Constants.Swerve.driveKI, Constants.Swerve.driveKD), // Translation PID constants
        new PIDConstants(Constants.Swerve.turnKP, Constants.Swerve.turnKI, Constants.Swerve.turnKD) // Rotation PID constants)
        ), 
      Constants.Swerve.pathPlanneRobotConfig,
      () -> {
        // Boolean supplier that controls when the path will be mirrored for the red alliance
        // This will flip the path being followed to the red side of the field.
        // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
          return alliance.get() == DriverStation.Alliance.Red;
        }
        return false;
      },
      this // Reference to this subsystem to set requirements
    );
  }

  public void zeroGyroscope() {
      gyro.reset();
  }

  public Rotation2d getGyroscopeRotation() {
      return Rotation2d.fromDegrees(gyro.getAngle());
  }

  public double getHeading() {
    return Math.IEEEremainder(getGyroscopeRotation().getDegrees(), 360);
  }

  public Rotation2d getHeadingRotation2d() {
    return Rotation2d.fromDegrees(getHeading());
  }

  public void stopModules() {
    frontLeft.stop();
    frontRight.stop();
    backLeft.stop();
    backRight.stop();
  }

  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.maxSpeed);
    frontLeft.setDesiredState(desiredStates[0]);
    frontRight.setDesiredState(desiredStates[1]);
    backLeft.setDesiredState(desiredStates[2]);
    backRight.setDesiredState(desiredStates[3]);
  }

  public void setChassisSpeeds(ChassisSpeeds chassisSpeeds) {
    SwerveModuleState[] swerveModuleStates = Constants.Swerve.swerveKinematics.toSwerveModuleStates(chassisSpeeds);
    setModuleStates(swerveModuleStates);
  }

  public ChassisSpeeds getChassisSpeeds() {
    return Constants.Swerve.swerveKinematics.toChassisSpeeds(frontLeft.getState(), frontRight.getState(), backLeft.getState(), backRight.getState());
  }

  public SwerveModulePosition[] getPositions() {
    return new SwerveModulePosition[]{ 
      frontLeft.getPosition(), frontRight.getPosition(), backLeft.getPosition(), backRight.getPosition()
    };
  }

  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  public void resetOdometry(Pose2d pose) {
    odometry.resetPosition(getGyroscopeRotation(), getPositions(), pose);
  }

  /* 
  public double getRoll() {
    return gyro.getRoll().getValueAsDouble();
  }

  public double getYaw() {
    return gyro.getYaw().getValueAsDouble();
  }
  */

  public void setAllVoltage(Voltage voltage) {
    frontLeft.getDriveMotor().setVoltage(voltage);
    frontLeft.getTurnMotor().setVoltage(voltage);
    frontRight.getDriveMotor().setVoltage(voltage);
    frontRight.getTurnMotor().setVoltage(voltage);
    backLeft.getDriveMotor().setVoltage(voltage);
    backLeft.getTurnMotor().setVoltage(voltage);
    backRight.getDriveMotor().setVoltage(voltage);
    backRight.getTurnMotor().setVoltage(voltage);
  }
  

  @Override
  public void periodic() {
    SmartDashboard.putNumber("swerve heading", getHeading());
    SmartDashboard.putNumber("swerve pose heading", getPose().getRotation().getDegrees());


    if(Constants.Swerve.isTunable) {
      SmartDashboard.putNumber("fl abs angle", frontLeft.getAbsoluteEncoderDegrees());
      SmartDashboard.putNumber("fr abs angle", frontRight.getAbsoluteEncoderDegrees());
      SmartDashboard.putNumber("fr adjusted angle", frontRight.getAbsoluteEncoderRadians());
      SmartDashboard.putNumber("bl abs angle", backLeft.getAbsoluteEncoderDegrees());
      SmartDashboard.putNumber("bl adjusted angle", backLeft.getAbsoluteEncoderRadians());
      SmartDashboard.putNumber("br abs angle", backRight.getAbsoluteEncoderDegrees());

      SmartDashboard.putNumber("drive distance in meters", frontLeft.getDrivePositionMeters());
    }

    odometry.update(getGyroscopeRotation(), getPositions());
    field.setRobotPose(getPose());
  }

}