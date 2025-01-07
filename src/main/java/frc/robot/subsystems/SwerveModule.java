// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.OpenLoopRampsConfigs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

public class SwerveModule {

    private final SparkMax driveMotor;
    private final SparkMax turnMotor;

    //private RelativeEncoder driveEncoder;
    private final CANcoder absoluteEncoder;

    double integratedEncoderValue;
    double integratedEncoderVelocity;

    double driveEncoderValue;
    double driveEncoderVelocity;

    double absoluteEncoderOffsetRad;
    
    public final int swervePodId;

    public Rotation2d lastAngle;

    SparkMaxConfig motorConfigs = new SparkMaxConfig();
    OpenLoopRampsConfigs openLoopConfigs = new OpenLoopRampsConfigs();
    ClosedLoopRampsConfigs closedLoopConfigs = new ClosedLoopRampsConfigs();
    CurrentLimitsConfigs currentLimitsConfigs = new CurrentLimitsConfigs();
    CANcoderConfiguration absoluteCanCoderConfig = new CANcoderConfiguration();

    private final PIDController drivePidController;
    private final PIDController turnPidController;

    public SwerveModule(
        int swervePodId,
        int driveMotorId, 
        int turnMotorId,
        int absoluteEncoderId,
        double absoluteEncoderOffsetRad
    ) {
        this.swervePodId = swervePodId;

        driveMotor = new SparkMax(driveMotorId, MotorType.kBrushless);

        turnMotor = new SparkMax(turnMotorId, MotorType.kBrushless);
        //turnMotor.getRotorPosition().refresh();
        integratedEncoderValue = turnMotor.getEncoder().getPosition();
        integratedEncoderVelocity = turnMotor.getEncoder().getVelocity();

        driveEncoderValue = driveMotor.getEncoder().getPosition();
        driveEncoderVelocity = driveMotor.getEncoder().getVelocity();

        configMotors();

        drivePidController = new PIDController(Constants.Swerve.driveKP, Constants.Swerve.driveKI, Constants.Swerve.driveKD);

        turnPidController = new PIDController(Constants.Swerve.turnKP, Constants.Swerve.turnKI, Constants.Swerve.turnKD);
        turnPidController.enableContinuousInput(-Math.PI, Math.PI);


        absoluteEncoder = new CANcoder(absoluteEncoderId, Constants.Swerve.canbusName);
        configAbsoluteEncoder();
        this.absoluteEncoderOffsetRad = absoluteEncoderOffsetRad;

        resetEncoders();
        lastAngle = getState().angle;
    }

    private void configMotors() {
        configDriveMotor();
        configTurnMotor();
    }

    private void configDriveMotor() {
        motorConfigs.inverted(Constants.Swerve.driveMotorInvert);
        motorConfigs.idleMode(Constants.Swerve.driveNeutralMode);
        motorConfigs.openLoopRampRate(Constants.Swerve.openLoopRampDutyCycle);
        motorConfigs.closedLoopRampRate(Constants.Swerve.closedLoopRampDutyCycle);
        motorConfigs.smartCurrentLimit(Constants.Swerve.driveSupplyCurrentLimit, Constants.Swerve.driveSupplyCurrentThreshold);

        driveMotor.configure(motorConfigs, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    private void configTurnMotor() {
        motorConfigs.inverted(Constants.Swerve.angleMotorInvert);
        motorConfigs.idleMode(Constants.Swerve.turnNeutralMode);
        motorConfigs.openLoopRampRate(Constants.Swerve.openLoopRampDutyCycle);
        motorConfigs.closedLoopRampRate(Constants.Swerve.closedLoopRampDutyCycle);
        motorConfigs.smartCurrentLimit(Constants.Swerve.driveSupplyCurrentLimit, Constants.Swerve.driveSupplyCurrentThreshold);

        driveMotor.configure(motorConfigs, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    private void configAbsoluteEncoder() {
        // 1 makes the value range 0, 1, 0.5 makes it -0.5 to 0.5, and so on.
        absoluteCanCoderConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 1;
        if(Constants.Swerve.canCoderInvert) {
            absoluteCanCoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
        } else {
            absoluteCanCoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
        }
    }

    public double getDrivePosition() {
        //Returns rotations, orignal returned ticks.
        return driveEncoderValue;
    }
    public double getDrivePositionMeters() {
        //Could be the wrong equation, I couldn't test it.
        return (getDrivePosition() / Constants.Swerve.driveGearRatio) * (1 / Constants.Swerve.wheelCircumference);
    }
    public double getTurnPosition() {
        return integratedEncoderValue;
    }

    public double getDriveVelocity() {
        return driveEncoderVelocity;
    }

    public double getTurnVelocity() {
        return integratedEncoderVelocity;
    }

    public double getAbsoluteEncoderRadians() {
        Rotation2d v = Rotation2d.fromDegrees(absoluteEncoder.getAbsolutePosition().getValueAsDouble());
        double radians = v.getRadians() - absoluteEncoderOffsetRad;
        return radians;
    }

    public double getAbsoluteEncoderDegrees() {
        return absoluteEncoder.getAbsolutePosition().getValueAsDouble();
    }

    //Possible error here, where the drive motor rotor doesn't get set to 0 each time. For the life of me I couldn't figure out how to get it to set to 0.
    public void resetEncoders() {
        driveMotor.getEncoder().setPosition(0);
        integratedEncoderValue = getAbsoluteEncoderRadians();
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getTurnPosition()));
    } 

    //Sets the speed for the turn motor, idk why it's called "DesiredState"
     public void setDesiredState(SwerveModuleState state) {
        state.optimize(getState().angle);

        Rotation2d angle = (Math.abs(state.speedMetersPerSecond) <= (Constants.Swerve.maxSpeed * 0.01)) ? lastAngle : state.angle;


        double percentOutput = drivePidController.calculate(getDrivePosition(), state.speedMetersPerSecond) / Constants.Swerve.maxSpeed;
        driveMotor.set(percentOutput);
        //double driveOutput = drivePidController.calculate(getDrivePosition(), percentOutput);

        double turnOutput = turnPidController.calculate(getTurnPosition(), angle.getRadians());

        turnMotor.set(turnOutput);

        lastAngle = angle;

        if (Constants.Swerve.outputPodInfo) {
            SmartDashboard.putNumber("swerve " + swervePodId + " turn output", turnOutput);
            SmartDashboard.putNumber("swerve " + swervePodId + " turn position", getTurnPosition());
            SmartDashboard.putNumber("swerve " + swervePodId + " turn setpoint", state.angle.getRadians());
            SmartDashboard.putString("swerve[" + swervePodId + "] state", state.toString());
        }
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
            (getDrivePosition() / Constants.Swerve.driveGearRatio) * (1 / Constants.Swerve.wheelCircumference),
            new Rotation2d(getTurnPosition())
        );
      }

    public void stop() {
        driveMotor.set(0);
        turnMotor.set(0);
    }
}
