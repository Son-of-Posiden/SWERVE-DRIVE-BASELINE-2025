// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.OpenLoopRampsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

public class SwerveModule {

    private final TalonFX driveMotor;
    private final TalonFX turnMotor;

    //private RelativeEncoder driveEncoder;
    private final CANcoder absoluteEncoder;

    double integratedEncoderValue;
    double integratedEncoderVelocity;

    double absoluteEncoderOffsetRad;
    
    public final int swervePodId;

    public Rotation2d lastAngle;

    TalonFXConfiguration motorConfigs = new TalonFXConfiguration();
    OpenLoopRampsConfigs openLoopConfigs = new OpenLoopRampsConfigs();
    ClosedLoopRampsConfigs closedLoopConfigs = new ClosedLoopRampsConfigs();
    CurrentLimitsConfigs currentLimitsConfigs = new CurrentLimitsConfigs();
    CANcoderConfiguration absoluteCanCoderConfig = new CANcoderConfiguration();

    private final PIDController turnPidController;

    public SwerveModule(
        int swervePodId,
        int driveMotorId, 
        int turnMotorId,
        int absoluteEncoderId,
        double absoluteEncoderOffsetRad
    ) {
        this.swervePodId = swervePodId;

        driveMotor = new TalonFX(driveMotorId, Constants.Swerve.canbusName);

        turnMotor = new TalonFX(turnMotorId, Constants.Swerve.canbusName);
        turnMotor.getRotorPosition().refresh();
        integratedEncoderValue = turnMotor.getRotorPosition().getValueAsDouble();
        integratedEncoderVelocity = turnMotor.getRotorVelocity().getValueAsDouble();

        configMotors();


        turnPidController = new PIDController(Constants.Swerve.turnKP, Constants.Swerve.turnKI, Constants.Swerve.turnKD);
        turnPidController.enableContinuousInput(-Math.PI, Math.PI);


        absoluteEncoder = new CANcoder(absoluteEncoderId, Constants.Swerve.canbusName);
        configAbsoluteEncoder();
        this.absoluteEncoderOffsetRad = absoluteEncoderOffsetRad;

        resetEncoders();
        lastAngle = getState().angle;
    }

    private void configMotors() {
        
        openLoopConfigs.DutyCycleOpenLoopRampPeriod = Constants.Swerve.openLoopRampDutyCycle;
        openLoopConfigs.TorqueOpenLoopRampPeriod = Constants.Swerve.openLoopRampTorque;
        openLoopConfigs.VoltageOpenLoopRampPeriod = Constants.Swerve.openLoopRampVoltage;
        
        closedLoopConfigs.DutyCycleClosedLoopRampPeriod = Constants.Swerve.closedLoopRampDutyCycle;
        closedLoopConfigs.TorqueClosedLoopRampPeriod = Constants.Swerve.closedLoopRampTorque;
        closedLoopConfigs.VoltageClosedLoopRampPeriod = Constants.Swerve.closedLoopRampVoltage;
        
        currentLimitsConfigs.SupplyCurrentLimitEnable = Constants.Swerve.driveEnableSupplyCurrentLimit;
        currentLimitsConfigs.SupplyCurrentLimit = Constants.Swerve.driveSupplyCurrentLimit;
        //Unware if LowerLimit and LowerTime are the aproprate switches.
        currentLimitsConfigs.SupplyCurrentLowerLimit = Constants.Swerve.driveSupplyCurrentThreshold;
        currentLimitsConfigs.SupplyCurrentLowerTime = Constants.Swerve.driveSupplyTimeThreshold;

        configDriveMotor();
        configTurnMotor();
    }

    private void configDriveMotor() {
        motorConfigs.Slot0.kP = Constants.Swerve.driveKP;
        motorConfigs.Slot0.kI = Constants.Swerve.driveKI;
        motorConfigs.Slot0.kD = Constants.Swerve.driveKD;
        motorConfigs.Slot0.kV = Constants.Swerve.driveKF;
        
        motorConfigs.OpenLoopRamps = openLoopConfigs;
        motorConfigs.ClosedLoopRamps = closedLoopConfigs;
        motorConfigs.CurrentLimits = currentLimitsConfigs;

        /* POSSIBLY INVERTED, IN A WAY YOU CAN"T FIX IN CONSTENTS  */
        if (Constants.Swerve.driveMotorInvert) {
            motorConfigs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        } else {motorConfigs.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;}

        /* Testing needed to conferm if this is correct or not. Unlikely to be correct. */

        // motorConfigs.OpenLoopRamps.equals(openLoopConfigs);
        // motorConfigs.ClosedLoopRamps.equals(closedLoopConfigs);
        // motorConfigs.CurrentLimits.equals(currentLimitsConfigs);
        // motorConfigs.MotorOutput.Inverted.equals(Constants.Swerve.driveMotorInvert);
        
        /* Use in case of emergency */
        //driveMotor.setInverted(Constants.Swerve.driveMotorInvert);

        driveMotor.getConfigurator().apply(motorConfigs, 0.050);
        driveMotor.setNeutralMode(Constants.Swerve.driveNeutralMode);
    }

    private void configTurnMotor() {
        motorConfigs.Slot0.kP = Constants.Swerve.turnKP;
        motorConfigs.Slot0.kI = Constants.Swerve.turnKI;
        motorConfigs.Slot0.kD = Constants.Swerve.turnKD;
        motorConfigs.Slot0.kV = Constants.Swerve.turnKF;
        
        motorConfigs.OpenLoopRamps = openLoopConfigs;
        motorConfigs.ClosedLoopRamps = closedLoopConfigs;
        motorConfigs.CurrentLimits = currentLimitsConfigs;

        /* POSSIBLY INVERTED, IN A WAY YOU CAN"T FIX IN CONSTENTS  */
        if (Constants.Swerve.angleMotorInvert) {
            motorConfigs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        } else {motorConfigs.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;}

        /* Testing needed to conferm if this is correct or not. Unlikely to be correct. */

        // motorConfigs.OpenLoopRamps.equals(openLoopConfigs);
        // motorConfigs.ClosedLoopRamps.equals(closedLoopConfigs);
        // motorConfigs.CurrentLimits.equals(currentLimitsConfigs);
        // motorConfigs.MotorOutput.Inverted.equals(Constants.Swerve.angleMotorInvert);
        
        /* Use in case of emergency */
        //driveMotor.setInverted(Constants.Swerve.angleMotorInvert);

        turnMotor.getConfigurator().apply(motorConfigs, 0.050);
        driveMotor.setNeutralMode(Constants.Swerve.turnNeutralMode);
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
        return driveMotor.getRotorPosition().getValueAsDouble();
    }
    public double getDrivePositionMeters() {
        //Could be the wrong equation, I couldn't test it.
        return (getDrivePosition() / Constants.Swerve.driveGearRatio) * (1 / Constants.Swerve.wheelCircumference);
    }
    public double getTurnPosition() {
        return integratedEncoderValue;
    }

    public double getDriveVelocity() {
        return driveMotor.getRotorVelocity().getValueAsDouble();
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
        driveMotor.getRotorPosition().refresh();
        integratedEncoderValue = getAbsoluteEncoderRadians();
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getTurnPosition()));
    } 

    //Sets the speed for the turn motor, idk why it's called "DesiredState"
     public void setDesiredState(SwerveModuleState state) {
        state.optimize(getState().angle);

        Rotation2d angle = (Math.abs(state.speedMetersPerSecond) <= (Constants.Swerve.maxSpeed * 0.01)) ? lastAngle : state.angle;


        double percentOutput = state.speedMetersPerSecond / Constants.Swerve.maxSpeed;
        driveMotor.set(percentOutput);

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
