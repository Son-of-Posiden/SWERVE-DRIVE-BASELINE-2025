// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

public class SwerveTeleopCommand extends Command{
  
  private final Swerve swerve;
  private DoubleSupplier x, y, z, visionOffset; //driveInversionMultiplier;
  private double xOutput, yOutput, zOutput;
  private boolean isAuton;

  //This is the command that will be constently running during the driver operated period. Here, joystick inputs are turned into power outputs for motors.
  //The "isAuton" value is for autonomous alignment using a camera senser. So you don't miss.
  public SwerveTeleopCommand(Swerve swerve, DoubleSupplier x, DoubleSupplier y, DoubleSupplier z, boolean isAuton, DoubleSupplier visionOffset) {
    this.swerve = swerve;
    this.x = x;
    this.y = y;
    this.z = z;
    this.isAuton = isAuton;
    this.visionOffset = visionOffset;

    addRequirements(swerve);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    if(Constants.Swerve.isTunable) {
      SmartDashboard.putNumber("joystick raw x", this.x.getAsDouble());
      SmartDashboard.putNumber("joystick raw y", this.y.getAsDouble());
      SmartDashboard.putNumber("joystick raw z", this.z.getAsDouble());
      SmartDashboard.putNumber("vision offset", this.visionOffset.getAsDouble());
    }

    // adjust for joystick drift
    xOutput = modifyAxis(this.x.getAsDouble(), Constants.Swerve.xDeadband); //* driveInversionMultiplier.getAsDouble();
    yOutput = modifyAxis(this.y.getAsDouble(), Constants.Swerve.yDeadband); //* driveInversionMultiplier.getAsDouble();

    zOutput = modifyAxis(this.z.getAsDouble(), Constants.Swerve.zDeadband);

    if (isAuton) {
      zOutput = zOutput + visionOffset.getAsDouble(); //Change to yOutput if you wish to align side-to-side rather then to face.
    }

    if (Math.abs(yOutput) > 1) {
      yOutput = 1 * Math.signum(yOutput);
    }
    zOutput *= Constants.Swerve.maxTurnMultiplier;

    xOutput *= Constants.Swerve.maxSpeed;
    yOutput *= Constants.Swerve.maxSpeed;
    zOutput *= Constants.Swerve.maxAngularVelocity;

    ChassisSpeeds chassisSpeeds = new ChassisSpeeds(xOutput, yOutput, zOutput);

    SwerveModuleState[] moduleStates = Constants.Swerve.swerveKinematics.toSwerveModuleStates(chassisSpeeds);

    swerve.setModuleStates(moduleStates);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    swerve.stopModules();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isAuton ? visionOffset.getAsDouble() < Constants.Vision.allowedError : false;
  }

  private static double modifyAxis(double value, double deadband) {
      // Deadband - ignore really low numbers
      value = MathUtil.applyDeadband(value, deadband);

      // Square the axis for finer lower # control (.9 * .9 = .81, .2 * .2 = .4)
      value = Math.copySign(value * value, value);

      return value;
  }
}

