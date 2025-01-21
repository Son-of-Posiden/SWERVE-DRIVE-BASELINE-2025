// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.commands.SwerveTeleopCommand;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class VisionSubsystem extends SubsystemBase {

  NetworkTable table;
  NetworkTableEntry tx, ty, ta;
  double x, y, area;

  PIDController pidController;

  public VisionSubsystem() {
    table = NetworkTableInstance.getDefault().getTable("limelight");
    tx = table.getEntry("tx");
    ty = table.getEntry("ty");
    ta = table.getEntry("ta");

    pidController = new PIDController(Constants.Vision.alignmentKp, Constants.Vision.alignmentKi, Constants.Vision.alignmentKd);
  }

  @Override
  public void periodic() {
    x = tx.getDouble(0.0);
    y = ty.getDouble(0.0);
    area = ta.getDouble(0.0);

    if (Constants.Vision.isTunable) {
      SmartDashboard.putNumber("LimelightX", x);
      SmartDashboard.putNumber("LimelightY", y);
      SmartDashboard.putNumber("LimelightArea", area);
    }
  }

  public void align(Swerve swerve, DoubleSupplier x, DoubleSupplier y, DoubleSupplier z) {
    new SwerveTeleopCommand(swerve, x, y, z, false, () -> pidController.calculate(this.y));
  }
}
