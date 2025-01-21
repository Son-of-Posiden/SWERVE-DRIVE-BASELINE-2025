// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.ExamplePathPlannerAuton;
import frc.robot.commands.SwerveTeleopCommand;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.Swerve;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Mechanism;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  private final Swerve m_swerve = new Swerve();

  // Replace with CommandPS4Controller or CommandJoystick if needed.
  private final Joystick driverStick =
      new Joystick(OperatorConstants.kDriverControllerPort);

  private final SendableChooser<Command> autonChooser;

  //System Identification stuff
  SysIdRoutine systemIDRoutine;
  Config systemIDConfig;
  Mechanism systemIDMechanism;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    //Regester all your commands, otherwise they won't be availible in PathPlanner.
    NamedCommands.registerCommand("exampleCommand", new ExampleCommand(m_exampleSubsystem));
    NamedCommands.registerCommand("examplePathPlannerAuton", new ExamplePathPlannerAuton());

    systemIDConfig = new Config();
    systemIDMechanism = new Mechanism((voltage) -> m_swerve.setAllVoltage(voltage), null, m_swerve);

    systemIDRoutine = new SysIdRoutine(systemIDConfig, systemIDMechanism);

    m_swerve.setDefaultCommand(new SwerveTeleopCommand(
          m_swerve,
          () -> driverStick.getY(),
          () -> -driverStick.getX(),
          () -> -driverStick.getTwist(),
          false,
          () -> 0.0
      )
    );

    // Configure the trigger bindings
    configureBindings();

    autonChooser = AutoBuilder.buildAutoChooser();

    SmartDashboard.putData("Auto Chooser", autonChooser);
  }

  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    new Trigger(m_exampleSubsystem::exampleCondition)
        .onTrue(new ExampleCommand(m_exampleSubsystem));

    new JoystickButton(driverStick, 1).onTrue(systemIDRoutine.quasistatic(SysIdRoutine.Direction.kForward));
    new JoystickButton(driverStick, 2).onTrue(systemIDRoutine.quasistatic(SysIdRoutine.Direction.kReverse));
    new JoystickButton(driverStick, 3).onTrue(systemIDRoutine.dynamic(SysIdRoutine.Direction.kForward));
    new JoystickButton(driverStick, 4).onTrue(systemIDRoutine.dynamic(SysIdRoutine.Direction.kReverse));

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
    //Eg. m_driverController.b().whileTrue(m_exampleSubsystem.exampleMethodCommand());
  }


  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return Autos.exampleAuto(m_exampleSubsystem);
  }
}
