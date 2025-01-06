// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.io.IOException;

import org.json.simple.parser.ParseException;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FileVersionException;

import edu.wpi.first.wpilibj2.command.Command;

public class ExamplePathPlannerAuton extends Command {
  /** Creates a new ExamplePathPlannerAuton. */
  public ExamplePathPlannerAuton() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  //Unsure if the "throws" for the exceptions are needed, but they got rid of an error.
  public Command Run() throws FileVersionException, IOException, ParseException {
    // Load the path you want to follow using its name in the GUI
    PathPlannerPath path = PathPlannerPath.fromPathFile("ExamplePath");

    //Ideally, you'd use a sequential command group, but if you're just doing one action then a command is fine.

    /* 

    Alternatively, you can load an entire auton instead of a single path.

    // Use the PathPlannerAuto class to get a path group from an auto
    List<PathPlannerPath> pathGroup = PathPlannerAuto.getPathGroupFromAutoFile("Example Auto");

    // You can also get the starting pose from the auto. Only call this if the auto actually has a starting pose.
    Pose2d startingPose = PathPlannerAuto.getStaringPoseFromAutoFile("Example Auto");

    //Inorder for this to work, you'll have to loop through each path in the list of paths, moving to the next one once the current is done.

    */

    // Create a path following command using AutoBuilder. This will also trigger event markers.
    return AutoBuilder.followPath(path);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
