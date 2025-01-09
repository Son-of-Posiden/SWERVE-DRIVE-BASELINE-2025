# SWERVE-DRIVE-BASELINE-2025
 A basline of swerve drive code for the 2025 FRC FIRST robotics season
 Built to be used with PathPlanner

 This repository/baseline swerve drive code is designed for a 4 module, all FalconFX or Neo motor swerve drive.
Converting it into Kraken, or other CTRE motors should be fairly easy, but it comes built for FalconFX motors.
Neo motor code is in the branch labeled "NEO Drive". The "Main" branch is FalconFX.

Needed:
You will need an absolute encoder for each swerve module. Some bought modules come with them, some don’t.
You will also need a Pigeon2 gyroscope. It is likely very easy to swap it for another kind of gyroscope, and you can find it in the “Swerve” code file.
Lastly, you will need a plane/flight controller joystick that can twist. Using a regular controller with swerve is possible, and is very easy code wise, it’s just confusing to control.

Just about everything needed to be tuned should be in the constants file. I’ve done my best to consolidate it all there, and if there is any deviation, I will say it here.
Please bear in mind this is the first, beta version. There is a chance it works, but it’s a very small one.
Expect lots of bugs until I can update it, though if you want to debug it on your own, for practice or some reason or another, be my guest.
The vast majority of this code is just a heavily edited version of the code from team 6637’s year 2023 robot, updated for the 2024-2025 years.
I’ve attempted to make this as user friendly as possible so that even non-coders can understand it enough to use it in a pinch.

How to use:

Before anything, you’ll need to do some setup,
Firstly, have a robot. Simple, right?
Secondly, you’ll have to go through the constants file and input several measurements from your robot. I’ll try to have most of them in the same place. It’ll be basic stuff like weight, module distance from the center, that kind of thing.
Thirdly, at the bottom of constants, you’ll need to put in the Port IDs of all your motors, found in each of the modules. While you're at it, set anything that’s underlined in blue and marked with “TODO:” if you can, such as Canbusname, and gyro ID. Don’t bother with stuff you have to test like “max speed” or “max acceleration” unless stuff doesn’t work, or don’t bother with it at all if the preset values work fine.
Fourthly, use characterisation tools, like the Sysid tool in WPILIB, or https://github.com/wpilibsuite/frc-characterization, inorder to get the KS, KV, and KA values, which you will need to set in the constants file.
Lastly, you’ll have to tune the Drive and Turn PID controllers.
To do that, essentially just plug in random values until it runs smoothly. Looking up some videos on how to tune PID would probably be helpful.


Now with everything set up, you can begin. Examples of how to run a path made in PathPlanner are included in the “RobotContainer” and “ExamplePathPlannerAuton” files, just copy paste and input your path’s name. Though, I would also recommend watching some videos on how to use PathPlanner.

Possible bug notes:
Betalib PID config may not work.

getDrivePosition in swerveModules returns rotations now, not ticks.

Idk if (getDrivePosition() / Constants.Swerve.driveGearRatio) * (1 / Constants.Swerve.wheelCircumference) Is the right equation to convert to meters.


Drive motor rotor position doesn’t get reset to 0 each time, I couldn’t find out how to reset it.
