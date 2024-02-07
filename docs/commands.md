# 2024 Subsystems/Commands
This page is for design purposes only and excludes drivetrain translation commands.\
It is meant to be a reference when laying out auto modes in PathPlanner. It is also used document all named commands used by PathPlanner autos to ensure they are all registered to actual commands.
## Commands
**RunShooter()**\
*requires: ShooterWheels*\
Sets the shooter speed to the shot speed, ends immidiately

**RunShooterSlow()**\
*requires: ShooterWheels*\
Sets the shooter speed to the amp scoring speed, ends immidiately

**StopShooter()**\
*requires: ShooterWheels*\
Sets the shooter speed to zero, ends immidiately

**SetShooterDistance(distanceInches)**\
*requires: ShooterPose*\
Set the shooter height and rotation for the ideal launch angle when the robot is at a ground distance of `distanceInches`\
Ends when the shooter is at the target pose

**SetShooterPose(ShooterPoseEnum)**\
*requires: ShooterPose*\
Set the shooter height and rotation based on the input pose enum (handoff, amp, source, trap, subwoofer)\
Ends when the shooter is at the target pose

**FeedShooter()**\
*requires: Feeder*\
Feed the Shooter, ends shortly after it is fed

**FeedShooterManual()**\
*requires: Feeder*\
Feed the Shooter, never ends

**FeedShooterSmart()**\
*requires: Feeder*\
start running feeder rollers when:
- shooter is not stopped\
AND
- shooterPose is Amp OR shooterPose is Speaker and Limelight is aligned\
ends when feeder.hasNote() is false

**IntakeNote()**\
*requires: Intake, Feeder, ShooterPose*\
Run the intake and feeder in, ensure the shooter is oriented for a handoff. Stop everything and end when piece is staged.

**FinishIntakingNote()**\
*requires: Intake, Feeder, ShooterPose*\
If piecePosessed flag is true, run intake and feeder. Stop everything and end when piece is staged.

**IntakeSourceNote()**\
*requires: Feeder, ShooterPose*\
Run the feeder in, ensure the shooter is oriented for a source load. Stop everything and end when piece is staged.

**Eject()**\
*requires: Intake, Feeder, ShooterPose*\
Run everything backwards, never ends.

**WaitForPieceUnstaged()**\
*requires:*\
Does nothing, ends when feeder.hasNote() is false

## Subsystems
**Drivetrain**\
Controls all eight motors used by the swerve modules

**Intake**\
Controls the motors powering the intake rollers and wheels

**Feeder**\
Controls the motors powering the feeder rollers

**ShooterWheels**\
Controls the two motors powering the shooter wheels

**ShooterPose**\
Controls the motors powering the elevator and shooter rotation

**Climber**\
Controls the two motors powering the climbers

## Existing PathPlanner Named Commands
**RunShooter**\
Spin-up the shooter so it can be at speed when we need it, ends immidiately

**SetShooterDistance115**\
Set the shooter pose for a shot distance of 115"

**FeedShooter**\
Feed the Shooter, end after it is fed

**IntakeNote**\
Intake a note from the ground, end when note is in feeder\

**SetShooterDistance125**\
Set the shooter pose for a shot distance of 125"

**SetShooterDistance150**\
Set the shooter pose for a shot distance of 150"
