# 2024 Subsystems/Commands
This page is for design purposes only and excludes drivetrain translation commands.\
It is meant to be a reference when laying out auto modes in PathPlanner. It is also used document all named commands used by PathPlanner autos to ensure they are all registered to actual commands.
## Commands
**SpinShooter()**\
*requires: ShooterWheels*\
Sets the shooter speed to the shot speed, never ends\
\
**SetShooterDistance(distanceInches)**\
*requires: ShooterOrientation*\
Set the shooter height and rotation for the ideal launch angle when the robot is at a ground distance of `distanceInches`\
Ends when the shooter is at the target orientation\
\
**ShootNote()**\
*requires: Intake, ShooterWheels*\
Feed the Shooter, ends shortly after it is fed\
\
**IntakeNote()**\
*requires: Intake, Feeder, ShooterOrientation*\
Run the intake and feeder in, ensure the shooter is oriented for a handoff, when the Feeder detects a game piece, stop everything and end the command.

## Subsystems
**Drivetrain**\
Controls all eight motors used by the swerve modules\
\
**Intake**\
Controls the motors powering the intake rollers and wheels\
\
**Feeder**\
Controls the motors powering the feeder rollers\
\
**ShooterWheels**\
Controls the two motors powering the shooter wheels\
\
**ShooterOrientation**\
Controls the motors powering the elevator and shooter rotation\
\
**Climber**\
Controls the two motors powering the climbers

## Existing PathPlanner Named Commands
**SpinShooter**\
Spin-up the shooter so it can be at speed when we need it, run continuously\
\
**SetShooterDistance115**\
Set the shooter orientation for a shot distance of 115"\
\
**ShootNote**\
Feed the Shooter, end after it is fed\
\
**IntakeNote**\
Intake a note from the ground, end when note is in feeder\
\
**SetShooterDistance125**\
Set the shooter orientation for a shot distance of 125"\
\
**SetShooterDistance150**\
Set the shooter orientation for a shot distance of 150"
