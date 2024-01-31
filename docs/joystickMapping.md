# 2024 Joystick Mapping
This page is meant to be a reference for what commands/funciton each joystick button should fulfill.
## Joysticks

**Left Joystick:**\
*Trigger (speaker aim)*\
deadline(waitForPieceUnstaged, setShooterDistance(vision),  RotateToAngle(vision, joystick)\
*onRelease:*\
setShooterPose(handoff)

*2 (amp scoring)*\
deadline(waitForPieceUnstaged, setShooterPose(amp).andThen(runShooterSlow))\
*onRelease:*\
setShooterPose(handoff).andThen(runShooter)

*3 (source loading)*\
intakeSource.andThen(runShooter)\
*onRelease:*\
setShooterPose(handoff)

6 (feed through) - runShooterSlow.andThen(feedShooterManual)\
*onRelease:*\
stopShooter

7 (eject) - eject

**Right Joystick:**\
*Trigger (intake)*\
Intake.andThen(FinishIntaking).andThen(runShooter)

*2 (feed shooter)*\
feedShooterSmart.andThen(stopShooter)

*3 (manual feed shooter)*\
feedShooterManual\
*onRelease:*\
stopShooter
