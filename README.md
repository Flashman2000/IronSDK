<div align="center">
  <img src="https://i.imgur.com/9nZyLP6.jpg" width="70%" height="70%" class="center">
  <br></br>
<b>Developed by Vedant Thorat of Ironclad #8080</b>
  <br/>
  <i>Version World's Edition v1.0 | Updated 3/21/2019</i>
</div>

# IronSDK
The beta branch.... name kinda says it. This is the beta testing branch. Code in here usually isn't fully developed and may or may not be unstable. Beware.

## Project Status:
**Our programming gnomes are working hard to get us World's Edition v1.1**

## Changelog:

### Version 0.1-beta (9/13/18)
- Method Repository 'robot' created
- Skeleton RC Program created called 'IronDrive_BETA'

### Version 0.1.1-beta (9/14/18)
- Vuforia-based decision making is now in development

### Version 0.2-beta (9/14/18)
- Last update before physical testing with robot
- Cleaned up 'IronDrive_BETA' file
- Added remote control protocol method to 'robot'
- Made updates to Vuforia-based decision making program

### Version 0.2.1-beta (9/20/18)
- Bug fixes for upcoming update

### Version 0.2.2-beta (9/21/18)
- Added a drivable opmode called 'VuDrive'
  - Is drivable
  - Used for testing vuforia coordinate system
- 'IronDrive_BETA' renamed to 'IronRC_BETA'
- Class 'IronAutonomous_BETA' which is currently empty
  - Will be later developed

### Version 0.3.0-beta (9/21/18)
- OpenCV is now in development
  - Only works for last year's objects
  - Will be developed for the new Rover Ruckus objects
  
### Version 0.3.1-beta (9/21/18)
- Bug fixes

### Version 0.4.0-beta_unstable (9/23/18)
**This version of IronSDK is EXTREMELY unstable. IT WILL NOT BUILD IN ANDROID STUDIO. Building results in a compiler error. Will be fixed in a future update**
- OpenCV can now be used to complete Rover Ruckus Objectives
- OpenCV Updated to OpenCV3
- Known fatal bugs with using vuforia with opencv

### Version 0.4.0-beta (9/24/18)
- Patched fatal bug with using vuforia with opencv

### Version 0.4.1-beta (10/11/18)
- DogeCV Compter vision library has been updated to v2018.2.1
- DogeCV implemented into robot.java repository and in IronRC_BETA.java

### Version 0.5.0-beta (10/23/18)
- First version of beta that is close to competition ready
- SDK is mostly competition ready.
  - Tweaks may be made to teleop depending on driver pref.
    - in which case a hotfix will be released
- Details on changes:
  - IronRC is now competition ready
  - IronAutonomous is in theoretical development stage (yet to test)
  - DogeCV now works with the new 2018-2019 challenge items
  - class 'robot' is now updated with new methods to help in autonomous and utilize REV imu

### Version 0.5.1-beta (10/27/18)
- Beginning of development for competition-level autonomous
- Tele-op is completely done with technical development

### Version 0.5.2-beta (10/29/18)
- Created test autonomous with method integration
- Implementation of initialization methods in competition autonomous
- New 'RobotMovements.java' file that houses mineral scanning methods
- Addition of new variables to 'VarRepo'

### Version 0.6.0-beta (11/7/18)
- Update to FTC SDK version 4.3

### Version 1.0.0 (11/8/18)
- Our first competition ready release!
  - Autonomous is competition ready
  - Remote Control is competition ready
    - Horizontal x-rails are disabled in code due to hardware difficullties that render them unsuable

### Version 1.1-beta (1/18/19)
- LED Driver support
- Minor autonomous tweaks (renaming of files for ease of use)

### Version 1.5-beta (2/5/19)
- Remodeling of autonomous for new chassis
  - Encoder support
  - Pivot arm support
- Creation of type LinearOpMode teleop program
  - New pivot arm support
- Old teleop program deprecated
- Remodeling of methods in 'RobotConfigs' around new robot

### Version 2.0-beta (2/25/19)
- Almost
- Autonomous completed
- Teleop Finalized
- Added REV Blining LED Driver support

### Version 3.0
- Final Competition SDK
- Autonomous perfected
- Inclusion of double sampling autonomous (95 points total)
- TeleOperative ease of use features added

### World's Edition v1.0 (3/21/19)
- New versioning system in order to commemorate, and *maybe* flex :)
- Updated to DogeCV 2019.1 (https://github.com/MechanicalMemes/DogeCV)
- New methods in development for greater effeciency and streamlining
- New test autonomous
- Teleop redesgined around a mecanum drive-train (yet to be tested)
- ALOT of clean up/optimization
- Recovered the readme. Welcome back :)
