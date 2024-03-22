<img width="1352" alt="image" src="https://github.com/obround/ftcsim/assets/75817213/87095559-a8e4-4bc7-99e7-41e3f60f3ad2">

### NOTE: If you were wondering, bifunctor is my alt account

# FTCSIM (v1.0)
Trajectory planner and simulator for FTC Centerstage.

## Path to v2
Over the summer, the plan is to create a 3D simulator to more accurately simulate the game. Stay tuned!

## Installation
### Requirements
- IntelliJ Idea
- Default IntelliJ JDK set as [Amazon Corretto 11](https://docs.aws.amazon.com/corretto/latest/corretto-11-ug/downloads-list.html)
- Maybe JavaFX 17 (this *should* be automatically installed by Gradle, but this installation procedure has not been tested)
### Instructions
- Clone this repository and open it in IntelliJ
- Gradle should automatically build
- Open `Main.kt` (in `/src/main/kotlin`), and run it


## Features
### Implemented (v1.0):
- Trajectories can be created and simulated on the fly
- Trajectories can be saved and later loaded back into the simulator
- Trajectories can be exported for immediate use in the autonomous
- Path highlighting
- Score counting

### Todo (v1.1):
- Proper error handling
- Address TODOs in code
