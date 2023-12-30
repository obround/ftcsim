<img width="1352" alt="image" src="https://github.com/obround/ftcsim/assets/75817213/72534d76-3959-4ee9-9735-c6fe523526ee">

# FTCSIM (v1.0)
Trajectory planner and simulator for FTC Centerstage.

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
- Custom robot starting position (currently hard coded)
- Proper error handling
- New icon set
