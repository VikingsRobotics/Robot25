# Build

Go to [WPILib](https://docs.wpilib.org/en/stable/index.html) and install the required Visual Studio Code and compilers needed for your system. There are two ways to build this project.
1. In WPILib Visual Studio Code, run the command `WPILib: Build Robot Code` which will invoke the required compilers to build the robot code.
2. In the Project Directory, run in the command line `./gradlew build` which will invoke the required compiler to build the robot code as well as run the tests (this project contains no unit tests).

# Deploy

In WPILib Visual Studio Code, run the command `WPILib: Deploy Robot Code` when connected to the robot wireless access point/router or through an ethernet connection.

# Contributing

Before pushing code
1. In the project directory, run in the command line `./gradlew spotlessApply`.
2. This will edit the code to conform to spotless C++ style guide.
3. Add these edited files to git staging then push.
