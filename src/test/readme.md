# Running Tests
Run all tests using the `./gradelw` command from the main repo folder: `FRC-2025`

To run all the tests:
```
./gradlew test
```

Tests results are cached. Tests will only rerun if their source files change. To force rerunning tests:
```
./gradlew test --rerun-tasks
```

To run a specific test:
```
./gradlew test --tests [TestClassPath]

# example:
./gradlew test --tests frc.pathplanner.PathPlannerTest
```

