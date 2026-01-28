# Requirements Document

## Introduction

This specification addresses the issue where the MyCobot Pro 450 robot becomes unresponsive after detecting a low height condition (below 180mm), even when the slider is moved to a safe position above the threshold. The system should allow recovery from low-height conditions when safe target positions are commanded.

## Glossary

- **End_Effector**: The gripper or tool at the end of the robot arm
- **Target_Height**: The calculated height of the end effector based on commanded joint angles
- **Current_Height**: The actual real-time height of the end effector from the coords broadcaster
- **Safe_Height_Threshold**: The minimum acceptable height (180mm) to prevent ground collision
- **Height_Monitor**: Background thread that monitors current end effector height
- **Command_Executor**: Thread that processes and sends commands to the robot
- **Slider_Control_System**: The ROS node that translates slider inputs to robot commands

## Requirements

### Requirement 1: Height Safety Monitoring

**User Story:** As a robot operator, I want the system to continuously monitor end effector height, so that the robot stops before colliding with the ground.

#### Acceptance Criteria

1. WHEN the Current_Height falls below Safe_Height_Threshold, THEN the Height_Monitor SHALL set the stopped state to true
2. WHEN the Current_Height falls below Safe_Height_Threshold, THEN the Height_Monitor SHALL call the robot stop command
3. WHEN the Current_Height rises above Safe_Height_Threshold, THEN the Height_Monitor SHALL set the stopped state to false
4. THE Height_Monitor SHALL check height at a rate of at least 10Hz

### Requirement 2: Command Validation and Execution

**User Story:** As a robot operator, I want commands with safe target heights to be executed even after a low-height condition, so that I can recover the robot to a safe position.

#### Acceptance Criteria

1. WHEN a command has Target_Height below Safe_Height_Threshold, THEN the Command_Executor SHALL reject the command and log a warning
2. WHEN a command has Target_Height above Safe_Height_Threshold, THEN the Command_Executor SHALL execute the command regardless of stopped state
3. WHEN a safe command is executed after a low-height condition, THEN the Command_Executor SHALL clear the stopped state
4. WHEN a command is rejected due to low Target_Height, THEN the Command_Executor SHALL increment the commands_skipped counter

### Requirement 3: State Synchronization

**User Story:** As a robot operator, I want the safety state to be consistent across monitoring and execution threads, so that the system behaves predictably.

#### Acceptance Criteria

1. WHEN the stopped state is modified, THEN the system SHALL use thread-safe locking mechanisms
2. WHEN the Height_Monitor detects safe height, THEN the stopped state SHALL be cleared within 100ms
3. WHEN the Command_Executor executes a safe command, THEN the stopped state SHALL be cleared immediately
4. THE system SHALL log state transitions for debugging purposes

### Requirement 4: Recovery Behavior

**User Story:** As a robot operator, I want clear feedback when the system recovers from a low-height condition, so that I know the robot is operational again.

#### Acceptance Criteria

1. WHEN a safe command is executed after being stopped, THEN the system SHALL log a recovery message
2. WHEN the Current_Height returns to safe levels, THEN the system SHALL log a recovery message
3. WHEN in stopped state, THEN the system SHALL log warnings at most once per second to avoid spam
4. THE system SHALL distinguish between "target too low" and "current height too low" in log messages

### Requirement 5: Command Queue Management

**User Story:** As a robot operator, I want the command queue to be cleared when recovering from a stopped state, so that stale low-height commands don't execute.

#### Acceptance Criteria

1. WHEN transitioning from stopped to running state, THEN the Command_Executor SHALL clear any queued commands with unsafe Target_Height
2. WHEN a safe command arrives during stopped state, THEN the system SHALL process it immediately
3. THE Command_Executor SHALL process the most recent command from the queue, not the oldest
4. WHEN the queue is full, THEN the system SHALL discard the oldest command to make room for new ones
