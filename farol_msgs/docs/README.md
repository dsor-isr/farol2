# Farol Msgs Package

## Description

The *farol_msgs* package houses all custom ROS messages and services created for the FAROL stack.

| Message    | Description |
| -------- | ------- |
| ProcessState | To be used by the *farol_bringup* node, containing the internal state of a single process launched by the aforementioned node. |
| NavigationState | Defines the state of the vehicle, populated by a navigation filter. |
| UTM | Part of the NavigationState. Defines the UTM position and zone. |
| LocalDatum | Part of the NavigationState. A local reference for the altitude. |
| GCS | Part of the NavigationState. Position in latitude and longitude. |
| Measurement | Defines the possible types of measurement that feed back to the navigation filter. |

| Service    | Description |
| -------- | ------- |
| ProcessState | To be used by the *farol_bringup* node, containing the service that handles processes (start, kill, restart, etc.). |