# paths Node
This node deals with the rationale previously explained in the *paths* documentation. It manages and deploys paths for the vehicles to follow. A detailed exaplanation is offered here:

* [Implementation](implementation.md)

## Diagram
![paths Diagram](img/paths.png)

## Subscribers
| Subscribers | msgs type | Purpose |
| ----------- | -------------- | --------- |
| /#vehicle#/control/outer_loop/gamma | [std\_msgs/Float32](http://docs.ros.org/en/api/std_msgs/html/msg/Float32.html) | The coordination state of a certain vehicle with respect to others on the network |
| /#vehicle#/nav/filter/state | farol\_msgs/NavigationState | The navigation state of the vehicle after filtering                               |

## Publishers
| Publishers | msgs type | Purpose |
| ----------- | -------------- | --------- |
| "/#vehicle#/control/outer_loop/path_data" | [paths/PathData](PathData.md) | Message with the path data on a certain point (curvature, tangent, velocity, etc.) |
| "/#vehicle#/control/outer_loop/virtual_state" | farol\_msgs/StateConsole | Vehicle state message                                                              |

## Services
| Services                      | msgs type                                 | Purpose                                                                  |
| -----------                   | --------------                            | ---------                                                                |
| /#vehicle#/ResetPath          | [paths/ResetPath](ResetPath.md)           | Reset the current assigned path                                          |
| /#vehicle#/SetConstVdRabbit   | [paths/SetConstSpeed](SetConstSpeed.md)   | Set a constant speed for the rabbit which a certain vehicle is following |
| /#vehicle#/SetConstVdVehicle  | [paths/SetConstSpeed](SetConstSpeed.md)   | Set a constant speed for the vehicle itself                              |
| /#vehicle#/SetMode            | [paths/SetMode](SetMode.md)               | Set if mode of operation calculates closest point to the path or if it receives an external gamma for path progression |
| /#vehicle#/SpawnArc2DPath     | [paths/SpawnArc2D](SpawnArc2D.md)         | Create a 2D circumference arc path                                       |
| /#vehicle#/SpawnBernoulliPath | [paths/SpawnBernoulli](SpawnBernoulli.md) | Create a Bernoulli path                                                  |
| /#vehicle#/SpawnCircle2DPath  | [paths/SpawnCircle2D](SpawnCircle2D.md)   | Create a 2D Circle path                                                  |
| /#vehicle#/SpawnLineDPath     | [paths/SpawnLine](SpawnLine.md)           | Create a Line path                                                       |

## Parameters
| Parameters      | type   | Purpose                       |
| -----------     | ----   | ---------                     |
| frame\_id       | string | ID assigned to the path frame |
| node\_frequency | float  | Working frequency of the node |
