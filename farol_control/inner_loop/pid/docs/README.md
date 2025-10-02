# PID package

## Description
The *pid* ROS package contains PID controllers for multiple DOFs (Degrees of Freedom) of a vehicle. All PID controllers were designed having the Delta Implementation in mind, i.e., rearranging the control loop in order to have an integrator before the actuation on the system.

Currently implemented controllers are the following:
- Surge,
- Sway,
- Heave,
- Yaw,
- Pitch,
- Roll,
- Yaw Rate,
- Pitch Rate,
- Roll Rate.

An Attitude controller is also accounted for in this node (having in vision the integration of Bottom Following in *FAROL2*), but it is not actually implemented at the moment.

## Configuration

A default configuration file *control.yaml* is available in the *farol_bringup* package, where parameters can be tuned.

To turn on different controllers independently, simply change the *enabled* flag for each controller in the configuration file. This architecture makes it possible for the implementation of other inner-loop control packages (with the same base principal of parameter handling as the *pid* package) under the *farol_control/inner_loop/* folder structure and provides the means to have different types of controllers for different DOFs. An example follows:

```yaml
/**:
  ros__parameters:
    control:
      inner_loop:
        pid:
          surge:
            enabled: true # <===== PID SURGE ENABLED
            kp: 10.0
            ki: 1.0
            lpf_pole: 31.4 # rad/s
            tau_min: -30.0 # N
            tau_max: 30.0 # N
          sway:
            enabled: false # <===== DISABLED
            kp: 1.0
            ki: 1.0
            lpf_pole: 31.4 # rad/s
            tau_min: -20.0 # N
            tau_max: 20.0 # N
        sliding_mode: # an example of a different control structure that could be implemented
          surge:
            enabled: false # <===== DISABLED
            param_1: 1.0
            param_2: 1.0
            # ...
          sway:
            enabled: true # <===== SLIDING MODE SWAY ENABLED
            param_1: 1.0
            param_2: 1.0
            # ...
```

Parameters can also be changed live through the *ChangeParams* service.

## Implementation Details

Some important implementation details to take into consideration.

### References callback

Given the frequency $f_i$ of a controller $i$ (defined in the *ros.yaml* configuration file), this controller should only compute an actuation if a reference has been received since the last computation cycle. Ideally (Scenario 1), the controller would compute an actuation if the last reference had been received less than or exactly $1/f$ seconds ago, but fluctuations in the frequency at which references are published could cause the controller to not compute an actuation even when the reference is still being published.

In order to avoid this, it was decided that a reference is considered recent if it was received less than $2/f$ seconds ago (Scenario 2). In practice, the only drawback of this approach compared to Scenario 1 is that the controller will compute an actuation for one cycle more after the reference has stopped.

## Nodes

* [pid](pid.md)