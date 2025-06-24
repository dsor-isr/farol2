# Control Allocation Package

## Description

<!-- The *nav_filters* package encapsulates the navigation filter algorithms and messages/services closely related to vehicle navigation. It is designed within the scope of live filter comparison and extensibility, such that new filters can be easily added with little effort (besides coding the algorithm itself, obviously).

The *sample_and_hold* node implements the simplest navigation filter, taking measurements and assigning these to the navigation state asynchronously, while the current state is published at a fixed frequency.

The *filter_handler* node is responsible for handling the different navigation filters' outputs, choosing which is redirected to the main navigation filter state, which is considered the true state of the vehicle from a control perspective. It is possible to define the default filter to be used and to change filters live, using a ROS service.

When implementing a new navigation filter algorithm in a new node, it is only required to:
- publish the navigation state to a topic with the ROS message `farol_msgs::msg::NavigationState`;
- add the new topic name to the list of subscribers of the *filter_handler* node (`nav.filter_handler.topics.subscribers`), in the *ros.yaml* configuration file;
- add the new filter name to the list of filters of the *filter_handler* node (`nav.filter_handler.filters`), in the *nav.yaml* configuration file;
- add the new node to the navigation launch file under `farol_bringup/launch/nav.launch.py`, similarly to the other algorithms' nodes. -->

## Nodes

<!-- * [filter_handler](filter_handler.md) -->
<!-- * [sample_and_hold](sample_and_hold.md) -->