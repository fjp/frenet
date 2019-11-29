# Trajectory Planning in the Frenet Space

Path planning in frenet coordinates:

<figure>
    <a href="/python/optimal_frenet.gif"><img src="/python/optimal_frenet.gif"></a>
    <figcaption>Path Planning using frenet coordinates.</figcaption>
</figure>

## Algorithm

1. **Determine the trajectory start state** \[x1,x2,theta,kappa,v,a\](0)
The trajectory start state is obtained by evaluating the previously calculated trajectory
at the prospective start state (low-level-stabilization). 
At system initialization and after reinitialization, the current vehicle 
position is used instead (high-level-stabilization).
2. **Selection of the lateral mode**
Depending on the velocity v the time based (d(t)) or running length / arc length based (d(s))
lateral planning mode is activated. By projecting the start state onto the reference curve the
the longitudinal start position s(0) is determined. The frenet state vector 
\[s,ds,dds,d,d',d''\](0) can be determined using the frenet transformation.
For the time based lateral planning mode, \[dd, ddd\](0) need to be calculated.
3. **Generating the laterl and longitudinal trajectories**
Trajectories including their costs are generated for the lateral (mode dependent) 
as well as the longitudinal motion (velocity keeping, vehicle following / distance keeping) in the frenet space.
In this stage, trajectories with high lateral accelerations with respect to the reference
path can be neglected to improve the computational performance.
4. **Combining lateral and longitudinal trajectories**
Summing the partial costs of lateral and longitduinal costs using
J(d(t),s(t)) = Jd(d(t)) + ks*Js(s(t)), for all active longidtuinal mode every
longitudinal trajectory is combined with every lateral trajectory and transfromed
back to world coordinates using the reference path. The trajectories are verified if they obey physical driving limits by
subsequent point wise evaluation of curvature and acceleration. 
This leads to a set of potentially drivable maneuvers of a specific mode in world coordinates.
5. **Static and dynamic collision check**
Every trajectory set is evaluated with increasing total costs if static and dynamic 
collisions are avoided. The trajectory with the lowest cost is then selected.
6. **Longitudinal mode alternation**
Using the sign based (in the beginning) jerk da(0), the trajectory with the
strongest decceleration or the trajectory which accelerates the least respectively 
is selected and passed to the controller.

## Usage

### Jupyter Notebook

In the python folder you find a Jupyter Notebook which shows the described planning algorithm.

### Frenet GUI

Note: The Frenet GUI is not functional yet. Contributions are welcome. Here's the plan for this GUI:

Allows you to generate trajectories in a local (world) reference frame from two quintic polynomials.
One polynomial describes the longitudinal direction and the other one the lateral. 
Providing a reference path and applying the Frenet coordinate transformation on this path will result in a trajectory.

The GUI was created with python3 in a conda environment:

```
conda create -n frenetenv python=3.6
pip install PySide2
pip install matplotlib
```

Execute `frenet.sh` run the GUI. This will call `uic` (Qt's user interface compiler) to process the `ui` file. 
Afterwards the `main.py` will be executed.

### References

- [Optimal Trajectory Generation for Dynamic Street Scenarios in a Frenet Frame](https://www.researchgate.net/profile/Moritz_Werling/publication/224156269_Optimal_Trajectory_Generation_for_Dynamic_Street_Scenarios_in_a_Frenet_Frame/links/54f749df0cf210398e9277af.pdf)
- [Dissertation](https://www.ksp.kit.edu/download/1000021738)

### Python code:

- https://github.com/AtsushiSakai/PythonRobotics#optimal-trajectory-in-a-frenet-frame
- https://github.com/AtsushiSakai/PythonRobotics/tree/master/PathPlanning/FrenetOptimalTrajectory


## Helper Function

https://de.mathworks.com/matlabcentral/fileexchange/22441-curve-intersections
