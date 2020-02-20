# A* Planner to Catch a Moving Target
An A* motion planner written for Assignment 1 of the Graduate Course 16-782: Planning and Decision Making in Robotics, at Carnegie Mellon University.

An agent (in green) must plan a least-cost path to catch a target (in pink) with a predefined trajectory in any given map.

![Example](/images/maps.jpg)

## Compilation Instructions
To compile the planner, open the runtest.m file and type
```console
$ mex planner.cpp
```

To run the planner, type the name of the map to test (map1.txt or map2.txt ...)
```console
$ runtest('map?.txt')
```

## Academic Integrity
If you are currently enrolled in the Graduate 16-782 Planning and Decision Making in Robotics, or the Undergraduate 16-350 Planning Techniques for Robotics course at Carnegie Mellon University, please refer to [CMUs Academic Integrity Policy](https://www.cmu.edu/policies/student-and-student-life/academic-integrity.html) before referring to any of the contents of this repository.