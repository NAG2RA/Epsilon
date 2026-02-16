# Epsilon — 2D Physics Engine Prototype

**Epsilon** is a multithreaded 2D rigid body physics engine prototype written in **C++20**, focused on real-time stability, accurate collision handling, and scalable simulation architecture.

The project was built as a deep exploration of physics engine fundamentals, including collision detection, constraint solving, multithreaded execution, and simulation stability techniques commonly used in modern game engines.

----------

##  Features

### Collision Detection & Resolution

-   Broad-phase collision detection using AABB pruning
    
-   Narrow-phase collision detection using SAT
    
-   Quadtree spatial partitioning
    
-   Stable collision resolution with static and dynamic friction
    
-   Warm-started constraint solving
    

### Physics Simulation

-   Spring and thread-like joint simulation
    
-   Realistic air resistance modeling
    
-   Density-based liquid buoyancy simulation
    
-   Explosion force simulation
    

### Stability & Optimization

-   Island-based rigid body sleeping
    
-   Constraint warm starting for solver stability
    
-   Stack-friendly collision handling
    

### Multithreading

-   Task-based parallel simulation pipeline
    
-   Scheduler-driven workload distribution
    

----------

## Project Goals

-   Explore real-time physics engine architecture
    
-   Implement stable rigid body simulation techniques
    
-   Investigate multithreaded physics execution
    
-   Gain practical experience with collision systems and solvers
    

This project prioritizes learning, experimentation, and architectural clarity over production completeness.

----------

## Demo

The demo application showcases:

-   Rigid body stacking and collision response
    
-   Joint behavior
    
-   Buoyancy interaction
    
-   Explosion dynamics
    
-   Real-time multithreaded simulation


Rendering is handled using OpenGL with a lightweight windowing framework.

## Build & Run

### Requirements

-   Windows
    
-   Visual Studio 2022 (or compatible C++20 compiler)
    
-   OpenGL-capable graphics environment
    

### Steps

1.  Clone the repository
    
    `git clone https://github.com/NAG2RA/Epsilon.git` 
    
2.  Open `Epsilon.sln` in Visual Studio
    
3.  Build and run the demo project
    

----------

## Learning References

This project was inspired by educational and open-source resources that explore physics engine design:

-   Two-Bit Coding — “Let’s Make a Physics Engine” Youtube playlist
    
-   enkiTS task scheduler library
    

All implementations in Epsilon are original, based on learned concepts rather than copied code.

----------

## Author

Developed by **Vladimir Shirkhanyan**

----------

## Project Status

Epsilon is an actively evolving prototype intended for experimentation, learning, and portfolio demonstration. Features and architecture may change as new ideas are explored.

----------

## Contributions

Feedback, suggestions, and discussions are welcome. This project exists primarily as a learning and exploration platform.

----------
