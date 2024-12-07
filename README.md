# n-bodies-final

## Project Proposal

I would like to create a project that can produce video animations of certain physical phenomena. The first thing that comes to my mind is an n-body problem animation, where users can interact and set initial constraints for n-isolated bodies and then create an animation using python that would demonstrate how these bodies would move based on their initial conditions. This involves plugging the initial values into a differential equation and using numerical methods (e.g. Euler or Runge-Kutta) to solve for the paths over a certain time. This could be accompanied with concurrent plots showing the potential and kinetic energies of the bodies involved.


## To-Do / Execution

1. By the end of week 4, I will have all of the dependencies and packages I need for my project. I will do a little bit of research to refamiliarize myself with the numerical methods used to solve the physics problems. I will have an outline of the project architecture (i.e. user input, output, etc.)
2. By the end of week 5, I will hopefully implement the numerical solver to get the correct solution for a series of inputs. If not, hopefully it will be done for 2 and 3-body problems then I will scale up to larger ones
3. By the end of week 6, I will begin working on the animation aspect of the project. I think most of the coding will be done using the manim python library which is a package that helps create math animations. I will also find a way to measure the kinetic and potential energies of the bodies and implement them into the animation.
4. By the end of week 7, I will work on revising the visuals, making the animations look more smooth and to scale
5. By the end of week 8, I will test a simple user interface for user input and where the animation will display. I will optimize and run some animations to test.
6. Finally, I will write up some documentation and tidy up the project to make it a little more clean

## About

This project is contains some scripts for a flask web application for users to interact with and input in intial conditions for an n-body system and have the system be animated. The project uses the manimgl library to create the animations and the scipy library to solve the differential equations. 

The file ```simulation.py``` contains the main code for the simulation. The file ```app.py``` contains the flask application that will run the simulation. The file ```2d_sim.py``` contains a code to render a matplotlib graph of a 2d gravitational problem, where there is no movement along the z-axis. 

## About the Google drive

Here is the link: https://drive.google.com/drive/folders/1b_pEJhSkM3VrCMvc1XUQxSckjki9HvQK?usp=sharing

The images from the video presentation with 4-bodies and their energies are in there. The initial values are:
masses = [10, 20, 30, 40] 
initial_positions = [
            [-10, 10, 0],
            [0, 1, 0],
            [10, 10, 0],
            [1, 0, 0]
        ]
initial_velocities = [
            [0, 0, 0],
            [0, 0, 0],
            [0, 0, 0],
            [0, 0, 0]
        ]

There are two other videos in there. The 3-body video is the same one from the video presentation. The 4-body video has the following initial values:
masses = [10, 20, 30, 40]
initial_positions = [
    [-10, 10, 1],
    [0, 0, 2],
    [10, 10, -1],
    [0, 0, -2]
]
initial_velocities = [
    [0, 0, 0],
    [0, 0, 0],
    [0, 0, 0],
    [0, 0, 0]
]
