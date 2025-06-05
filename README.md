# N-Bodies Simulation

## About

This project is contains some scripts for a flask web application for users to interact with and input in intial conditions for an n-body system and have the system be animated. The project uses the manimgl library to create the animations and the scipy library to solve the differential equations. 

The file ```simulation.py``` contains the main code for the simulation. The file ```app.py``` contains the flask application that will run the simulation. The file ```2d_sim.py``` contains a code to render a matplotlib graph of a 2d gravitational problem, where there is no movement along the z-axis. 

You can run any manimgl script by running the following command in the terminal:
```bash
manimgl <script_name>.py class_name
```

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
