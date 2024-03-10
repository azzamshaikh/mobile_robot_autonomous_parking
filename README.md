# Mobile Robot Path Planning

This repository contains code to navigate a mobile robot to a goal position. In this case, the goal position is a parallel parked position. 

Three general vehicles will be tasked to complete this task: a differential drive robot, a car, and a car pulling a trailer.

A full report write up for these simulations can be found in the `docs` folder

## Dependencies 

The code used for the simulation was developed with Python 3.11 and Pygame 2.4. 

## Differential Drive Robot

To run the simulation, from the `src/robot` directory of this repo, run the following command. The result for the differential drive robot is shown below.

```
python diff_drive_valet.py
```



![Alt Text](media/diff_drive_valet.gif)

## Car

For the car, there are two versions available. The main difference is the sorting method used for the priority queue. One version sorts the queue based on the heuristic f cost - `car_valet_f_sort.py` - while the other version sorts based on the heuristic h cost - `car_valet_h_sort.py`.

To run the simulations, from the `src/car` directory of this repo, run the following commands. The results for the car is shown below.

```
python car_valet_f_sort.py
```

![Alt Text](media/car_valet_f_cost_sort.gif)

```
python car_valet_h_sort.py
```

![Alt Text](media/car_animation_h_cost_sort.gif)


## Truck with Trailer

For the truck with a trailer, similar to the car, two versions are available based on the sorting order for the priority queue, `truck_valet_f_sort.py` and `truck_valet_h_sort.py`.

To run the simulations, from the `src/truck` directory of this repo, run the following commands. The results for the truck is shown below.

```
python truck_valet_f_sort.py
```

![Alt Text](media/truck_animation_f_cost_sort.gif)

```
python truck_valet_h_sort.py
```

![Alt Text](media/truck_animation_h_cost_sort.gif)
