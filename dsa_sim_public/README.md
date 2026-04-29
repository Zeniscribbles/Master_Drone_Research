# Master_Drone_Research# DOTS fast simulator

Based on Box2D, with GBP swarm-centric distributed frame of reference.

Intended to support evo experiments.


### Prerequisites

```
GLFW
GLEW
Box2D
Eigen3
Behaviortree.cpp
```


## Installing Behaviortree.cpp

Also needed SQLite but I already had them installed.

```
brew install googletest


git clone --depth 1 --branch  4.3.5 https://github.com/BehaviorTree/BehaviorTree.CPP.git
cd BehaviorTree.CPP
mkdir build; cd build
cmake -DBTCPP_GROOT_INTERFACE=OFF ..
make -j10
sudo make install
```

## Building

```
git clone https://simonj23@bitbucket.org/hauertlab/dsa_sim_public.git
cd dsa_sim_public
mkdir build
cd build
cmake ..
make
```

## Running interactively
```
./sim
```

## Running simulations for paper

Create jobfile:
```
../scripts/run_gbp.sh
```

Run with parallel:
```
parallel -j 20 <jobfile
```


