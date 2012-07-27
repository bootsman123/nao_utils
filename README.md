# Nao Robot Experiment
Code used for an experiment with our Nao. It is not functional yet, though should become in the next few weeks. Furthermore, it contains many pieces of old code which should be removed anytime soon as well.

## Basic usage
To start the 'Setup Interface':

	PARENT_IP=<NAO_IP> roslaunch nao_experiment setup_gui.launch

After setting up the experiment, one can use:

	PARENT_IP=<NAO_IP> roslaunch nao_experiment nao_experiment.launch
