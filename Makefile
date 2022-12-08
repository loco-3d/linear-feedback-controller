SHELL := /bin/bash
YOUR_WS := /home/mnaveau/devel/workspace_wbmpc


## Run the docker
docker_pull:
	docker pull gitlab.laas.fr:4567/gsaurel/docker-pal:gsaurel

docker_run:
	sudo chown -R :gepetto $(YOUR_WS)  # you might need sudo if you get errors here
	sudo chmod -R g+rwX  $(YOUR_WS)
	xhost +local:
	docker run --rm -v $(YOUR_WS)/src:/ws/src -v $(YOUR_WS)/src/linear_feedback_controller/Makefile:/ws/Makefile -v /home/$(USER)/devel:/home/user/devel -v $(YOUR_WS)/build:/ws/build --gpus all --net host -e DISPLAY -it gitlab.laas.fr:4567/gsaurel/docker-pal:gsaurel

docker_pull_and_run:
	make docker_pull
	make docker_run

docker_connect:
	docker exec --workdir=/ws -u user -it `docker ps --latest --quiet` bash


## Run the example:
reset:
	reset

build:
	catkin build linear_feedback_controller

simu:
	make reset
	make build
	roslaunch talos_pal_physics_simulator talos_pal_physics_simulator_with_actuators.launch robot:=full_v2

default_ctrl:
	roslaunch talos_controller_configuration default_controllers.launch

lf_ctrl:
	roslaunch linear_feedback_controller talos_linear_feedback_controller.launch simulation:=true default_params:=true

pd_ctrl:
	catkin build linear_feedback_controller
	source install/setup.bash && rosrun linear_feedback_controller pd_controller
