build:
	docker build -t super .

build-clone:
	docker build --build-arg DUMMY=`date +%s` -t super .
run:
	docker run  -it \
			--gpus all \
			--env="NVIDIA_DRIVER_CAPABILITIES=all" \
			--privileged \
			-e DISPLAY=${DISPLAY} \
			--network=host \
			--env="DISPLAY" \
			--env="QT_X11_NO_MITSHM=1" \
			--rm super
