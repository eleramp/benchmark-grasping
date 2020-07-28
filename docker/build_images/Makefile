#default: latest intel nvidia

# =====
# TOOLS
# =====

superquadric:
	docker build --rm \
	    --build-arg from=nvidia/cuda:10.0-cudnn7-devel-ubuntu18.04 \
	    --tag eleramp/superquadric_grasp_planner:latest \
	    superquadric_based/

gpnet:
	docker build --rm \
		--build-arg from=nvidia/cuda:10.0-cudnn7-devel-ubuntu18.04 \
		--tag eleramp/graspnet:latest \
		graspnet/

