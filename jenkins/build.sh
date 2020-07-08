docker build --target=base-dependencies -t tritonroute .
docker run -v $(pwd):/tritonroute tritonroute bash -c "./tritonroute/jenkins/install.sh"
