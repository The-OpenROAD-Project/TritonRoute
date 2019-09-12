docker build -f jenkins/Dockerfile.dev -t tritonroute .
docker run -v $(pwd):/tritonroute tritonroute bash -c "./tritonroute/jenkins/install.sh"