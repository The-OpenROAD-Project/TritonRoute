docker run -v $(pwd):/tritonroute tritonroute bash -c "make -C /tritonroute/build test && cd /tritonroute/test/ && ./unit_test.sh /tritonroute/build/TritonRoute"
