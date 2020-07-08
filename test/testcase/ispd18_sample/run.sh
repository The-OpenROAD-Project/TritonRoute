#!/bin/bash

if [ "$#" -ne 1 ]; then
	exit 2
fi

binary=$1

$binary -lef /home/zf4_projects/FlexRoute/benchmark/ispd18_sample/ispd18_sample.input.lef -def /home/zf4_projects/FlexRoute/benchmark/ispd18_sample/ispd18_sample.input.def -guide /home/zf4_projects/FlexRoute/benchmark/ispd18_sample/ispd18_sample.input.guide -output ../../result/ispd18_sample/ispd18_sample.output.def | tee ../../result/ispd18_sample/ispd18_sample.log

./compare.tcl ./golden/ispd18_sample.log ../../result/ispd18_sample/ispd18_sample.log
compare_return_code=$?
exit $compare_return_code
