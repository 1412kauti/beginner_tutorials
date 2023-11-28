#!/bin/bash

# Define the master output file
master_output="master_cpp_check_out.txt"

cd ..

# Loop through each package and run cppcheck
for package in cpp_parameters cpp_pubsub cpp_srvcli tutorial_interfaces
do
    echo "Running cppcheck for $package..."
    cppcheck --enable=all --std=c++17 --suppress=missingInclude $package/src/ >> $package/cppcheck_out.txt
    echo "cppcheck for $package completed."
    
    # Append the package's cppcheck output to the master output file
    cat $package/cppcheck_out.txt >> $master_output
done

# Return to the original directory
cd ..
