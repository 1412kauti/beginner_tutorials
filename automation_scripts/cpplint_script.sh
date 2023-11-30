#!/bin/bash

# Define the master output file
master_output="master_cpplint_out.txt"

# Navigate to the beginner_tutorials directory
cd ..

# Loop through each package and run cpplint
for package in cpp_parameters cpp_pubsub cpp_srvcli tutorial_interfaces tf2_test
do
    echo "Running cpplint for $package..."
    cpplint --recursive $package/src/ >> $package/cpplint_out.txt
    echo "cpplint for $package completed."
    
    # Append the package's cpplint output to the master output file
    cat $package/cpplint_out.txt >> $master_output
done

# Return to the original directory
cd ..
