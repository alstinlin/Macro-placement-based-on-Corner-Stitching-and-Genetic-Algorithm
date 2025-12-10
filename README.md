====================================================================
                NCKU-ePlacer (CPU Version for RockyLinux 8)
====================================================================

This package provides a Dockerized version of NCKU-ePlacer that
runs on RockyLinux 8 without requiring CUDA or GPU drivers.
All benchmarks and necessary runtime files are included inside 
the Docker image.

Users only need Docker installed on their machine.

--------------------------------------------------------------------
1. Create a Result Output Directory (Recommended)
--------------------------------------------------------------------

Before running the container, create a folder on your machine 
to store the output result files.

Example:

    mkdir result_output

--------------------------------------------------------------------
2. Launch Docker Container
--------------------------------------------------------------------

Use the following command to start a container:

    docker run --name NCKU-eplacer -it \
        -w /workspace \
        -v <local result directory>:/workspace/result \
        resulteplacer:release \
        /bin/bash

Example (using the folder created above):

    docker run --name NCKU-eplacer -it \
        -w /workspace \
        -v $(pwd)/result_output:/workspace/result \
        resulteplacer:release \
        /bin/bash

Explanation:
    --name NCKU-eplacer       : Assigns a name to the container
    -it                       : Interactive terminal
    -w /workspace             : Set working directory
    -v local:container        : Mount local folder to container output directory
    resulteplacer:release     : Docker image name
    /bin/bash                 : Starts a bash shell inside container

--------------------------------------------------------------------
3. Running the Placement Tool
--------------------------------------------------------------------

After entering the container, run:

    ./run.sh <benchmark_name>

Example:

    ./run.sh des_perf_a_md1

Notes:
    - Benchmarks are located inside /workspace/benchmarks
    - Output files will be written to /workspace/result
    - Because you mounted the result folder, outputs will also
      appear on your host machine inside the folder you specified.

--------------------------------------------------------------------
4. List Available Benchmarks
--------------------------------------------------------------------

Inside the container:

    ls /workspace/benchmarks

--------------------------------------------------------------------
5. Exit the Container
--------------------------------------------------------------------

Type:

    exit

--------------------------------------------------------------------
6. Restarting an Existing Container
--------------------------------------------------------------------

You do NOT need to create a new container every time.  
To restart the existing NCKU-eplacer container:

    docker start -i NCKU-eplacer

--------------------------------------------------------------------
7. Removing the Container (Optional)
--------------------------------------------------------------------

If you need to delete the container:

    docker rm -f NCKU-eplacer

--------------------------------------------------------------------

If you have any questions or encounter issues, please contact the 
tool developer or refer to project documentation.

====================================================================
                        End of README
====================================================================
