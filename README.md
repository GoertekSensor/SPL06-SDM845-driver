# SPL06-SDM845-driver
Qualcomm Incorporated SDM845-SSC-SEE--goertek-pressure-sensor-SPL06-driver

Step1.Register the driver with the SEE framework.

To link a driver in the SSC image statically, the driver must be registered with the SEE framework.
In OpenSSC 5.0, add the driver’s registration function in the sns_register_sensor_list[] list that is located in \<root>\ssc\framework\src\
hexagon\sns_static_sensors.c. 

Step2. Update the registry to configure the newly added driver on SLPI.

1. Add two JSON files required for the physical sensor.
a.Platform-specific .json file –vendor/qcom/proprietary/sensors-see/ssc/registry/config/sdm845_lsm6dsm_0.json
b.Driver-specific configuration for the physical sensor –vendor/qcom/proprietary/sensors-see/ssc/registry/config/lsm6dsm_0.json

2. Compile the application processor build and load the device with the compiled application processor build.
a.Registry (.json) files are present at /persist/sensors/registry/config.
b.On successful parsing of these files, the appropriate parsed files appear in /persist/sensors/registry/registry.

Note: Build command lists for example
python ssc/build/config_nanopb_dependency.py -f nanopb-0.3.6-linux-x86
python ./build/build.py -c sdm845 -o all -f TOUCH_USES_PRAM
python ./build/build.py -c sdm845 -o clean

