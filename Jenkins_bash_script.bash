#!/bin/bash


# Configure ssl for git
echo -e "[http \"https://gitlab.intranet.eprosima.com\"]\n\tsslVerify = false" > /etc/gitconfig

# VARIABLES
WORK_DIR="/root/"
#WORK_DIR="$(PWD)"


# VARIABLES (FOR LOCAL TESTING)
if [ "" == "" ]
then
    FEATURE_TO_TEST=""

    AGENT_REPO_LIST_URL="https://raw.githubusercontent.com/microROS/micro-ROS-doc/master/repos/agent_minimum.repos"
    AGENT_UROS_PATH="uros"
    AGENT_PACKAGES_TO_TEST=""
    AGENT_IGNORE_PACKAGE_RESULT="ament_tools osrf_pycommon ament_copyright ament_lint_cmake ament_package ament_pep257 microxrcedds_agent_cmake_module rcl rcutils rosidl_generator_cpp rosidl_parser"
    AGENT_SET_BRANCHES=""

    CLIENT_REPO_LIST_URL="https://raw.githubusercontent.com/microROS/micro-ROS-doc/master/repos/client_minimum.repos"
    CLIENT_UROS_PATH="uros"
    CLIENT_PACKAGES_TO_TEST=""
    CLIENT_IGNORE_PACKAGE_RESULT="ament_tools osrf_pycommon ament_copyright ament_lint_cmake ament_package ament_pep257 rcl_lifecycle builtin_interfaces diagnostic_msgs geometry_msgs lifecycle_msgs nav_msgs rcl rclcpp rcutils rmw_microxrcedds rosidl_generator_cpp rosidl_parser rosidl_typesupport_microxrcedds_shared sensor_msgs std_msgs std_srvs test_msgs"
    CLIENT_SET_BRANCHES=""
    CLIENT_BUILD_SHARED_LIBS="ON"
fi


#--------------
# AGENT CONFIG
agent_script_args=""

# URL
if [ ! "${AGENT_REPO_LIST_URL}" == "" ]
then
    agent_script_args="$agent_script_args  --url ${AGENT_REPO_LIST_URL}"
fi

# Branch work path
if [ ! "${AGENT_UROS_PATH}" == "" ]
then
	agent_script_args="$agent_script_args  --scope_folder ${AGENT_UROS_PATH}"
fi

# Packages to test
packages_arg=""
for package in ${AGENT_PACKAGES_TO_TEST}
do 
  packages_arg="$packages_arg ${package}"
done
if [ ! "${packages_arg}" == "" ]
then
    agent_script_args="$agent_script_args --package_to_test $packages_arg"
fi

# Packages test to ignore
Ignore_package_result_arg=""
for Ignore in ${AGENT_IGNORE_PACKAGE_RESULT}
do 
  Ignore_package_result_arg="$Ignore_package_result_arg ${Ignore}"
done
if [ ! "${Ignore_package_result_arg}" == "" ]
then
    agent_script_args="$agent_script_args --Ignore_package_result $Ignore_package_result_arg"
fi

# Branches to test (change)
for package in ${AGENT_SET_BRANCHES}
do 
   IFS=':' 
  value=($package) 
  agent_script_args="$agent_script_args  --branch $UROS_PATH/${value[0]}  ${value[1]}"
  IFS=' '
done

# Feature to test
if [ ! "${FEATURE_TO_TEST}" == "" ]
then
    agent_script_args="$agent_script_args  --feature_to_test ${FEATURE_TO_TEST}"
fi




#---------------
# CLIENT CONFIG
client_script_args=""

# URL
if [ ! "${CLIENT_REPO_LIST_URL}" == "" ]
then
    client_script_args="$client_script_args  --url ${CLIENT_REPO_LIST_URL}"
fi

# Branch work path
if [ ! "${CLIENT_UROS_PATH}" == "" ]
then
	client_script_args="$client_script_args  --scope_folder ${CLIENT_UROS_PATH}"
fi

# Test shared build
#if [ "${CLIENT_BUILD_SHARED_LIBS}" == "OFF" ]
#then
    #client_script_args="$client_script_args --Build_extra_args \"--cmake-args -DBUILD_SAHRED_LIBS=ON\""
#else
    #client_script_args="$client_script_args --Build_extra_args \"--cmake-args  -DBUILD_SAHRED_LIBS=OFF\""
#fi


# Packages to test
packages_arg=""
for package in ${CLIENT_PACKAGES_TO_TEST}
do 
  packages_arg="$packages_arg ${package}"
done
if [ ! "${packages_arg}" == "" ]
then
    client_script_args="$client_script_args --package_to_test $packages_arg"
fi

# Packages test to ignore
Ignore_package_result_arg=""
for Ignore in ${CLIENT_IGNORE_PACKAGE_RESULT}
do 
  Ignore_package_result_arg="$Ignore_package_result_arg ${Ignore}"
done
if [ ! "${Ignore_package_result_arg}" == "" ]
then
    client_script_args="$client_script_args --Ignore_package_result $Ignore_package_result_arg"
fi

# Branches to test (change)
for package in ${CLIENT_SET_BRANCHES}
do 
   IFS=':' 
  value=($package) 
  client_script_args="$client_script_args  --branch $UROS_PATH/${value[0]}  ${value[1]}"
  IFS=' '
done


# Feature to test
if [ ! "${FEATURE_TO_TEST}" == "" ]
then
    client_script_args="$client_script_args  --feature_to_test ${FEATURE_TO_TEST}"
fi


#-------------
# Execute test

#Execute script with arguments
python Exec_repo_Test.py    --work_dir="${WORK_DIR}/Agent_ws" \
                            ${agent_script_args} \
                            --file \
                            --Exec_command_after_build "install/uros_agent/lib/uros_agent/uros_agent udp 8888"
if  [ ! "$?" == "0" ]
then
    exit $?
fi

#Execute script with arguments
if [ "${CLIENT_BUILD_SHARED_LIBS}" == "OFF" ]
then
    python Exec_repo_Test.py    --work_dir="${WORK_DIR}/Client_ws" ${client_script_args}
                                --Build_extra_args "--cmake-args -DBUILD_SHARED_LIBS=OFF"
    if  [ ! "$?" == "0" ]
    then
        exit $?
    fi
else
    python Exec_repo_Test.py --work_dir="${WORK_DIR}/Client_ws" ${client_script_args} --Build_extra_args "--cmake-args -DBUILD_SHARED_LIBS=ON"
    if  [ ! "$?" == "0" ]
    then
        exit $?
    fi
fi
