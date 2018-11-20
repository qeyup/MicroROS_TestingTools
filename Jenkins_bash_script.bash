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


#Execute script with arguments
python Exec_repo_Test.py    --work_dir "${WORK_DIR}/Agent_ws" \
                            --scope_folder ${AGENT_UROS_PATH} \
                            --url ${AGENT_REPO_LIST_URL} \
                            --file \
                            --Test_name "Agent test" \
                            --feature_to_test ${FEATURE_TO_TEST} \
                            --branch ${CLIENT_SET_BRANCHES}\
                            --Exec_command_after_build "install/uros_agent/lib/uros_agent/uros_agent udp 8888" \
                            --Ignore_package_result ${AGENT_IGNORE_PACKAGE_RESULT} \
                            --packages_to_test ${AGENT_PACKAGES_TO_TEST} \
                            --Test_extra_args \
                            --Build_extra_args
if  [ ! "$?" == "0" ]
then
    exit $?
fi


python Exec_repo_Test.py    --work_dir "${WORK_DIR}/Client_ws" \
                            --scope_folder ${CLIENT_UROS_PATH} \
                            --url ${CLIENT_REPO_LIST_URL} \
                            --file \
                            --Test_name "Client test" \
                            --feature_to_test ${FEATURE_TO_TEST} \
                            --branch ${CLIENT_SET_BRANCHES}\
                            --Exec_command_after_build \
                            --Ignore_package_result ${CLIENT_IGNORE_PACKAGE_RESULT} \
                            --packages_to_test ${CLIENT_PACKAGES_TO_TEST} \
                            --Test_extra_args \
                            --Build_extra_args "--cmake-args -DBUILD_SHARED_LIBS=${CLIENT_BUILD_SHARED_LIBS}"
if  [ ! "$?" == "0" ]
then
    exit $?
fi
