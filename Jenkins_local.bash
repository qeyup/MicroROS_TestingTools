#!/bin/bash

FEATURE_TO_TEST="develop"

AGENT_REPO_LIST_URL="https://raw.githubusercontent.com/microROS/micro-ROS-doc/master/repos/agent_minimum.repos"
AGENT_UROS_PATH="uros"
AGENT_PACKAGES_TO_TEST=""
AGENT_IGNORE_PACKAGE_RESULT="ament_tools osrf_pycommon ament_copyright ament_lint_cmake ament_package ament_pep257 rcl rcutils rosidl_generator_cpp rosidl_parser"
AGENT_SET_BRANCHES=""

CLIENT_REPO_LIST_URL="https://raw.githubusercontent.com/microROS/micro-ROS-doc/master/repos/client_minimum.repos"
CLIENT_UROS_PATH="uros"
CLIENT_PACKAGES_TO_TEST=""
CLIENT_IGNORE_PACKAGE_RESULT="ament_tools osrf_pycommon ament_copyright ament_lint_cmake ament_package ament_pep257 rcl_lifecycle rcl rclcpp rcutils rmw_microxrcedds rosidl_generator_cpp rosidl_parser"
CLIENT_SET_BRANCHES=""
CLIENT_BUILD_SHARED_LIBS="ON"


#----------------------------------------------------------------------------------------------------------------


# Download python test script
python_script="https://raw.githubusercontent.com/qeyup/MicroROS_TestingTools/feature/splitRepos/Exec_repo_Test.py"
wget ${python_script}  -O $HOME/Exec.py


#Execute script with arguments
python $HOME/Exec.py    --work_dir "$HOME/Agent_ws" \
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
Agent_test=$?

python $HOME/Exec.py    --work_dir "$HOME/Client_ws" \
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
Client_test=$?

echo "Agent result: $Agent_test"
echo "Client result: $Client_test"

if  [ ! "$Agent_test" == "0" ]
then
    exit -1
fi
if  [ ! "$Client_test" == "0" ]
then
    exit -1
fi
