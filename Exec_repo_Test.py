import os
import sys
import shutil
import argparse
import ssl
import glob
import subprocess
import time

try:
    import urllib.request
except:
    import urllib

from os.path import expanduser


def main(argv=sys.argv[1:]):

    ssl._create_default_https_context = ssl._create_unverified_context
    error_string=""
    
    # Configure parser
    parser = argparse.ArgumentParser(
        description='Testing script for MicroROS Agent-side.',
        formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument(
        '--Test_name',
        required=False,
        nargs='*',
        default = "",
        help='Test name.')
    parser.add_argument(
        '--work_dir',
        required=False,
        nargs='*',
        default = "",
        help='Work directory.')
    parser.add_argument(
        '--branch',
        required=False,
        nargs='*',
        action='append',
        help='Specify testing branch (--branch package1:branch_to_set1 package1:branch_to_set2 ...).')
    parser.add_argument(
        '--url',
        required=False,
        nargs='*',
        default="",
        help='Specify the URL of the repos file.')
    parser.add_argument(
        '--file',
        required=False,
        nargs='*',
        default="",
        help='Specify the path of the repos file. Specifies this field will invalidate the url field.')
    parser.add_argument(
        '--feature_to_test',
        required=False,
        nargs='*',
        action='append',
        help='Specify the the feature to test.')
    parser.add_argument(
        '--packages_to_test',
        required=False,
        nargs='*',
        action='append',
        help='Add packages to be tested. If not defined, all packages will be tested.')
    parser.add_argument(
        '--Ignore_package_result',
        required=False,
        nargs='*',
        action='append',
        help='Ignore de packages result.')
    parser.add_argument(
        '--scope_folder',
        required=False,
        nargs='*',
        default="",
        help='Folder where script will look for the repos.')
    parser.add_argument(
        '--skip_build',
        required=False,
        default=False,
        action='store_true',
        help='Set to skip build step.')
    parser.add_argument(
        '--skip_test',
        required=False,
        default=False,
        action='store_true',
        help='Set to skip test step.')
    parser.add_argument(
        '--skip_download',
        required=False,
        default=False,
        action='store_true',
        help='Set to skip download step.')
    parser.add_argument(
        '--Skip_cleaning_ws',
        required=False,
        default=False,
        action='store_true',
        help='Skip cleaning work space.')
    parser.add_argument(
        '--Build_extra_args',
        required=False,
        nargs='*',
        action='append',
        help='Add extra args to the build process.')
    parser.add_argument(
        '--Test_extra_args',
        required=False,
        nargs='*',
        action='append',
        help='Add extra args to the test process.')
    parser.add_argument(
        '--Exec_command_after_build',
        required=False,
        nargs='*',
        action='append',
        help='Execute command after build. Execution path will be the workspace directory')
    parser.add_argument(
        '--Exec_command_after_test',
        required=False,
        nargs='*',
        action='append',
        help='Execute command after test. Execution path will be the workspace directory.')
    parser.add_argument(
        '--Exec_wait_time',
        required=False,
        nargs=1,
        default=5,
        help='Wait time after exe comand.')
    args = parser.parse_args(argv)


    # Set test name
    Test_name = ' '.join(args.Test_name)
    if Test_name != "":
        sys.stdout.write("# BEGIN SECTION: TEST " + Test_name + "\n\n")
    else:
        sys.stdout.write("# BEGIN SECTION: TEST\n\n")
    sys.stdout.flush()


    # Parse args
    sys.stdout.write("# BEGIN SECTION: Parse args\n\n")
    sys.stdout.flush()
    sys.stdout.write('\n'.join(argv)  + "\n")
    sys.stdout.flush()
    sys.stdout.write("# END SECTION\n\n\n")
    sys.stdout.flush()


    # Set enviroment
    sys.stdout.write("# BEGIN SECTION: Set enviroment\n\n")
    sys.stdout.flush()
    
    work_path=os.path.join(expanduser("~"), "ros2_ws")

    # Set work direcory
    work_path=' '.join(args.work_dir)
    if work_path != "":
        sys.stdout.write("work dir set to: %s\n" % work_path)
        sys.stdout.flush()

    # Generate / Clean workspace
    if not args.Skip_cleaning_ws and not args.skip_download:
        if os.path.exists(work_path):
            shutil.rmtree(work_path)

    if not os.path.exists(work_path):
        os.makedirs(work_path)
        sys.stdout.write("Created folder: %s\n" % work_path)
        sys.stdout.flush()

    # Change working path
    os.chdir(work_path)

    # Create source path
    src_path=os.path.join(work_path, "src")
    if not os.path.exists(src_path):
        os.makedirs(src_path)
        sys.stdout.write("Created folder: %s\n" % src_path)
        sys.stdout.flush()

    # End section
    sys.stdout.write("# END SECTION\n\n\n")
    sys.stdout.flush()


    # Download repos
    sys.stdout.write("# BEGIN SECTION: Download repos\n\n")
    sys.stdout.flush()

    if not args.skip_download:
        repo_file=' '.join(args.file)

        if repo_file != "":
            if not os.path.isfile(repo_file):
                sys.stderr.write("Given repo file do not exists\n")
                sys.stderr.flush()
                return -1
        else:
            if len(args.url) > 0:
                repo_file=os.path.join(work_path, "ros2.repos") 
                sys.stdout.write("Download file from: %s\n" % args.url[0])
                sys.stdout.flush()
                try:
                    urllib.request.urlretrieve(args.url[0], repo_file)
                except:
                    urllib.urlretrieve (args.url[0], repo_file)
            else:
                sys.stderr.write("No repo file was specify\n")
                sys.stderr.flush()
                return -1

        command="vcs-import " + "src" + " < " + "\"" + repo_file + "\""
        sys.stdout.write(command + "\n")
        sys.stdout.flush()
        p = subprocess.Popen(command, shell=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
        while p.poll() is None:
            sys.stdout.write(p.stdout.read(1))
            sys.stdout.flush()
        sys.stdout.write(p.stdout.read())
        sys.stdout.flush()
        if p.returncode != 0:
            sys.stderr.write("Download error\n")
            sys.stderr.flush()
            return -1

    else:
        sys.stdout.write("Skipped\n")
        sys.stdout.flush()

    # End section
    sys.stdout.write("# END SECTION\n\n\n")
    sys.stdout.flush()


    # Get all repos folder
    sys.stdout.write("# BEGIN SECTION: Get all repo paths\n\n")
    sys.stdout.flush()
    repo_count = 0
    scope_folder=os.path.join(src_path, ' '.join(args.scope_folder)) 

    repo_list=[]
    for root, dirs, files in os.walk(scope_folder):
        for dir in dirs:
            if dir == ".git":
                sys.stdout.write(root + "\n")
                sys.stdout.flush()
                repo_count += 1
                repo_list.append(root)

    sys.stdout.write("Found %i repos paths\n" % repo_count)
    sys.stdout.flush()

    # End section
    sys.stdout.write("# END SECTION\n\n\n")
    sys.stdout.flush()


    # Change to feature in all repos
    sys.stdout.write("# BEGIN SECTION: Set feature\n\n")
    sys.stdout.flush()
    if args.feature_to_test is not None and len(args.feature_to_test) > 0:
        for feature_group in args.feature_to_test:
            for feature in feature_group:
                repo_changed=0
                for repo in repo_list:
                    command="git -C " + "\"" + repo + "\"" + " checkout " + feature
                    p = subprocess.Popen(command, shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
                    out, err = p.communicate()
                    if p.returncode == 0:
                        sys.stdout.write(command + "\n")
                        sys.stdout.flush()
                        sys.stdout.write("`stdout:`\n" + out + "\n")
                        sys.stdout.flush()
                        sys.stdout.write("`stderr:`\n" + err + "\n")
                        sys.stdout.flush()
                        repo_changed += 1
                sys.stdout.write("%i of %i repos swithed to %s \n" % (repo_changed, repo_count, feature))
                sys.stdout.flush()

    else:
        sys.stdout.write("No feature to be set\n")
        sys.stdout.flush()

    # End section
    sys.stdout.write("# END SECTION\n\n\n")
    sys.stdout.flush()


    # Change requested branches 
    sys.stdout.write("# BEGIN SECTION: Change branches\n\n")
    sys.stdout.flush()
    if args.branch is not None and len(args.branch) > 0:
        for branch_group in args.branch:
            for branch in branch_group:
                value=branch.split(":")
                
                tmp_repo_path=""
                for repo_path in repo_list:
                    if value[0] == os.path.basename(repo_path):
                        tmp_repo_path = repo_path

                if tmp_repo_path != "":
                    command="git -C " + "\"" + repo_path + "\"" + " checkout " + value[1]
                    sys.stdout.write(command + "\n")
                    sys.stdout.flush()
                    p = subprocess.Popen(command, shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
                    out, err = p.communicate()
                    sys.stdout.write("`stdout:`\n" + out + "\n")
                    sys.stdout.flush()
                    sys.stdout.write("`stderr:`\n" + err + "\n")
                    sys.stdout.flush()
                    if p.returncode != 0:
                        sys.stderr.write("Switch branch error\n")
                        sys.stderr.flush()
                        return -1
                else:
                    sys.stderr.write("Repo '%s' not found" % value[0] + "\n")
                    sys.stderr.flush()
                    return -1
    else:
        sys.stdout.write("No branches to be set\n")
        sys.stdout.flush()

    # End section
    sys.stdout.write("# END SECTION")
    sys.stdout.flush()
    sys.stdout.write("\n\n")


    # Build
    sys.stdout.write("# BEGIN SECTION: Colcon build\n")
    sys.stdout.flush()
    if not args.skip_build:
        Build_extra_args =[]
        if args.Build_extra_args is not None:
            for Build_extra_args_group in args.Build_extra_args:
                for value in Build_extra_args_group:
                    Build_extra_args.append(value)
        command="colcon build " + ' '.join(Build_extra_args)
        sys.stdout.write(command + "\n")
        sys.stdout.flush()
        p = subprocess.Popen(command, shell=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
        while p.poll() is None:
            sys.stdout.write(p.stdout.read(1))
            sys.stdout.flush()
        sys.stdout.write(p.stdout.read())
        sys.stdout.flush()

        if p.returncode != 0:
            sys.stderr.write("Build error\n")
            sys.stderr.flush()
            return -1
    else:
        sys.stdout.write("Skipped\n")
        sys.stdout.flush()

    # End section
    sys.stdout.write("# END SECTION\n\n\n")
    sys.stdout.flush()


    # Check locak setup file
    sys.stdout.write("# BEGIN SECTION: Check local setup file\n")
    sys.stdout.flush()

    # Display platform
    sys.stdout.write("Platform: " + sys.platform + "\n")
    sys.stdout.write("Type: " + os.name + "\n")
    sys.stdout.flush()

    # Configure path
    local_setup_file=os.path.join(os.path.join(work_path, "install"), "local_setup")
    local_setup_command = ""
    if sys.platform.startswith('linux'):
        sys.stdout.write("Linux\n")
        sys.stdout.flush()
        local_setup_file += ".bash"
        local_setup_command = ". " + local_setup_file
        command_exec="bash"
        command_exit="exit"
    elif sys.platform.startswith("win"):
        sys.stdout.write("Windows\n")
        sys.stdout.flush()
        local_setup_file += ".bat"
        local_setup_command = local_setup_file
        command_exec="cmd"
        command_exit="exit"
    else:
        sys.stderr.write("Platform not supported\n")
        sys.stderr.flush()
        return -1

    # Check if exist
    if os.path.isfile(local_setup_file):
        sys.stdout.write("Setup file found in:" + local_setup_file + "\n")
        sys.stdout.flush()
    else:
        sys.stderr.write("Setup file not found in: " + local_setup_file + "\n")
        sys.stderr.flush()
        #return -1

    # End section
    sys.stdout.write("# END SECTION\n\n\n")
    sys.stdout.flush()


    # Execute commands
    sys.stdout.write("# BEGIN SECTION: Execute command in configured shell\n")
    sys.stdout.flush()
    if args.Exec_command_after_build is not None and len(args.Exec_command_after_build) > 0:
        for command in args.Exec_command_after_build:
            if command is not None:
                command =  ' '.join(command)
                sys.stdout.write("# BEGIN SECTION: Exec: " + command + "\n")
                sys.stdout.flush()

                # Open terminal
                p = subprocess.Popen(command_exec, shell=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, stdin=subprocess.PIPE)
                
                # Configure enviroment variables
                p.stdin.write(local_setup_command + "\n")

                # Execute user command
                sys.stdout.write(command + "\n")
                sys.stdout.flush()
                p.stdin.write(command + "\n")

                # Close terminal
                p.stdin.write(command_exit + "\n")

                # Wait
                time.sleep(args.Exec_wait_time)
                if p.poll() is None:
                    sys.stdout.write("Command still running after %i seconds\n\n" % args.Exec_wait_time)
                    sys.stdout.flush()
                else:
                    sys.stdout.write("Command output:\n")
                    sys.stdout.write(p.stdout.read() + "\n\n")
                    sys.stdout.write("Command terminated after %i seconds\n" % args.Exec_wait_time)
                    sys.stdout.write("Return code: %i\n\n" % p.returncode)
                    sys.stdout.flush()

                # End section
                sys.stdout.write("# END SECTION\n\n\n")
                sys.stdout.flush()

                # Change working path
                os.chdir(work_path)
    else:
        sys.stdout.write("No commands to execute\n")
        sys.stdout.flush()


    # End section
    sys.stdout.write("# END SECTION\n\n\n")
    sys.stdout.flush()



    # Genetare test string
    sys.stdout.write("# BEGIN SECTION: Colcon test\n")
    if not args.skip_test:

        # Execute tests
        command="colcon test"

        packages_to_test = ""
        if args.packages_to_test is not None:
            for packages_to_test_group in args.packages_to_test:
                packages_to_test += " " + ' '.join(packages_to_test_group)
            if packages_to_test != "":
                command+=" --packages-select " + packages_to_test

        Test_extra_args =[]
        if args.Test_extra_args is not None:
            for Test_extra_args_group in args.Test_extra_args:
                for value in Test_extra_args_group:
                    Test_extra_args.append(value)

        command+=" " + ' '.join(Test_extra_args)
        
        sys.stdout.write(command + "\n")
        sys.stdout.flush()

        p = subprocess.Popen(command, shell=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
        while p.poll() is None:
            sys.stdout.write(p.stdout.read(1))
            sys.stdout.flush()
        sys.stdout.write(p.stdout.read())
        sys.stdout.flush()


        # Execute report
        sys.stdout.write("# BEGIN SECTION: General report\n")
        sys.stdout.flush()
        command="colcon test-result"
        p = subprocess.Popen(command, shell=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
        while p.poll() is None:
            sys.stdout.write(p.stdout.read(1))
            sys.stdout.flush()
        sys.stdout.write(p.stdout.read())
        sys.stdout.flush()
        # End section
        sys.stdout.write("# END SECTION\n\n\n")
        sys.stdout.flush()


        # Convert list
        Ignore_package_result = []
        if args.Ignore_package_result is not None:
            for Ignore_group in args.Ignore_package_result:
                for Ignore in Ignore_group:
                    Ignore_package_result.append(Ignore)


        # Find log folder
        logs_path=os.path.join(work_path, "log")
        logs_pattern = os.path.join(logs_path, 'test_*')
        sorted_logs = sorted(
            glob.iglob(logs_pattern), key=os.path.getctime, reverse=True) 
        last_log_path = os.path.join(logs_path, sorted_logs[0])
        for log_package in os.listdir(last_log_path):
            log_package_dir=os.path.join(last_log_path,log_package)
            if os.path.isdir(log_package_dir):
                with open(os.path.join(log_package_dir, "stderr.log")) as f:
                    log_report=f.read()
                    if log_report != "":
                        if log_package not in Ignore_package_result:
                            error_string+="Error in test: " + log_package + "\n"
                        sys.stdout.write("# BEGIN SECTION: Test error in: %s\n" % log_package)
                        sys.stdout.flush()
                        sys.stdout.write("test report:\n")
                        sys.stdout.flush()
                        with open(os.path.join(log_package_dir, "stdout.log")) as full_report:
                            sys.stdout.write(full_report.read())
                            sys.stdout.flush()

                        # End section
                        sys.stdout.write("# END SECTION\n\n\n")
                        sys.stdout.flush()


    # End section
    sys.stdout.write("# END SECTION\n\n\n")
    sys.stdout.flush()


    # Execute commands
    sys.stdout.write("# BEGIN SECTION: Execute command in configured shell\n")
    sys.stdout.flush()
    if args.Exec_command_after_test is not None and len(args.Exec_command_after_test) > 0:
        for command in args.Exec_command_after_test:
            if command is not None:
                command =  ' '.join(command)
                sys.stdout.write("# BEGIN SECTION: Exec: " + command + "\n")
                sys.stdout.flush()

                # Open terminal
                p = subprocess.Popen(command_exec, shell=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, stdin=subprocess.PIPE)
                
                # Configure enviroment variables
                p.stdin.write(local_setup_command + "\n")

                # Execute user command
                sys.stdout.write(command + "\n")
                sys.stdout.flush()
                p.stdin.write(command + "\n")

                # Close terminal
                p.stdin.write(command_exit + "\n")

                # Wait
                time.sleep(args.Exec_wait_time)
                if p.poll() is None:
                    sys.stdout.write("Command still running after %i seconds\n\n" % args.Exec_wait_time)
                    sys.stdout.flush()
                else:
                    sys.stdout.write("Command output:\n")
                    sys.stdout.write(p.stdout.read() + "\n\n")
                    sys.stdout.write("Command terminated after %i seconds\n" % args.Exec_wait_time)
                    sys.stdout.write("Return code: %i\n\n" % p.returncode)
                    sys.stdout.flush()

                # End section
                sys.stdout.write("# END SECTION\n\n\n")
                sys.stdout.flush()

                # Change working path
                os.chdir(work_path)
    else:
        sys.stdout.write("No commands to execute\n")
        sys.stdout.flush()


    # End section
    sys.stdout.write("# END SECTION\n\n\n")
    sys.stdout.flush()


    # Return success
    if error_string=="":
        sys.stdout.write("Success!\n")
        sys.stdout.flush()
        return 0
    else:
        sys.stdout.write(error_string + "\n")
        sys.stdout.flush()
        return -1


if __name__ == '__main__':
    sys.exit(main())
