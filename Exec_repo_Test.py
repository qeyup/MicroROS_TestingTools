import os
import sys
import shutil
import argparse
import ssl
import glob
import subprocess
import time
import tarfile

try:
    import urllib.request
except:
    import urllib

from os.path import expanduser


class custom_output:
    discard_output_chars = ""
    new_output_line = ""
    last_output_line = ""

    @staticmethod
    def std_print(out_string):
        for c in out_string:
            custom_output.new_output_line+= c

            # To avoid duplicated lines
            if not custom_output.last_output_line.startswith(custom_output.new_output_line) or custom_output.new_output_line == "\n":
                sys.stdout.write(custom_output.discard_output_chars+c)
                sys.stdout.flush()
                custom_output.discard_output_chars=""
            else:
                custom_output.discard_output_chars+=c 

            # New line
            if c == '\n':
                custom_output.last_output_line=custom_output.new_output_line
                custom_output.discard_output_chars=""
                custom_output.new_output_line=""



class tags_generator:
    def __init__(self):
        self.tag_count = []

    def start(self, title):

        if len(self.tag_count) > 0:
            self.tag_count[-1] += 1

        tag_number=""
        for index in self.tag_count:
            tag_number += "%i." % index
        if tag_number != "":
            tag_number += "- "

        self.tag_count.append(0)

        out="# BEGIN SECTION: " + tag_number + title + "\n\n"
        custom_output.std_print(out)

    def end(self):
        out = "# END SECTION\n\n\n"
        custom_output.std_print(out) 

        if len(self.tag_count) > 0:
            self.tag_count.pop()


def main(argv=sys.argv[1:]):

    tags_gen = tags_generator()
    ok_string = "success"

    ssl._create_default_https_context = ssl._create_unverified_context

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
    parser.add_argument(
        '--Compress_log',
        required=False,
        default=False,
        action='store_true',
        help='Indicate if compress log folder.')
    args = parser.parse_args(argv)


    # Set test name
    Test_name = ' '.join(args.Test_name)
    if Test_name != "":
        tags_gen.start(Test_name)
    else:
        tags_gen.start("Test")


    # Parse args
    tags_gen.start("Parse args")
    custom_output.std_print('\n'.join(argv)  + "\n")
    tags_gen.end()



    # Set enviroment
    tags_gen.start("Set enviroment")
    work_path=os.path.join(expanduser("~"), "ros2_ws")

    # Set work direcory
    work_path=' '.join(args.work_dir)
    if work_path != "":
        custom_output.std_print("work dir set to: %s\n" % work_path)

    # Generate / Clean workspace
    if not args.Skip_cleaning_ws and not args.skip_download:
        if os.path.exists(work_path):
            shutil.rmtree(work_path)

    if not os.path.exists(work_path):
        os.makedirs(work_path)
        custom_output.std_print("Created folder: %s\n" % work_path)

    # Change working path
    os.chdir(work_path)

    # Create source path
    src_path=os.path.join(work_path, "src")
    if not os.path.exists(src_path):
        os.makedirs(src_path)
        custom_output.std_print("Created folder: %s\n" % src_path)

    # End section
    tags_gen.end()


    # Process
    def execute_test():

        # Download repos
        tags_gen.start("Download repos")

        if not args.skip_download:
            repo_file=' '.join(args.file)

            if repo_file != "":
                if not os.path.isfile(repo_file):
                    #sys.stderr.write("Given repo file do not exists\n")
                    #sys.stderr.flush()
                    return "Given repo file do not exists"
            else:
                if len(args.url) > 0:
                    repo_file=os.path.join(work_path, "ros2.repos") 
                    custom_output.std_print("Download file from: %s\n" % args.url[0])
                    
                    try:
                        urllib.request.urlretrieve(args.url[0], repo_file)
                    except:
                        urllib.urlretrieve (args.url[0], repo_file)
                else:
                    #sys.stderr.write("No repo file was specify\n")
                    #sys.stderr.flush()
                    return "No repo file was specify"

            command="vcs-import " + "src" + " < " + "\"" + repo_file + "\""
            custom_output.std_print(command + "\n")
            try:
                p = subprocess.Popen(command, shell=True, encoding='ascii', stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
            except:
                p = subprocess.Popen(command, shell=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
            while p.poll() is None:
                custom_output.std_print(p.stdout.read(1))
            custom_output.std_print(p.stdout.read())
            if p.returncode != 0:
                #sys.stderr.write("Download error\n")
                #sys.stderr.flush()
                return "Download error"

        else:
            custom_output.std_print("Skipped\n")

        # End section
        tags_gen.end()


        # Get all repos folder
        tags_gen.start("Get all repo path")
        
        repo_count = 0
        scope_folder=os.path.join(src_path, ' '.join(args.scope_folder)) 

        repo_list=[]
        for root, dirs, files in os.walk(scope_folder):
            for dir in dirs:
                if dir == ".git":
                    custom_output.std_print(root + "\n")
                    repo_count += 1
                    repo_list.append(root)

        custom_output.std_print("Found %i repos paths\n" % repo_count)

        # End section
        tags_gen.end()


        # Change to feature in all repos
        tags_gen.start("Set feature")
        repo_changed=0
        if args.feature_to_test is not None and len(args.feature_to_test) > 0:
            for feature_group in args.feature_to_test:
                feature_list = ' '.join(feature_group)
                for feature in feature_list.split():
                    for repo in repo_list:
                        command="git -C " + "\"" + repo + "\"" + " checkout " + feature
                        try:
                            p = subprocess.Popen(command, shell=True, encoding='ascii', stdout=subprocess.PIPE, stderr=subprocess.PIPE)
                        except:
                            p = subprocess.Popen(command, shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
                        out, err = p.communicate()
                        if p.returncode == 0:
                            custom_output.std_print(command + "\n")
                            custom_output.std_print("`stdout:`\n" + out + "\n")
                            custom_output.std_print("`stderr:`\n" + err + "\n")
                            repo_changed += 1
            custom_output.std_print("%i of %i repos swithed to %s \n" % (repo_changed, repo_count, feature))

        else:
            custom_output.std_print("No feature to be set\n")


        # End section
        tags_gen.end()


        # Change requested branches 
        tags_gen.start("Change branches")
        if args.branch is not None and len(args.branch) > 0:
            for branch_group in args.branch:
                branch_group_list = ' '.join(branch_group)
                for branch in branch_group_list.split():
                    value=branch.split(":")

                    tmp_repo_path=""
                    for repo_path in repo_list:
                        if value[0] == os.path.basename(repo_path):
                            tmp_repo_path = repo_path
                            break

                    if tmp_repo_path != "":
                        command="git -C " + "\"" + repo_path + "\"" + " checkout " + value[1]
                        custom_output.std_print(command + "\n")
                        
                        try:
                            p = subprocess.Popen(command, shell=True, encoding='ascii', stdout=subprocess.PIPE, stderr=subprocess.PIPE)
                        except:
                            p = subprocess.Popen(command, shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
                        out, err = p.communicate()
                        custom_output.std_print("`stdout:`\n" + out + "\n")
                        custom_output.std_print("`stderr:`\n" + err + "\n")

                        if p.returncode != 0:
                            #sys.stderr.write("Switch branch error\n")
                            #sys.stderr.flush()
                            return "Switch branch error"
                    else:
                        #sys.stderr.write("Repo '%s' not found" % value[0] + "\n")
                        #sys.stderr.flush()
                        return "Repo '%s' not found" % value[0]
        else:
            custom_output.std_print("No branches to be set\n")

        # End section
        tags_gen.end()


        # Build
        tags_gen.start("Colcon build")
        
        if not args.skip_build:
            Build_extra_args =[]
            if args.Build_extra_args is not None:
                for Build_extra_args_group in args.Build_extra_args:
                    for value in Build_extra_args_group:
                        if value == "":
                            continue
                        Build_extra_args.append(value.replace("#", ""))
            command="colcon build " + ' '.join(Build_extra_args)
            custom_output.std_print(command + "\n")

            try:
                p = subprocess.Popen(command, shell=True, encoding='ascii', stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
            except:
                p = subprocess.Popen(command, shell=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
            while p.poll() is None:
                custom_output.std_print(p.stdout.read(1))
            custom_output.std_print(p.stdout.read())

            if p.returncode != 0:
                #sys.stderr.write("Build error\n")
                #sys.stderr.flush()
                return "Build error"
        else:
            custom_output.std_print("Skipped\n")

        # End section
        tags_gen.end()


        # Check locak setup file
        tags_gen.start("Check local setup file")

        # Display platform
        custom_output.std_print("Platform: " + sys.platform + "\n")
        custom_output.std_print("Type: " + os.name + "\n")

        # Configure path
        local_setup_file=os.path.join(os.path.join(work_path, "install"), "local_setup")
        local_setup_command = ""
        if sys.platform.startswith('linux'):
            custom_output.std_print("Linux\n")

            local_setup_file += ".bash"
            local_setup_command = ". " + local_setup_file
            command_exec="bash"
            command_exit="exit"
        elif sys.platform.startswith("win"):
            custom_output.std_print("Windows\n")

            local_setup_file += ".bat"
            local_setup_command = local_setup_file
            command_exec="cmd"
            command_exit="exit"
        else:
            #sys.stderr.write("Platform not supported\n")
            #sys.stderr.flush()
            return "Platform not supported"

        # Check if exist
        if os.path.isfile(local_setup_file):
            custom_output.std_print("Setup file found in:" + local_setup_file + "\n")
        else:
            sys.stderr.write("Setup file not found in: " + local_setup_file + "\n")
            sys.stderr.flush()
            #return "Setup file not found in: " + local_setup_file

        # End section
        tags_gen.end()
        


        # Execute commands
        tags_gen.start("Execute command in configured shell")
        
        if args.Exec_command_after_build is not None and len(args.Exec_command_after_build) > 0:
            for command in args.Exec_command_after_build:
                if command == "":
                    continue
                command =  ' '.join(command)
                tags_gen.start("Exec: " + command)

                # Open terminal
                try:
                    p = subprocess.Popen(command_exec, shell=True, encoding='ascii', stdout=subprocess.PIPE, stderr=subprocess.STDOUT, stdin=subprocess.PIPE)
                except:
                    p = subprocess.Popen(command_exec, shell=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, stdin=subprocess.PIPE)

                # Configure enviroment variables
                p.stdin.write(local_setup_command + "\n")
                p.stdin.flush()

                # Execute user command
                custom_output.std_print(command + "\n")
                p.stdin.write(command + "\n")
                p.stdin.flush()

                # Close terminal
                p.stdin.write(command_exit + "\n")
                p.stdin.flush()

                # Wait
                time.sleep(args.Exec_wait_time)
                if p.poll() is None:
                    custom_output.std_print("Command still running after %i seconds\n\n" % args.Exec_wait_time)
                else:
                    custom_output.std_print("Command output:\n")
                    custom_output.std_print(p.stdout.read() + "\n\n")
                    custom_output.std_print("Command terminated after %i seconds\n" % args.Exec_wait_time)
                    custom_output.std_print("Return code: %i\n\n" % p.returncode)

                # End section
                tags_gen.end()

                # Change working path
                os.chdir(work_path)
        else:
            custom_output.std_print("No commands to execute\n")


        # End section
        tags_gen.end()


        # Genetare test string
        tags_gen.start("Colcon test")
        if not args.skip_test:

            # Execute tests
            command="colcon test"

            if args.packages_to_test is not None:
                packages_to_test = ""
                for packages_to_test_group in args.packages_to_test:
                    packages_to_test += " " + ' '.join(packages_to_test_group)
                packages_to_test = ' '.join(packages_to_test.split())
                if packages_to_test != "" and packages_to_test != " ":
                    command+=" --packages-select " + packages_to_test

            Test_extra_args =[]
            if args.Test_extra_args is not None:
                for Test_extra_args_group in args.Test_extra_args:
                    for value in Test_extra_args_group:
                        Test_extra_args.append(value.replace("#",""))

            command+=" " + ' '.join(Test_extra_args)
            
            custom_output.std_print(command + "\n")

            try:
                p = subprocess.Popen(command, shell=True, encoding='ascii', stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
            except:
                p = subprocess.Popen(command, shell=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
            while p.poll() is None:
                custom_output.std_print(p.stdout.read(1))
            custom_output.std_print(p.stdout.read())


            if p.returncode != 0:
                #sys.stderr.write("Test error\n")
                #sys.stderr.flush()
                return "Test error"


            # Execute report
            tags_gen.start("General report")
            command="colcon test-result"
            try:
                p = subprocess.Popen(command, shell=True, encoding='ascii', stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
            except:
                p = subprocess.Popen(command, shell=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
            while p.poll() is None:
                custom_output.std_print(p.stdout.read(1))
            custom_output.std_print(p.stdout.read())

            # End section
            tags_gen.end()


            # Convert list
            #Ignore_package_result = []
            #if args.Ignore_package_result is not None:
            #    for Ignore_group in args.Ignore_package_result:
            #        Ignore_list = ' '.join(Ignore_group)
            #        for Ignore in Ignore_list.split():
            #            Ignore_package_result.append(Ignore)


            # Find log folder
            #logs_path=os.path.join(work_path, "log")
            #logs_pattern = os.path.join(logs_path, 'test_*')
            #sorted_logs = sorted(
            #    glob.iglob(logs_pattern), key=os.path.getctime, reverse=True) 
            #last_log_path = os.path.join(logs_path, sorted_logs[0])
            #for log_package in os.listdir(last_log_path):
            #    log_package_dir=os.path.join(last_log_path,log_package)
            #    if os.path.isdir(log_package_dir):
            #        with open(os.path.join(log_package_dir, "stderr.log")) as f:
            #            log_report=f.read()
            #            if log_report.find("Errors while running CTest") != -1:
            #                if log_package not in Ignore_package_result:
            #                    error_string+="Error in test: " + log_package + "\n"

            #            tags_gen.start("Test report (%s)" % log_package)
            #            custom_output.std_print("stderr.log:\n")
            #            custom_output.std_print(log_report + "\n\n")
            #            custom_output.std_print("stdout.log:\n")
            #            with open(os.path.join(log_package_dir, "stdout.log")) as full_report:
            #                custom_output.std_print(full_report.read() + "\n\n")
            #            tags_gen.end()

        # End section
        tags_gen.end()


        # Execute commands
        tags_gen.start("Execute command in configured shell")
        
        if args.Exec_command_after_test is not None and len(args.Exec_command_after_test) > 0:
            for command in args.Exec_command_after_test:
                if command == "":
                    continue
                command =  ' '.join(command)
                tags_gen.start("Exec: " + command)

                # Open terminal
                try:
                    p = subprocess.Popen(command_exec, shell=True, encoding='ascii', stdout=subprocess.PIPE, stderr=subprocess.STDOUT, stdin=subprocess.PIPE)
                except:
                    p = subprocess.Popen(command_exec, shell=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, stdin=subprocess.PIPE)

                # Configure enviroment variables
                p.stdin.write(local_setup_command + "\n")
                p.stdin.flush()

                # Execute user command
                custom_output.std_print(command + "\n")
                p.stdin.write(command + "\n")
                p.stdin.flush()

                # Close terminal
                p.stdin.write(command_exit + "\n")
                p.stdin.flush()

                # Wait
                time.sleep(args.Exec_wait_time)
                if p.poll() is None:
                    custom_output.std_print("Command still running after %i seconds\n\n" % args.Exec_wait_time)
                else:
                    custom_output.std_print("Command output:\n")
                    custom_output.std_print(p.stdout.read() + "\n\n")
                    custom_output.std_print("Command terminated after %i seconds\n" % args.Exec_wait_time)
                    custom_output.std_print("Return code: %i\n\n" % p.returncode)

                # End section
                tags_gen.end()


                # Change working path
                os.chdir(work_path)
        else:
            custom_output.std_print("No commands to execute\n")


        # End section
        tags_gen.end()

        # Test end tag
        tags_gen.end()

        return ok_string

    # Execute test
    str_rv = execute_test()

    # Returned val
    tags_gen.start("Execution result")
    custom_output.std_print("%s\n\n" % str_rv)
    tags_gen.end()


    # Compress log
    if args.Compress_log:
        tar = tarfile.open("log.tar.gz", "w:gz")
        tar.add("log", arcname="log")
        tar.close()
        shutil.rmtree("log")


    if (str_rv == ok_string):
        return 0
    else:
        return -1


if __name__ == '__main__':
    sys.exit(main())
