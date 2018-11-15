import os
import sys
import shutil
import argparse
import ssl
import glob
import subprocess

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
		description='Test execution for uROS project.',
		formatter_class=argparse.ArgumentDefaultsHelpFormatter)
	parser.add_argument(
		'--build_static_libs',
		required=False,
		default=False,
		action='store_true',
		help='Build all as static libs.')
	parser.add_argument(
		'--branch',
		required=False,
		nargs=2,
		action='append',
		help='Specify testing branch (--branch package_path branch_to_set).')
	parser.add_argument(
		'--url',
		required=False,
		nargs=1,
		help='Specify the URL of the ros2.repos file.')
	parser.add_argument(
		'--file',
		required=False,
		nargs=1,
		help='Specify the path of the ros2.repos file.')
	parser.add_argument(
		'--feature_to_test',
		required=False,
		nargs=1,
		help='Specify the the feature to test.')
	parser.add_argument(
		'--package_to_test',
		required=False,
		nargs=1,
		action='append',
		help='Add package to be tested. If not defined, all packages will be tested.')
	parser.add_argument(
		'--scope_folder',
		required=False,
		nargs=1,
		help='folder where script will look for the repos.')
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
		'--Build_extra_args',
		required=False,
		action='append',
		help='Add extra args to the build process (remove starting --).')
	parser.add_argument(
		'--Test_extra_args',
		required=False,
		action='append',
		help='Add extra args to the test process (remove starting --).')
	parser.add_argument(
		'--Skip_cleaning_ws',
		required=False,
		default=False,
		action='store_true',
		help='Skip cleaning work space.')
	parser.add_argument(
		'--work_dir',
		required=False,
		nargs='+',
		help='Wokr directory.')


	args = parser.parse_args(argv)

	work_path=os.path.join(expanduser("~"), "ros2_ws")

	# Info
	print("\n\n\n")
	print("=====================================================")
	print("\tSet enviroment")
	print("=====================================================")
	print("\n")

	# Set work direcory
	if args.work_dir is not None:
		work_path=' '.join(args.work_dir)
		print ("work dir set to: %s" % work_path)


	# Generate / Clean workspace
	if not args.Skip_cleaning_ws and not args.skip_download:
		if os.path.exists(work_path):
			shutil.rmtree(work_path)

	if not os.path.exists(work_path):
		os.makedirs(work_path)
		print("Created folder: %s" % work_path)

	os.chdir(work_path)

	src_path=os.path.join(work_path, "src")
	if not os.path.exists(src_path):
		os.makedirs(src_path)
		print("Created folder: %s" % src_path)


	# Download repos
	if not args.skip_download:
		# Info
		print("\n\n\n")
		print("=====================================================")
		print("\tDownload repos")
		print("=====================================================")
		print("\n")
		
		if args.file is not None:
			repo_file=args.file[0]
		else:
			if args.url is not None:
				repo_file=os.path.join(work_path, "ros2.repos") 
				print ("Download file from: %s " % args.url[0])
				try:
					urllib.request.urlretrieve(args.url[0], repo_file)
				except:
					urllib.urlretrieve (args.url[0], repo_file)
			else:
				print ("No repo file was specify")
				return -1

		command="vcs-import " + "src" + " < " + "\"" + repo_file + "\""
		print (command)
		if os.system(command)  != 0:
			print ("Download error")
			return -1
	
	
	# Get all repos folder
	scope_folder=src_path
	if args.scope_folder is not None:
		scope_folder=os.path.join(src_path, scope_folder) 
	repo_list=[]
	for root, dirs, files in os.walk(scope_folder):
		for dir in dirs:
			if dir == ".git":
				repo_list.append(root)


	# Change to feature in all repos
	if args.feature_to_test is not None:
		# Info
		print("\n\n\n")
		print("=====================================================")
		print("\tSet feature")
		print("=====================================================")
		print("\n")


		for repo in repo_list:
			command="git -C " + "\"" + repo + "\"" + " checkout " + args.feature_to_test[0]
			git_exe = subprocess.Popen(["git", "-C", "%s" % repo, "checkout", "%s" % args.feature_to_test[0]],stdout=subprocess.PIPE, stderr=subprocess.PIPE)
			while git_exe.poll() == None:
				git_exe.stdout.readline()
			if git_exe.returncode == 0:
				print(command)
				print("%s" % repo)
				print(git_exe.stderr.readline())
			
			#print (command)
			#if os.system(command)  == 0:
			#	print ("Switched " + "\"" + repo + "\"" + " to " + args.feature_to_test[0])
	
	# Change requested branches 
	if args.branch is not None:
		# Info
		print("\n\n\n")
		print("=====================================================")
		print("\tChange branges")
		print("=====================================================")
		print("\n")

		for value in args.branch:
			if value is not None:
				command="git -C " + "\"" + os.path.join(src_path, value[0]) + "\"" + " checkout " + value[1]
				print (command)
				if os.system(command)  != 0:
					print ("Switch branch error")
					return -1


	# Build
	if not args.skip_build:
		# Info
		print("\n\n\n")
		print("=====================================================")
		print("\tBuild")
		print("=====================================================")
		print("\n")

		colcon_args=""
		if args.Build_extra_args is not None:
			for value in args.Build_extra_args:
				if value is not None:
					colcon_args += "--" + value + " "
		shared="ON"
		if args.build_static_libs:
			shared="OFF"
		colcon_args +="--cmake-args -DBUILD_SHARED_LIBS=" + shared + " "

		command="colcon build " + colcon_args
		print (command)
		if os.system(command)  != 0:
			print ("Build error")
			return -1


	# Genetare test string
	if not args.skip_test:
		# Info
		print("\n\n\n")
		print("=====================================================")
		print("\tTest")
		print("=====================================================")
		print("\n")

		test_args = ""
		if args.Test_extra_args is not None:
			for value in args.Test_extra_args:
				if value is not None:
					test_args += " --" + value
		
		test_packages= ""
		if args.package_to_test is not None:
			for value in args.package_to_test:
				if value is not None:
					test_packages += value[0] + " "

		# Execute tests
		command="colcon test"
		if not test_packages == "":
			command+=" --packages-select " + test_packages
		if not test_args == "":
			command+=" " + test_args
		print(command)
		if os.system(command)  != 0:
			print ("Run test error")
			return -1

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
					if not log_report == "":
						error_string="Error in test"
						print("\n\n\n")
						print("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~")
						print("\tTest error in: %s" % log_package)
						print("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~")
						print("\n")
						print("test report:")
						with open(os.path.join(log_package_dir, "stdout.log")) as full_report:
							print(full_report.read())

	# Return success
	if error_string==0:
		print("Success!")
		return 0
	else:
		print(error_string)
		return -1


if __name__ == '__main__':
	sys.exit(main())
