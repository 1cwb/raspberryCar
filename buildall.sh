#!/bin/bash

echo "---------------------------------------------------------------------"
echo "welcom~~
you can use this bash build all project
you can use \"./build.sh\" or \"bash build.sh\" to build this project
if you want to clean ,you can run \"./build.sh clean\"
if you just want to build one of the projects, you can enter to the 
directory and run \"make/make clean\""
echo "---------------------------------------------------------------------"

TOP_PATH=$(cd `dirname $0`; pwd)
MAKE_PARAM="make"
ALL_MAKEFILE_PATH=""

do_make()
{
	for path in $ALL_MAKEFILE_PATH
	do
		cd $path
		#echo "cd $path"
		if [[ $MAKE_PARAM == "make" ]]
		then
			echo "make -j32"
			make -j32
		else if [[ $MAKE_PARAM == "clean" ]]
		then
			echo "make clean"
			make clean
		fi
		fi
	done
}

read_dir()
{
    for file in `ls -a $1`
    do
        if [ -d $1"/"$file ]
        then
            if [[ $file != '.' && $file != '..' ]]
            then
                read_dir $1"/"$file
            fi
        else
			if [ $file == "Makefile" ];then
				STR_TEMP=$1
				MAKEFILE_PATH=${STR_TEMP:1}
				BUILD_PATH=$TOP_PATH$MAKEFILE_PATH
				ALL_MAKEFILE_PATH=$ALL_MAKEFILE_PATH$BUILD_PATH" "
				#echo $ALL_MAKEFILE_PATH
			fi
        fi
    done
}

main_func()
{
	PARAM1=$1
	if [[ $PARAM1 == "make" ]]
	then
		MAKE_PARAM="make"
	else 
		if [[ $PARAM1 == "clean" ]]
		then
			MAKE_PARAM="clean"
		fi
	fi
	read_dir .
	do_make
}

main_func $1
