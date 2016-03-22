#!/bin/bash          

#this file runs all of the style tests for the alice subdirectory
#this file will be run by the travis build system every time that 
#new code is pushed
export BUGGY_ROOT=`pwd`
export GRADLE_HOME=$BUGGY_ROOT/gradle-2.7
export GRADLE_BIN=$GRADLE_HOME/bin

#sets init value if it is set to false then the script will fail
true

# for viewing the find bugs error file
cat real_time/surface_src/java_src/Alice/build/reports/findbugs/main.xml
# for counting the number of lines of java code that we have in alice
find real_time/surface_src/java_src/Alice/ -name '*.java' | xargs wc -l
# for ensuring that the only println( is in logic error aka the console is not cluttered with debug info
PRINT_LINE=$(grep -r 'System.out.println(' real_time/surface_src/java_src/Alice/src/main)
PRINT_NUM=$((grep -r 'System.out.println(' real_time/surface_src/java_src/Alice/src/main) | wc -l)
if [ $PRINT_NUM != 1 ] 
then
    echo $PRINT_NUM
    echo $PRINT_LINE
    echo "Detected System.out.println() in Alice";
    false
 fi
