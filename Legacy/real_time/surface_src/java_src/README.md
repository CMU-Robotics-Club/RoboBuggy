High Level Software Overview
============================

The High-level software stack is in charge telling buggy where to go.

TODO put more information here

Directory Structure
===================

Each logically separate project has a separate top-level directory.
Currently, this is only Alice and BuggyRos.

Building and Running
====================

To build:
    <Install Java JDK (NOT THE JRE) x86 Latest (greater than 1.8:60)>
    $> cd Alice
    $> gradlew build # Note that buggyros will get build automatically

    # for more information, see https://docs.gradle.org/current/userguide/java_plugin.html

To generate eclipse files:

    <Clone RoboBuggy into C:\workspace\RoboBuggy>
    <Install 32-bit Eclipse (greater than Mars) using eclipseinstaller>
    <Copy RxTx into local path?>
    $> gradle eclipse
    < DO NOT STOP READING THIS GUIDE >
    (launch Eclipse > set Workspace: Directory Above RoboBuggy repo )
    (File > Import > General > Existing Project into workspace > Browse to Robobuggy)
    (Confirm only Alice and BuggyRos are checked)

If the Eclipse build SEEMS broken:
    (go into eclipse > right click on each project > "Refresh" )

    (if that doesn't work, bring the big guns)
    $> gradlew cleanEclipse
    $> gradlew eclipse

    # for more information, see https://docs.gradle.org/current/userguide/eclipse_plugin.html

To install Vim-Editing in Eclipse:

    (if you don't learn vim or emacs, people in industry won't take you seriously)


To get a sweet .vimrc (and develop on windows):

    (you owe it to yourself because you're beautiful)
