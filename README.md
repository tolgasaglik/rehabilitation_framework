# QT Rehabilitation Framework #

This README currently only describes how to set up the QT RehaZenter rehabilitation framework and how to get it running.

### What is this repository for? ###

This repository contains my master thesis work for the University of Luxembourg, in collaboration with the RehaZenter facility in Kirchberg, Luxembourg. We have developed/are developing an exercise assistant that is capable of deploying and running an exercise session for heart stroke patients and elderly people on the QT Robot, completely autonomously.

### How do I get set up? ###

Before running the application, you need to download the *Qt4* and *PyQt4* packages. Run the following commands to do so:
`sudo apt-get install libqt4-dev python-qt4`

You can run the GUI application using the following commands:
`cd reha_game/src/gui && python QTRehaZenterGUI.py`

If you wish to only run the *roslaunch* file without the GUI, simply run:

`roslaunch reha_game Exercise_Launcher`

By default, this exercise has been initialized with 10 repetitions and no time limit. You can change these parameters in the GUI itself by clicking on the "*File*" tab and then clicking on "*Preferences...*", or directly in the *roslaunch* call as arguments like so:

`roslaunch reha_game Exercise_Launcher number_of_repetitions:=20 time_limit:=600` (time limit in seconds)

Take a look at the above launch file if you want to check which other parameters you can set.

### Who do I talk to? ###

* leandro.gomes.001@student.uni.lu
