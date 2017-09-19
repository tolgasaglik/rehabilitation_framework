# QT Rehabilitation Framework #

This README currently only describes how to set up the QT RehaZenter rehabilitation framework and how to get it running.

### What is this repository for? ###

This repository contains my master thesis work for the University of Luxembourg, in collaboration with the RehaZenter facility in Kirchberg, Luxembourg. We have developed/are developing an exercise assistant that is capable of deploying and running an exercise session for heart stroke patients and elderly people on the QT Robot, completely autonomously.

### How do I get set up? ###

Before running the application, you need to download the *Qt4* and *PyQt4* packages, as well as the Python API for Mysqldb. Run the following commands to do so:
`sudo apt-get install libqt4-dev python-qt4 python-mysqldb`

You can run the GUI application using the following commands:
`cd reha_game/src/gui && python QTRehaZenterGUI.py`

### Who do I talk to? ###

* gomesf.leandro@gmail.lu
