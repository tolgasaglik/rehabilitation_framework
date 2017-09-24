# QT Rehabilitation Framework #

This README currently only describes how to set up the QT RehaZenter rehabilitation framework and how to get it running.

### What is this repository for? ###

This repository contains my master thesis work for the University of Luxembourg, in collaboration with the RehaZenter facility in Kirchberg, Luxembourg. We have developed/are developing an exercise assistant that is capable of deploying and running an exercise session for heart stroke patients and elderly people on the QT Robot, completely autonomously.

### How do I get set up? ###

Before running the application, you need to download the *Qt4* and *PyQt4* packages, as well as the Python API for Mysqldb. Run the following commands to do so:
`sudo apt-get install libqt4-dev python-qt4 mysql-client mysql-server python-mysqldb`
After installing all of the above packages, we need to perform a little change to the MySQL configuration files. Log into your QT robot, open up the /etc/mysql/mysql.conf.d/mysqld.cnf file with your favourite text editor, and comment the following line out:
`bind-address = 127.0.0.1`
This line makes it so that the MySQL server only listens to connections coming from the robot itself and not from the outside, thus we need to comment this line if we want to make connections from the outside (such as a client computer). Restart the MySQL server to apply the changes:
`sudo service mysql restart`
Next, we will create a new MySQL user that has full permissions to access the database that the application is using. Login (locally) to your database by running the following command:
`mysql -u root -p`
A password prompt will appear after executing the command. Enter your password and, on the MySQL interpreter, run the following SQL command:
`CREATE USER 'qt_exercise_creator'@'localhost' IDENTIFIED BY 'qt_exercise_creator';`
You may choose an arbitrary password if you wish, but if you do so, then make sure you change the following line in the src/gui/QTRehaZenterGUI.py file to match your updated password (line 154):
`mysql_password = "[your_password]"`
Then, create a new database called "iot" and grant your newly-created user full access rights on that database:
`CREATE DATABASE iot;
GRANT ALL ON iot.* TO 'qt_exercise_creator'@'[IP_of_your_client_machine]';`
Make sure that both the robot and your client machine are located on the same network, and use your machine's IP address from that same network. The SQL command above will then allow you to remotely access the database through the user "qt\_exercise\_creator" that we created earlier. Finally, you can run the GUI application on your client machine by navigating to the reha\_game/src/gui folder and running the Python GUI using the following command:
`python QTRehaZenterGUI.py`
Make sure that roscore is running on the robot before executing the above command, as the GUI depends on it.

### Who do I talk to? ###

* gomesf.leandro@gmail.lu
