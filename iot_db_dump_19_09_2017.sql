-- MySQL dump 10.13  Distrib 5.7.19, for Linux (x86_64)
--
-- Host: localhost    Database: iot
-- ------------------------------------------------------
-- Server version	5.7.19-0ubuntu0.16.04.1

/*!40101 SET @OLD_CHARACTER_SET_CLIENT=@@CHARACTER_SET_CLIENT */;
/*!40101 SET @OLD_CHARACTER_SET_RESULTS=@@CHARACTER_SET_RESULTS */;
/*!40101 SET @OLD_COLLATION_CONNECTION=@@COLLATION_CONNECTION */;
/*!40101 SET NAMES utf8 */;
/*!40103 SET @OLD_TIME_ZONE=@@TIME_ZONE */;
/*!40103 SET TIME_ZONE='+00:00' */;
/*!40014 SET @OLD_UNIQUE_CHECKS=@@UNIQUE_CHECKS, UNIQUE_CHECKS=0 */;
/*!40014 SET @OLD_FOREIGN_KEY_CHECKS=@@FOREIGN_KEY_CHECKS, FOREIGN_KEY_CHECKS=0 */;
/*!40101 SET @OLD_SQL_MODE=@@SQL_MODE, SQL_MODE='NO_AUTO_VALUE_ON_ZERO' */;
/*!40111 SET @OLD_SQL_NOTES=@@SQL_NOTES, SQL_NOTES=0 */;

--
-- Table structure for table `tblEmotionalFeedback`
--

DROP TABLE IF EXISTS `tblEmotionalFeedback`;
/*!40101 SET @saved_cs_client     = @@character_set_client */;
/*!40101 SET character_set_client = utf8 */;
CREATE TABLE `tblEmotionalFeedback` (
  `feedbackID` int(11) NOT NULL AUTO_INCREMENT,
  `sessionID_fkey` int(11) NOT NULL,
  `is_fixed_feedback` tinyint(1) NOT NULL,
  `repetition` tinyint(3) unsigned NOT NULL,
  `face_to_display` varchar(256) NOT NULL,
  PRIMARY KEY (`feedbackID`),
  KEY `fkey_tblSession` (`sessionID_fkey`),
  CONSTRAINT `fkey_tblSession` FOREIGN KEY (`sessionID_fkey`) REFERENCES `tblSession` (`sessionID`)
) ENGINE=InnoDB DEFAULT CHARSET=latin1;
/*!40101 SET character_set_client = @saved_cs_client */;

--
-- Dumping data for table `tblEmotionalFeedback`
--

LOCK TABLES `tblEmotionalFeedback` WRITE;
/*!40000 ALTER TABLE `tblEmotionalFeedback` DISABLE KEYS */;
/*!40000 ALTER TABLE `tblEmotionalFeedback` ENABLE KEYS */;
UNLOCK TABLES;

--
-- Table structure for table `tblRepetitionResult`
--

DROP TABLE IF EXISTS `tblRepetitionResult`;
/*!40101 SET @saved_cs_client     = @@character_set_client */;
/*!40101 SET character_set_client = utf8 */;
CREATE TABLE `tblRepetitionResult` (
  `repetitionResultID` int(11) NOT NULL AUTO_INCREMENT,
  `sessionID_fkey` int(11) NOT NULL,
  `block_index` tinyint(3) unsigned NOT NULL,
  PRIMARY KEY (`repetitionResultID`),
  KEY `fkey_tblSession4` (`sessionID_fkey`),
  CONSTRAINT `fkey_tblSession4` FOREIGN KEY (`sessionID_fkey`) REFERENCES `tblSession` (`sessionID`)
) ENGINE=InnoDB DEFAULT CHARSET=latin1;
/*!40101 SET character_set_client = @saved_cs_client */;

--
-- Dumping data for table `tblRepetitionResult`
--

LOCK TABLES `tblRepetitionResult` WRITE;
/*!40000 ALTER TABLE `tblRepetitionResult` DISABLE KEYS */;
/*!40000 ALTER TABLE `tblRepetitionResult` ENABLE KEYS */;
UNLOCK TABLES;

--
-- Table structure for table `tblSession`
--

DROP TABLE IF EXISTS `tblSession`;
/*!40101 SET @saved_cs_client     = @@character_set_client */;
/*!40101 SET character_set_client = utf8 */;
CREATE TABLE `tblSession` (
  `sessionID` int(11) NOT NULL AUTO_INCREMENT,
  `repetitions` tinyint(3) unsigned NOT NULL,
  `blocks` tinyint(3) unsigned NOT NULL,
  `quantitative_feedback` tinyint(3) unsigned NOT NULL DEFAULT '0',
  `qualitative_feedback` tinyint(3) unsigned NOT NULL DEFAULT '0',
  `userID_fkey` varchar(38) NOT NULL,
  PRIMARY KEY (`sessionID`),
  KEY `fkey_tblUser` (`userID_fkey`),
  CONSTRAINT `fkey_tblUser` FOREIGN KEY (`userID_fkey`) REFERENCES `tblUser` (`userID`)
) ENGINE=InnoDB DEFAULT CHARSET=latin1;
/*!40101 SET character_set_client = @saved_cs_client */;

--
-- Dumping data for table `tblSession`
--

LOCK TABLES `tblSession` WRITE;
/*!40000 ALTER TABLE `tblSession` DISABLE KEYS */;
/*!40000 ALTER TABLE `tblSession` ENABLE KEYS */;
UNLOCK TABLES;

--
-- Table structure for table `tblTimeResult`
--

DROP TABLE IF EXISTS `tblTimeResult`;
/*!40101 SET @saved_cs_client     = @@character_set_client */;
/*!40101 SET character_set_client = utf8 */;
CREATE TABLE `tblTimeResult` (
  `timeResultID` int(11) NOT NULL AUTO_INCREMENT,
  `sessionID_fkey` int(11) NOT NULL,
  `repetition_index` tinyint(3) unsigned NOT NULL,
  `block_index` tinyint(3) unsigned NOT NULL,
  PRIMARY KEY (`timeResultID`),
  KEY `fkey_tblSession3` (`sessionID_fkey`),
  CONSTRAINT `fkey_tblSession3` FOREIGN KEY (`sessionID_fkey`) REFERENCES `tblSession` (`sessionID`)
) ENGINE=InnoDB DEFAULT CHARSET=latin1;
/*!40101 SET character_set_client = @saved_cs_client */;

--
-- Dumping data for table `tblTimeResult`
--

LOCK TABLES `tblTimeResult` WRITE;
/*!40000 ALTER TABLE `tblTimeResult` DISABLE KEYS */;
/*!40000 ALTER TABLE `tblTimeResult` ENABLE KEYS */;
UNLOCK TABLES;

--
-- Table structure for table `tblTrajSmoothResult`
--

DROP TABLE IF EXISTS `tblTrajSmoothResult`;
/*!40101 SET @saved_cs_client     = @@character_set_client */;
/*!40101 SET character_set_client = utf8 */;
CREATE TABLE `tblTrajSmoothResult` (
  `trajSmoothResultID` int(11) NOT NULL AUTO_INCREMENT,
  `sessionID_fkey` int(11) NOT NULL,
  `block_index` tinyint(3) unsigned NOT NULL,
  PRIMARY KEY (`trajSmoothResultID`),
  KEY `fkey_tblSession2` (`sessionID_fkey`),
  CONSTRAINT `fkey_tblSession2` FOREIGN KEY (`sessionID_fkey`) REFERENCES `tblSession` (`sessionID`)
) ENGINE=InnoDB DEFAULT CHARSET=latin1;
/*!40101 SET character_set_client = @saved_cs_client */;

--
-- Dumping data for table `tblTrajSmoothResult`
--

LOCK TABLES `tblTrajSmoothResult` WRITE;
/*!40000 ALTER TABLE `tblTrajSmoothResult` DISABLE KEYS */;
/*!40000 ALTER TABLE `tblTrajSmoothResult` ENABLE KEYS */;
UNLOCK TABLES;

--
-- Table structure for table `tblUser`
--

DROP TABLE IF EXISTS `tblUser`;
/*!40101 SET @saved_cs_client     = @@character_set_client */;
/*!40101 SET character_set_client = utf8 */;
CREATE TABLE `tblUser` (
  `userID` varchar(38) NOT NULL,
  `name_hash` varchar(256) NOT NULL,
  `surname_hash` varchar(256) NOT NULL,
  `pincode_hash` varchar(256) NOT NULL,
  `pincode_salt` varchar(16) NOT NULL,
  `birthdate_hash` varchar(256) NOT NULL,
  `picture` longblob,
  PRIMARY KEY (`userID`)
) ENGINE=InnoDB DEFAULT CHARSET=latin1;
/*!40101 SET character_set_client = @saved_cs_client */;

--
-- Dumping data for table `tblUser`
--

LOCK TABLES `tblUser` WRITE;
/*!40000 ALTER TABLE `tblUser` DISABLE KEYS */;
INSERT INTO `tblUser` VALUES ('3BEA00008131FE450031C573C0014000900077','Leandro','Gil','77E6A036870BC313D743E8BC40007FC3C76E9781A183CA15CC5CC6286B06C52D','testsalt','01/01/1988',NULL);
/*!40000 ALTER TABLE `tblUser` ENABLE KEYS */;
UNLOCK TABLES;
/*!40103 SET TIME_ZONE=@OLD_TIME_ZONE */;

/*!40101 SET SQL_MODE=@OLD_SQL_MODE */;
/*!40014 SET FOREIGN_KEY_CHECKS=@OLD_FOREIGN_KEY_CHECKS */;
/*!40014 SET UNIQUE_CHECKS=@OLD_UNIQUE_CHECKS */;
/*!40101 SET CHARACTER_SET_CLIENT=@OLD_CHARACTER_SET_CLIENT */;
/*!40101 SET CHARACTER_SET_RESULTS=@OLD_CHARACTER_SET_RESULTS */;
/*!40101 SET COLLATION_CONNECTION=@OLD_COLLATION_CONNECTION */;
/*!40111 SET SQL_NOTES=@OLD_SQL_NOTES */;

-- Dump completed on 2017-09-19 18:55:15
