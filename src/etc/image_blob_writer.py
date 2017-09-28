#!/usr/bin/env python

import MySQLdb
import sys

db = MySQLdb.connect(user='root', passwd='ltwforthewin1992+-', host='localhost', db='iot')
user_id = "3B7F380000006A444E496510024C340113039000"
blob_value = open('./picture.jpg', 'rb').read()
sql = "update tblUser set picture=%s where userID=%s"    
print sql
cursor = db.cursor()
cursor.execute(sql,(blob_value, user_id))
db.commit()
cursor.close()
db.close()
print("Blob written to database.")
