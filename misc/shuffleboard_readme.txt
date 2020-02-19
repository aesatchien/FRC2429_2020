Trying to speed up smartdashboard, because the thing never really saves correctly.
Also trying to get rid of grey values in the the command boxes - seems like they grey if different than in boot up?  Need to look into this.

Look for this type of thing:
      "title": "SmartDashboard",
      "autoPopulate": true,
      "autoPopulatePrefix": "SmartDashboard/",
	  
	  
1) set the autopopulate to false - for the live window and the smartdashboard.

The advice on running the jar straight vs the script doesn't seem to make much sense at first, but here it is anyway:

"%JAVA_HOME%\bin\java" -Dsun.net.client.defaultConnectTimeout=2 -jar "C:\Users\Public\wpilib\2020\tools\shuffleboard.jar"

The great thing is that if you run this in a cmd window you get all the error messages - like these: 
NT: ERROR: could not resolve frcvision.local address (TCPConnector.cpp:99)
WARNING: Saved source type is not present, adding destroyed source(s) instead
WARNING: Uncaught exception on JavaFX Application Thread
java.lang.ClassCastException: class com.google.gson.internal.LazilyParsedNumber cannot be cast to class java.lang.Integer (com.google.gson.internal.LazilyParsedNumber is in unnamed module of loader 'app'; java.lang.Integer is in module java.base of loader 'bootstrap')

It does create a log of the above that tells you how long it takes to load everything, and probably some other settings in the %userprofile%/Shuffleboard director - usually C:\Users\username
Try deleting (back up if you need it after) %userprofile%/Shuffleboard then opening ShuffleBoard again.


Other possibilities / suggestions from the web: (search delphi):
edu.wpi.first.shuffleboard.plugin.networktables entry under Computer\HKEY_CURRENT_USER\Software\JavaSoft\Prefs\edu\wpi\first\shuffleboard\plugin\networktables
sets the server - so I wonder if that can get screwed up when you play around with test servers
You can change this value in the file preferences menu

Prefs are here - autoload last save file and location of save file:
Computer\HKEY_CURRENT_USER\Software\JavaSoft\Prefs\edu\wpi\first\shuffleboard\app\prefs

BIGGEST IMPACT:
Kill the livewindow widgets.  The shuffleboard becomes far more robust and only takes a few seconds (6ish) to reconnect to a rebooted robot.


