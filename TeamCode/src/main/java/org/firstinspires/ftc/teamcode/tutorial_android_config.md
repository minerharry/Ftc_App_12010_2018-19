## Android configuration
The android phones for FTC use xml configuration files to store their configs. These are stored on
the robot controller phone, and can be found in the FIRST folder,

## Making your own config
# Step 1: Getting the file
in order to make a config file of your own in android studio, you should first create a file on the
phone itself that connects to all of the hubs and all of the complex hardware (webcams, anything
that's notmotors and servos and things). Then, you will need to retrieve the file and put it in
android studio. After plugging the phone int the computer with a good data cable, you will be able
to see the phone files - but no files in the phone. To fix this, go to developer settings and the
last option should be 'Select USB Configuration'. You want it to be MTP (Media Transfer Protocol),
but even if it is already MTP, you will probably have to select it again for it to register. From
the phone files, navigate to [Phone Name] -> FIRST -> [your config name]. Copy that file into
Teamcode -> res -> xml folder. WARNING: Even though on the phone you can name them with certain
characters like uppercase letters, to be registered they need to not be uppercase, and to be safe
you should just use lowercase + underscores.

# Step 2: Editing the file
The XML files are in a particular hierarchy of portal -> hubs -> hardware, as better explained at
https://www.sites.google.com/site/blackdiamondrobotics/resources/configuration-files. The variables
in the xml are fairly self explanatory: the name="" is the name used in the HardwareMap.get() method,
and the port= is which port of the revhub to which that particular piece of hardware is attached.
Most of it is pretty simple; copy and paste elements if you choose. If you have a piece of hardware
you haven't used before and you already have a custom config made, make a new separate config on the phone
with whatever you need to add and copy it from that file to the official one.
