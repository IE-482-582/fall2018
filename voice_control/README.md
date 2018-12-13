## Sample Code for Speech-to-Test

The `Blather.py` script (see https://github.com/ajbogh/blather for the original source) will convert speech to text, and then looks for matching words/phrases from a configuration file. 

This version of the script does not use ROS.  

### How to run the current version of the script:
This is a high-level overview.  You'll have to study the code to understand the details.

1. Use your file browser to find the `commands.conf` file.
2. Edit this file to include new commands.  
3. Change directories (`cd`) to the location of `Blather.py`.  Now, run Blather with the following command:
   ```
   ./Blather.py
   ```
4. Blather will hopefully recognize the words that you speak.  For example, try saying "hello world", or "say hi".

NOTE:  If you run Blather like this:
```
./Blather.py -c
```
you'll have to say "Vader" before each command.  Otherwise, Blather will ignore what you're saying.  For example, "hello world" will be ignored, but "Vader hello world" will print "hello world" to the screen.  If you want to change the name, edit `commands.conf` and replace "Vader" with another name.


