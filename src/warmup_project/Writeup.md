WarmUp WriteUp Project
==============
**Allie Duncan - CompRobo 2014**


I implemented wall following and person following behavior. 

**Wall following strategy**
As suggested in class, my Neato collected data ± 10° around 45° and 135°. These were averaged together to get two floats - for 45° and 135°. Both of these would ideally be √2 when the Neato is 1 meter away and parallel to the wall. When these averages are not √2, I adjusted the z angle sent to the Neato appropriately. One issue I experienced was sometimes both the 45° and 135° data sets returned 0. Therefore, I also collected data at ± 10 around 90°. When the 45° and 135° data failed, the Neato used the 90° data to navigate until normal data collection worked. When it used this “alternate” method, it moved extra slowly so as not to move too quickly to or from the wall.

**Person following strategy**
I originally tried to implement person following as motion following. I had a buffer of past scans data and noted the angle of any major changes. However, this ended up failing. I think this was due to my implementation because after initially successfully detecting motion, as I added more code, it became too slow. Therefore, I changed to a simple strategy. My person follow simply follows the closest object. It takes all 360° data and replaces any data above 7, below .2, or 0 with 1000. It then finds the minimum of that data and turns towards that angle. The Neato uses the same strategy of wall approach used in class to move forward/backwards appropriately. 

**Finite State Controller**
I combined the wall follow and person follow behaviors. The state starts with wall follow behavior and transitioned to person follow behavior if there is an object closer than 0.4 m directly in front of the Neato. The Neato then transitions back to wall follow behavior if there is no object closer to it than 1 m. (This happens when the person runs quickly away.) 

**Code Structure**
Because I was valuing speed during this project, my code is basically just loops and if statements. In combined.py (which contains almost verbatim code from wallfollow.py and personfollow.py but has both behaviors), there are two main functions - scan_received() and wall(). Scan_received checks what behavior is should be run and then collects the necessary data for that behavior. Wall checks what behavior should be run and then sends appropriate linear x and angular z values to the Neato.

**Challenges**
Data is always the key with robotics. I struggled a lot on the wall behavior because I was only collecting data in small segments, which were then filtered to give me little or no data. Person follow worked much better because I collected all of the data and then filtered any unreasonable points. Simple filtering is key here - don’t do unnecessary work but figure out initially what data is legitimate and what isn’t. 

**Future Improvements**
I’d like to incorporate motion follow with my new 360-data collection and simple filtering techniques.

**Lessons for Future Robotics**
Start simple. Seriously.
