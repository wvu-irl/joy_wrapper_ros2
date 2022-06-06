# joy_wrapper_ros2

Purpose is to remap joy inputs to specific buttons, accessed by name. This also gives users access to 
- raw feed
- toggles
- increment counters
- hold (to keep an axis at a setpoint)
- time in a given state (ns)
- rising edge detection
- falling edge detection
- double click detection (note these string together so 3 fast clicks counts as 2 double clicks, may debug later, can avoid this with larger sensitivity)

Project -> joy_wrapper

Users should launch 

``` 
ros2 launch joy_wrapper joy_wrapper_launch.py
```

Subscribe to 
```
\joy_wrapper
```

