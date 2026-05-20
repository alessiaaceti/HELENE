# Switched from int32 to float64 to prevent data truncation and overflow
# This ensures high-resolution signal integrity for Force Feedback 
# and prevents jitter in the control loop caused by integer rounding
float64 joint1
float64 joint2
float64 joint3
float64 joint4
float64 joint5
float64 joint6