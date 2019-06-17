FRC Team 5708's camera and vision code

#Streaming how to

Launch the start_steaming.sh script from a unix shell (WSL works). 
There are two environment variables: PI_ADDR and BITRATE (self-explainatory).
There's a bug where it will either exit or hang on launch and need to be relaunched.
If the first GStreamer window doesn't open within 10 seconds, kill it with ctrl-C and relaunch it.
The second window (if second camera is plugged in) takes longer to open. Wait at least 30 seconds for it.