-- Start capture.
-- We never intend to actually carry out this collection: we allow the python
-- data collection process to "steal" the radar sockets.
ar1.CaptureCardConfig_StartRecord(
    "C:\\Users\\dart\\tmp.bin", 1)
ar1.StartFrame()
