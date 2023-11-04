-- Start capture.
-- We never intend to actually carry out this collection: we `os.exit()` and
-- allow the python data collection process to "steal" the radar sockets.
ar1.CaptureCardConfig_StartRecord(
    "C:\\ti\\mmwave_studio_02_01_01_00\\mmWaveStudio\\PostProc\\adc_data.bin", 1)
ar1.StartFrame()
os.exit()
