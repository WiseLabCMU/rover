--    ______  _____  _    _ _______  ______
--   |_____/ |     |  \  /  |______ |_____/
--   |    \_ |_____|   \/   |______ |    \_
--   Radar Control Server for mmWave Studio
--

-- Initialize mmWave Studio ---------------------------------------------------

-- Taken from TI-supplied code. These lines call the default initialization
-- script which usually runs when mmWave studio starts.

-- mmwavestudio installation path
RSTD_PATH = RSTD.GetRstdPath()
-- Declare the loading function
dofile(RSTD_PATH .. "\\Scripts\\Startup.lua")

-- Initialize Radar -----------------------------------------------------------

ar1.FullReset()
ar1.SOPControl(2)

ar1.Connect({com}, {baudrate}, 1000)

ar1.Calling_IsConnected()
ar1.deviceVariantSelection("XWR1843")
ar1.frequencyBandSelection("77G")
ar1.SelectChipVersion("XWR1843")
ar1.DownloadBSSFw("{mmwave_studio}\\{bss_firmware}")
ar1.GetBSSFwVersion()
ar1.GetBSSPatchFwVersion()
ar1.DownloadMSSFw("{mmwave_studio}\\{mss_firmware}")
ar1.GetMSSFwVersion()
ar1.PowerOn(0, 1000, 0, 0)
ar1.RfEnable()
ar1.ChanNAdcConfig(1, 1, 1, 1, 1, 1, 1, 2, 2, 0)
ar1.LPModConfig(0, 1)
ar1.RfInit()
ar1.SetCalMonFreqLimitConfig(77,81)
ar1.DataPathConfig(513, 1216644097, 0)
ar1.LvdsClkConfig(1, 1)
ar1.LVDSLaneConfig(0, 1, 1, 0, 0, 1, 0, 0)
ar1.ProfileConfig(
    0, 77, 10, 6, 63.14, 0, 0, 0, 0, 0, 0, 63.343, 1, 512, 9121, 0, 0, 30)
ar1.ChirpConfig(0, 0, 0, 0, 0, 0, 0, 1, 0, 0)
-- ar1.ChirpConfig(1, 1, 0, 0, 0, 0, 0, 0, 1, 0)
ar1.ChirpConfig(1, 1, 0, 0, 0, 0, 0, 0, 0, 1)
ar1.DisableTestSource(0)
ar1.FrameConfig(0, 1, 0, 1, 1, 0, 0, 1)
ar1.GetCaptureCardDllVersion()
ar1.SelectCaptureDevice("DCA1000")
ar1.CaptureCardConfig_EthInit(
    "{static_ip}", "192.168.33.180", "12:34:56:78:90:12",
    {config_port}, {data_port})
ar1.CaptureCardConfig_Mode(1, 2, 1, 2, 3, 30)
ar1.CaptureCardConfig_PacketDelay(25)
ar1.GetCaptureCardFPGAVersion()

print("Initialization complete.")

-- Server ---------------------------------------------------------------------

function read()
    local file = io.open("{msgfile}", "r")
    if not file then
        return nil, nil
    end
    local msg = file:read("*a")
    file:close()
    os.remove("{msgfile}")
    return msg
end

running = false
while true do
    msg = read()
    if msg == "start" then
        if running == false then
            print("Starting capture...")
            ar1.CaptureCardConfig_StartRecord("{tmpfile}", 1)
            ar1.StartFrame()
            running = true
        else
            print("Tried to start an already-running radar.")
        end
    elseif msg == "stop" then
        if running == true then
            print("Stopping capture...")
            ar1.StopFrame()
            running = false
        else
            print("Tried to stop an already-stopped radar.")
        end
    elseif msg == "exit" then
        print("Exiting...")
        os.exit()
    end
end
