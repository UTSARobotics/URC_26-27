In Administrative PowerShell 
```powershell
winget install dorssel.usbipd-win
usbipd list
```
Will get something like 
```
PS C:\WINDOWS\system32> usbipd list
Connected:
BUSID  VID:PID    DEVICE                                                        STATE
1-3    0489:e0f6  MediaTek Bluetooth Adapter                                    Not shared
1-4    0b05:19b6  USB Input Device                                              Not shared
1-5    0b05:193b  USB Input Device                                              Not shared
1-7    8086:0b07  Intel(R) RealSense(TM) Depth Camera 435 with RGB Module D...  Not shared
4-1    3277:0051  USB2.0 FHD UVC WebCam, USB2.0 IR UVC WebCam, Camera DFU D...  Not shared
```
Search for the RealSense Device and copy the BUSID tag
```
usbipd bind --busid <BUSID>
usbipd attach --wsl --busid <BUSID>
```
You should then be able to use the real-sense camera in
```
realsense-viewer
```