import usb_cdc
import supervisor
usb_cdc.enable(console=True,data=True) 
supervisor.autoreload = False
supervisor.disable_autoreload()