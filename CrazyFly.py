#!/usr/bin/env python
import signal
import sys
from AbstractVirtualCapability import AbstractVirtualCapability, VirtualCapabilityServer, formatPrint


class CrazyFly(AbstractVirtualCapability):
    def __init__(self, server):
        super().__init__(server)
        self.copterPosition = [0., 0., 0.]
        self.functionality = {"arm": None, "disarm": None, "SetCopterPosition": None, "GetCopterPosition": None,
                              "GetArmingStatus": None, "setNeoPixelColor": None}

    def SetArmingStatus(self, params: dict):
        formatPrint(self, f"Set Arming Status to {params}")
        p = params["SimpleBooleanParameter"]
        if p and self.functionality["arm"] is not None:
            self.functionality["arm"]()
        elif not p and self.functionality["disarm"] is not None:
            self.functionality["disarm"]()
        return params

    def GetArmingStatus(self, params: dict):
        if self.functionality["GetArmingStatus"] is not None:
            return {"SimpleBooleanParameter": self.functionality["GetArmingStatus"]()}
        return {"SimpleBooleanParameter": False}

    def SetCopterPosition(self, params: dict) -> dict:
        try:
            p = params["Position3D"]
        except:
            return self.GetCopterPosition(params)
        formatPrint(self, f"Flying to {p}")
        if self.functionality["SetCopterPosition"] is not None:
            self.functionality["SetCopterPosition"](p)
        else:
            pass
        return self.GetCopterPosition({})

    def GetCopterPosition(self, params: dict) -> dict:
        if self.functionality["GetCopterPosition"] is not None:
            pos = self.functionality["GetCopterPosition"]()
            self.copterPosition = pos
        return {"Position3D": self.copterPosition}

    def FlyToPosition(self, params: dict) -> dict:
        formatPrint(self, f"Flying to position {params}")
        return self.SetCopterPosition(params)
        
    def setNeoPixelColor(self, params: dict) -> dict:
        if self.functionality["GetCopterPosition"] is None:
            return {}
        r = params["Red"]
        g = params["Green"]
        b = params["Blue"]
        deviceID = params["DeviceID"]
        if deviceID == "CrazyFly":
            if r is not None and g is not None and b is not None:
                self.functionality["setNeoPixelColor"](r, g, b)

    def loop(self):
        pass


if __name__ == "__main__":
    try:
        port = None
        if len(sys.argv[1:]) > 0:
            port = int(sys.argv[1])
        server = VirtualCapabilityServer(port)
        cf = CrazyFly(server)
        cf.start()


        def signal_handler(sig, frame):
            cf.kill()
            server.kill()


        signal.signal(signal.SIGINT, signal_handler)
        cf.join()
        server.join()
        signal.pause()

        # Needed for properly closing, when program is being stopped with a Keyboard Interrupt
    except KeyboardInterrupt:
        print("[Main] Received KeyboardInterrupt")
