#!/usr/bin/env python

from sensor_msgs.srv import SetCameraInfoRequest, SetCameraInfoResponse
from myproject.srv import BatResponse, BatRequest, BazRequest, BazResponse

cam_req = SetCameraInfoRequest()
cam_rep = SetCameraInfoResponse()

bat_req = BatRequest()
bat_rep = BatResponse()

baz_req = BazRequest()
baz_rep = BazResponse()
