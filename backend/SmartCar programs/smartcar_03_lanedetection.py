import smartcar

sc = smartcar.SmartCar(use_local_window = False, use_socket = True)
sc.lane_detection_loop()
# It is important for cleaning up to set the SmartCar object to None
sc = None
