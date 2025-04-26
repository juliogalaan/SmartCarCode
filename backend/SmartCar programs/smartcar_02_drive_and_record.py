import smartcar

sc = smartcar.SmartCar(use_local_window = False, use_socket = True)
sc.drive_and_record_loop()
# It is important for cleaning up to set the SmartCar object to None
sc = None
