# ChickenBot Bluetooth Controller

To run the controller, first install the required libraries with the following command:

```bash
python3 -m pip install -r requirements.txt
```

Then, pair with the ESP32 via your computer's Bluetooth settings and run the command below to run the controller:

```bash
python3 controller.py <BT device port> <joystick preference>
```

For Mac users, the BT device port should be of the form `/dev/cu.ESP32` and can be verified by checking the presence of the file with the command `ls /dev/`. For Windows users, find the COM port of the BT device using Device Manager and specify the device port as COMX, where X is the COM port number.
