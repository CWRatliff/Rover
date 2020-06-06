msg: 09:35:56 {D0}
Stopped
Traceback (most recent call last):
  File "/home/pi/bttyrover.py", line 361, in <module>
    simple_commands(xchr)
  File "/home/pi/bttyrover.py", line 169, in simple_commands
    robot.motor(speed, steer)
  File "/home/pi/motor_driver_ada.py", line 163, in motor
    self.lr_motor.angle = self.lrbias
  File "/usr/local/lib/python3.7/dist-packages/adafruit_motor/servo.py", line 123, in angle
    self.fraction = new_angle / self.actuation_range
  File "/usr/local/lib/python3.7/dist-packages/adafruit_motor/servo.py", line 71, in fraction
    self._pwm_out.duty_cycle = duty_cycle
  File "/usr/local/lib/python3.7/dist-packages/adafruit_pca9685.py", line 96, in duty_cycle
    self._pca.pwm_regs[self._index] = (0, value)
  File "/usr/local/lib/python3.7/dist-packages/adafruit_register/i2c_struct_array.py", line 77, in __setitem__
    i2c.write(buf)
  File "/usr/local/lib/python3.7/dist-packages/adafruit_bus_device/i2c_device.py", line 98, in write
    self.i2c.writeto(self.device_address, buf, **kwargs)
  File "/home/pi/.local/lib/python3.7/site-packages/busio.py", line 84, in writeto
    return self._i2c.writeto(address, buffer, stop=stop)
  File "/home/pi/.local/lib/python3.7/site-packages/adafruit_blinka/microcontroller/generic_linux/i2c.py", line 38, in writeto
    self._i2c_bus.write_bytes(address, buffer[start:end])
  File "/home/pi/.local/lib/python3.7/site-packages/Adafruit_PureIO/smbus.py", line 256, in write_bytes
    self._device.write(buf)
OSError: [Errno 121] Remote I/O error
>>> %Run bttyrover.py
Rover 1.0 200421

