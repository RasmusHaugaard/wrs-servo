from ur_control import Robot

for ip in 129, 131:
    Robot.from_ip(f'192.168.1.{ip}').ctrl.teachMode()
