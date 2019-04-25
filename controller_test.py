import inputs
import threading
import time

x = 0
y = 0


def gamepad_handler():
	global x, y
	while True:
		events = inputs.get_gamepad()
		for event in events:
			# print(event.ev_type, event.code, event.state)
			if event.code == "ABS_X":
				x = int(event.state) if abs(event.state) > 8000 else 0
			if event.code == "ABS_Y":
				y = int(event.state) if abs(event.state) > 8000 else 0


handler_thread = threading.Thread(target=gamepad_handler)
handler_thread.start()
while True:
	print(str(x) + " : " + str(y))
	time.sleep(0.1)
