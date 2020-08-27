import RPi.GPIO as GPIO
import smbus
import time
import sys
import smbus
import pymysql
from flask import Flask,request,jsonify,render_template, Response
from camera import Camera

conn = pymysql.connect(host="localhost",
						user="Iot_user",
						passwd="raspberry",
						db="Iot_db")


sht20_addr = 0x40
SHT20_CMD_MEASURE_TEMP = 0xf3
SHT20_CMD_MEASURE_HUMI = 0xf5
SHT20_SOFT_RESET = 0xfe
bus = smbus.SMBus(1)
bus.write_byte(sht20_addr, SHT20_SOFT_RESET)
time.sleep(0.05)
data = [0, 0]

led1 = 20
led2 = 21

e = 12
p = 4
n = 25
speed = 10
GPIO.setmode(GPIO.BCM)
GPIO.setup(led1, GPIO.OUT)
GPIO.setup(led2, GPIO.OUT)

GPIO.setup(e,GPIO.OUT)
GPIO.setup(p,GPIO.OUT)
GPIO.setup(n,GPIO.OUT)


pwm = GPIO.PWM(p, speed)


app = Flask(__name__)

@app.route('/')
def index():
	return render_template('index.html',state = Iot_state())

def Iot_state():
	state = None
	state_sql="select * from Iot_state"
	cur = conn.cursor()
	cur.execute(state_sql)
	state= cur.fetchall()
	print(state[0])
	return state


def gen(camera):
    while True:
        frame = camera.get_frame()
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')

@app.route('/video_feed')
def video_feed():
    return Response(gen(Camera()),
                    mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/cctvCont', methods=['POST'])
def cctvCont():
	servo_addr = 0x20

	OUT_PORT0 = 0x02
	OUT_PORT1 = 0x03
	CONFIG_PORT0 = 0x06
	CONFIG_PORT1 = 0x07
	POLARITY_IVE_PORT0 = 0x04
	POLARITY_IVE_PORT1 = 0x05

	servoUD = 0x01
	servoLR = 0x02
	Phase_1 =  [0x01, 0x02, 0x03, 0x04]
	bus = smbus.SMBus(1)
	
	way = request.form['way'] 
	if(way == "up"):
		bus.write_byte_data(servo_addr, CONFIG_PORT0, 0x02)
		bus.write_byte_data(servo_addr, CONFIG_PORT1, 0x03)
		for i in range(2):
			bus.write_byte_data(servo_addr, 0x02, Phase_1[i])
	elif(way == "down"):
		bus.write_byte_data(servo_addr, CONFIG_PORT0, 0x02)
		bus.write_byte_data(servo_addr, CONFIG_PORT1, 0x03)
		for i in range(2):
			bus.write_byte_data(servo_addr, 0x02, Phase_1[i])
			time.sleep(0.0015)
	elif(way == "left"):
		bus.write_byte_data(servo_addr, CONFIG_PORT0, 0x01)
		bus.write_byte_data(servo_addr, CONFIG_PORT1, 0x02)
		for i in range(4):
			bus.write_byte_data(servo_addr, 0x02, Phase_1[i])
			time.sleep(0.0015)
	elif(way == "right"):
		bus.write_byte_data(servo_addr, CONFIG_PORT0, 0x01)
		bus.write_byte_data(servo_addr, CONFIG_PORT1, 0x02)
		for i in range(4):
			bus.write_byte_data(servo_addr, 0x02, Phase_1[i])

	return ""
@app.route('/ledOnOff', methods=['POST'])
def ledOnOff():

	led = request.form['led']
	if(led == "20"):
		led_sql="update Iot_state set led1 = %s"
	else:
		led_sql="update Iot_state set led2 = %s"

	action = request.form['action']

	if(action == "on"):
		GPIO.output(int(led), GPIO.HIGH)
		action = "off";
	elif(action == "off"):
		GPIO.output(int(led), GPIO.LOW)
		action = "on"

	cur = conn.cursor()
	cur.execute(led_sql,(action))
	conn.commit()

	result = [led,action]
	return jsonify(result)
	conn.close()

@app.route('/temp', methods=['POST'])
def temp():
	power = request.form['power']
	air_temp = request.form['air_temp']
	autoP = request.form['autoP']
	auto_temp = request.form['auto_temp']

	auto_sql = "update Iot_state set air_cont = %s, air_temp = %s, auto_cont = %s, auto_temp = %s"
	cur = conn.cursor()
	bus.write_byte(sht20_addr, SHT20_CMD_MEASURE_TEMP)
	time.sleep(0.26)
	for i in range(2):
		data[i] = bus.read_byte(sht20_addr)
	value = data[0] << 8 | data[1]
	temp = -46.84 + 175.72 / 65536 * int(value)
	bus.write_byte(sht20_addr, SHT20_CMD_MEASURE_HUMI)
	time.sleep(0.26)
	for i in range(2):
		data[i] = bus.read_byte(sht20_addr)
	value = data[0] << 8 | data[1]
	humi = -6.0 + 125.0 / 65536 * int(value)

	if(autoP == "off"):
		speed = (26 - int(auto_temp))*10
		if(int(auto_temp) < int(temp)):
			pwm.start(10)
			GPIO.output(e, GPIO.HIGH)
			GPIO.output(p, GPIO.HIGH)
			GPIO.output(n, GPIO.LOW)
			pwm.ChangeDutyCycle(speed)
			power ="off"
			air_temp = auto_temp
		else:
			GPIO.output(e, GPIO.LOW)
			GPIO.output(p, GPIO.LOW)
			GPIO.output(n, GPIO.LOW)
			power = "on"

	cur.execute(auto_sql,(power, air_temp, autoP, auto_temp))
	conn.commit()

	result = [round(temp,2),round(humi,2),power, air_temp, autoP]

	return jsonify(result)
	conn.close()
 
@app.route('/air', methods=['POST'])
def air():
	power = request.form['power']
	powerS = request.form['powerS']
	air_temp = request.form['air_temp']


	air_sql = "update Iot_state set air_cont = %s, air_temp = %s"
	if(power == "on"):
		pwm.start(10)
		GPIO.output(e, GPIO.HIGH)
		GPIO.output(p, GPIO.HIGH)
		GPIO.output(n, GPIO.LOW)
		power ="off"
	elif(power == "off"):
		GPIO.output(e, GPIO.LOW)
		GPIO.output(p, GPIO.LOW)
		GPIO.output(n, GPIO.LOW)
		power = "on"

	speed = (26 - int(air_temp))*10
	if(power == "nope"):
		power = powerS
	cur = conn.cursor()
	cur.execute(air_sql,(power, air_temp))
	conn.commit()

	pwm.ChangeDutyCycle(speed)
	print("air")
	result = power
	return jsonify(result)
	conn.close()
      

if __name__ == '__main__':
    app.run(host='0.0.0.0', port = '8011', debug=True, threaded=True)
