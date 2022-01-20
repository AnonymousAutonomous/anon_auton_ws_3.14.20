from flask import Flask, request, redirect, url_for, render_template
import subprocess

import serial
from urllib.parse import urlsplit

app = Flask(__name__)

@app.route("/")
def index():
    return render_template("index.html")


@app.route("/test")
def test():
    return "<p>test</p>"


# POST with form: 
#    message: string        what is sent to the chairs
#    chair_num: int         what number is sent to the chairs (filters who should listen to it)
#    all_chairs: bool       whether or not all the chairs should act on the message
@app.route('/transmit', methods=['POST'])
def transmit():
    data = request.form
    print(data)
    print(request.base_url)
    print(urlsplit(request.base_url))
    send_to_antenna(data['message'])
    return redirect(url_for('index'))
    # if request.method == 'POST':
    #     return do_the_login()
    # else:
    #     return show_the_login_form()


def send_to_antenna(message):
    print('sending to antenna')
    antenna_port = '/dev/ttyUSB0'

    subprocess.check_output("rostopic pub -1 /from_hub std_msgs/String " + message, shell=True)



    # ser = serial.Serial(antenna_port, 57600)

    # ser.write((message + '\n').encode())

    # rostopic pub -1 something something something
