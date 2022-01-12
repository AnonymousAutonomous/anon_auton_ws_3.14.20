from flask import Flask, request, redirect, url_for, render_template

import serial

app = Flask(__name__)

@app.route("/")
def hello_world():
    render_template("index.html")
    data = request.form
    print(data)
    if request.method == 'GET':
        return render_template("index.html")
    else: 
        print("POST!")
        return redirect(url_for('transmit'))


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
    send_to_antenna(data['message'])
    return redirect(url_for('test'))
    # if request.method == 'POST':
    #     return do_the_login()
    # else:
    #     return show_the_login_form()


def send_to_antenna(message):
    print('sending to antenna')
    antenna_port = '/dev/ttyUSB0'
    ser = serial.Serial(antenna_port, 57600)

    ser.write(message + '\n')
