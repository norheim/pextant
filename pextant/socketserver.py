from flask import Flask, render_template
from flask_socketio import SocketIO, send, emit
import time
app = Flask(__name__)
socketio = SocketIO(app)

@socketio.on('connect')
def handle_connection():
    print('greetings')
    socketio.emit('event', 'my first greetings')
    #while True:
    #    time.sleep(1)
    #    socketio.emit('event','greetings')

app.run(host='localhost', port=3000)