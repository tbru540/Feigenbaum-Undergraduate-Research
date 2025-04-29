from flask import Flask, request, jsonify, render_template
from flask_sqlalchemy import SQLAlchemy
from flask_cors import CORS
from datetime import datetime, timezone
import os

app = Flask(__name__)
CORS(app)

# SQLite Database Configuration
basedir = os.path.abspath(os.path.dirname(__file__))
app.config['SQLALCHEMY_DATABASE_URI'] = 'sqlite:///' + os.path.join(basedir, 'sensor_data.db')
app.config['SQLALCHEMY_TRACK_MODIFICATIONS'] = False
db = SQLAlchemy(app)

# Database table model for storing sensor readings 
class SensorData(db.Model):
    id = db.Column(db.Integer, primary_key=True)
    temperature = db.Column(db.Float)
    moisture = db.Column(db.Float)
    humidity = db.Column(db.Float)
    ec = db.Column(db.Float)
    ph = db.Column(db.Float)
    pressure = db.Column(db.Float)
    lux = db.Column(db.Float)
    proximity = db.Column(db.Float)
    timestamp = db.Column(db.DateTime, default=lambda: datetime.now(timezone.utc))     # Time in UTC timezone

with app.app_context():
    db.create_all()

# Render the dashboard.html
@app.route('/')
def dashboard():
    return render_template('dashboard.html')

# Receive Data
@app.route('/sensor-data', methods=['POST'])
def receive_sensor_data():
    data = request.get_json()

    new_data = SensorData(
        temperature=data.get('temperature', 0),
        moisture=data.get('moisture', 0),
        humidity=data.get('humidity', 0),
        ec=data.get('ec', 0),
        ph=data.get('ph', 0),
        pressure=data.get('pressure', 0),
        lux=data.get('light', 0),
        proximity=data.get('proximity', 0)
    )

    db.session.add(new_data)
    db.session.commit()

    return jsonify({"status": "saved"})

# Get Last 100 Data Entries 
@app.route('/data', methods=['GET'])
def get_data():
    records = SensorData.query.order_by(SensorData.timestamp.desc()).limit(100).all()
    result = [{
        "temperature": r.temperature,
        "moisture": r.moisture,
        "humidity": r.humidity,
        "ec": r.ec,
        "ph": r.ph,
        "pressure": r.pressure,
        "lux": r.lux,
        "proximity": r.proximity,
        "timestamp": r.timestamp.strftime('%Y-%m-%dT%H:%M:%SZ')
    } for r in reversed(records)]  # Reversed = oldest → newest

    return jsonify(result)

# Run Application
if __name__ == '__main__':
    app.run(host='0.0.0.0', port=80)
