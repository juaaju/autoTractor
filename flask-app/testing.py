from flask import Flask, request, jsonify
import time
from flask_cors import CORS  # Import CORS

app = Flask(__name__)

# Menambahkan CORS di seluruh aplikasi
CORS(app)

@app.route('/process_data', methods=['POST'])
def process_data():
    data = request.get_json()

    tractor_position = data['tractorPosition']
    field_points = data['fieldPoints']

    # Proses data
    time.sleep(5)  # Simulasi pengolahan data

    # Kembalikan respons setelah selesai
    return jsonify({"message": "Data telah diproses."}), 200

if __name__ == "__main__":
    app.run(debug=True)
