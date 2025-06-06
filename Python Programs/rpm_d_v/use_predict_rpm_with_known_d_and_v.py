import joblib

# Load the model in another file
model = joblib.load('rpm_prediction_model.pkl')
# Load in another file
scaler = joblib.load('scaler.pkl')


while True:
    # Sample input
    sample_distance = float(input("Enter Distance(m): "))
    sample_voltage = float(input("Enter Voltage(v): "))
    sample_input = scaler.transform([[sample_distance, sample_voltage]])
    # Ensure RPM is non-negative
    predicted_rpm = max(model.predict(sample_input)[0], 0)

    # Use the loaded model
    predicted_rpm = model.predict(sample_input)

    # print(predicted_rpm)
    print(
        f"Predicted RPM for Distance={sample_distance}m and Voltage={sample_voltage}V: {predicted_rpm[0]:.2f}")
